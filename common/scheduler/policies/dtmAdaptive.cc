/**
 * dtmAdaptive.cc
 *
 * Implementation of the Priority-Aware Adaptive Thermal Management (DTM)
 * policy for SchedulerCFSLite in CoMeT.
 *
 * Algorithmic flow per core per DTM tick:
 *
 *  Phase A — Thermal check:
 *    T < T_warn   → skip (no action needed)
 *    T > T_crit   → EMERGENCY: DVFS to min_freq
 *
 *  Phase B — Workload classification (T_warn ≤ T ≤ T_crit):
 *    Memory-bound (util < 0.6):
 *      → DRAM_MODE: switch associated banks to low-power (high-latency) mode.
 *        Raises bank latency from ~15 ns to ~600 ns, creating back-pressure
 *        that limits the core's effective memory bandwidth.
 *        Banks are restored to normal mode once the core cools below T_warn.
 *    Compute-bound:
 *      Branch 1: Low-priority,  non-empty run-queue     → YIELD
 *      Branch 2: Low-priority,  empty run-queue         → DVFS  (f -= step)
 *      Branch 3: High-priority, thrashing (T > T_warn+3)→ MIGRATE to coolest
 * idle core Branch 4: High-priority, not thrashing           → DVFS on vertical
 * neighbor (steal-throttle: cool LP neighbor to act as a thermal heat-sink)
 */

#include "dtmAdaptive.h"
#include "misc/stats.h"
#include "simulator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

DtmAdaptive::DtmAdaptive(const PerformanceCounters *perf_counters,
                         int num_cores, int cores_in_x, int cores_in_y,
                         int num_banks, int num_channels, float core_t_warn,
                         float core_t_crit, float mem_t_warn,
                         float mem_t_crit, int min_freq_mhz,
                         int max_freq_mhz,
                         int freq_step_mhz, int k_max, float slack_scale,
                         float mem_intensity_threshold, float mpki_threshold,
                         int freq_history_len)
    : m_perf(perf_counters), m_num_cores(num_cores), m_cores_in_x(cores_in_x),
      m_cores_in_y(cores_in_y), m_num_banks(num_banks),
      m_num_channels(std::max(1, num_channels)),
      m_banks_per_channel(std::max(1, num_banks / std::max(1, num_channels))),
      m_cores_per_channel(std::max(1, num_cores / std::max(1, num_channels))),
      m_core_t_warn(core_t_warn), m_core_t_crit(core_t_crit),
      m_mem_t_warn(mem_t_warn), m_mem_t_crit(mem_t_crit),
      m_min_freq(min_freq_mhz),
      m_max_freq(max_freq_mhz), m_freq_step(freq_step_mhz), m_k_max(k_max),
      m_slack_scale(slack_scale),
      m_mem_intensity_threshold(mem_intensity_threshold),
      m_mpki_threshold(mpki_threshold),
      m_freq_history_len(std::max(1, freq_history_len)),
      m_bank_throttled(num_banks, false), m_freq_history(num_cores) {
  m_prev_instr.resize(num_cores, 0);
  m_prev_miss.resize(num_cores, 0);
  m_prev_mpki.resize(num_cores, 0.0);
  m_recently_yielded_tid.resize(num_cores, -1);

  m_log.open("dtm_adaptive.log");
  if (!m_log.is_open()) {
    std::cerr << "[DTM-Adaptive] WARNING: Failed to open dtm_adaptive.log!"
              << std::endl;
  }

  m_epoch_log.open("dtm_epoch.log");
  if (!m_epoch_log.is_open()) {
    std::cerr << "[DTM-Adaptive] WARNING: Failed to open dtm_epoch.log!"
              << std::endl;
  }

  if (m_log.is_open())
    m_log << "[DTM-Adaptive] initialized"
          << " core_t_warn=" << m_core_t_warn
          << " core_t_crit=" << m_core_t_crit
          << " mem_t_warn=" << m_mem_t_warn
          << " mem_t_crit=" << m_mem_t_crit
          << " min_freq=" << m_min_freq << " MHz"
          << " num_banks=" << m_num_banks << " channels=" << m_num_channels
          << " k_max=" << m_k_max << " mpki_threshold=" << m_mpki_threshold
          << " freq_history_len=" << m_freq_history_len << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * isMemoryBound
 *
 * Classifies a core as memory-bound using MPKI (Misses Per Kilo Instruction).
 *
 * MPKI = (delta_cache_misses / delta_instructions) * 1000
 *
 * This uses interval miss and instruction deltas, scaled per thousand
 * instructions. A high MPKI means the core stalls heavily on memory and is
 * memory-bound.  The threshold (m_mpki_threshold, e.g., 10.0) is configurable
 * in base.cfg via scheduler/cfs_lite/dtm/adaptive/mpki_threshold.
 *
 * Falls back to the utilization heuristic when miss metrics are unavailable
 * or when the current sample has too few instructions.
 */
bool DtmAdaptive::isMemoryBound(int core_id) {
  if (!m_perf)
    return false;
  if (core_id < 0 || core_id >= m_num_cores)
    return false;

  // 1. Fetch current raw counters
  StatsManager *stats = Sim()->getStatsManager();
  StatsMetricBase *instr_metric =
      stats->getMetricObject("performance_model", core_id, "instruction_count");
  uint64_t instr = instr_metric ? instr_metric->recordMetric() : 0;

  // Try L3 first; if not present, fallback to L2
  StatsMetricBase *miss_load =
      stats->getMetricObject("L3", core_id, "load-misses");
  StatsMetricBase *miss_store =
      stats->getMetricObject("L3", core_id, "store-misses");
  if (!miss_load) {
    miss_load = stats->getMetricObject("L2", core_id, "load-misses");
    miss_store = stats->getMetricObject("L2", core_id, "store-misses");
  }
  const bool have_miss_metrics = (miss_load != NULL) || (miss_store != NULL);

  uint64_t miss = 0;
  if (miss_load)
    miss += miss_load->recordMetric();
  if (miss_store)
    miss += miss_store->recordMetric();

  // 2. Compute deltas
  uint64_t d_instr =
      instr > m_prev_instr[core_id] ? instr - m_prev_instr[core_id] : 0;
  uint64_t d_miss =
      miss > m_prev_miss[core_id] ? miss - m_prev_miss[core_id] : 0;

  // 3. Fast-epoch robust MPKI update with fixed weights.
  // Tuned for 1ms DTM ticks with ~2ms smoothing half-life.
  double mpki = m_prev_mpki[core_id]; // default to previous
  if (have_miss_metrics && d_instr > 0) {
    double raw_mpki = ((double)d_miss / (double)d_instr) * 1000.0;
    const double kPrev = 0.7;
    const double kRaw = 0.3;
    mpki = kPrev * m_prev_mpki[core_id] + kRaw * raw_mpki;
  }

  // 5. Save state for next tick
  m_prev_instr[core_id] = instr;
  m_prev_miss[core_id] = miss;
  m_prev_mpki[core_id] = mpki;

  // 6. Classification.
  // For very small instruction samples or missing miss metrics, fall back to
  // utilization-based memory-intensity threshold from config.
  // (Lower utilization usually indicates memory-stall dominated execution.)
  const bool mpki_mem_bound = (mpki >= m_mpki_threshold);
  const uint64_t min_instr_for_mpki_only = 500;
  if (!have_miss_metrics || d_instr < min_instr_for_mpki_only) {
    const double util = m_perf->getUtilizationOfCore(core_id);
    const bool util_mem_bound = (util < (double)m_mem_intensity_threshold);
    return util_mem_bound || mpki_mem_bound;
  }

  return mpki_mem_bound;
}

/**
 * recordFreq
 *
 * Appends the current frequency of core_id to its sliding history window.
 * Keeps only the last m_freq_history_len observations.
 * Called once per DTM tick, before any decisions are made for that core.
 */
void DtmAdaptive::recordFreq(int core_id) {
  if (!m_perf)
    return;
  if (core_id < 0 || core_id >= m_num_cores)
    return;

  int f = m_perf->getFreqOfCore(core_id);
  if (f <= 0)
    return; // Not yet initialised.

  auto &hist = m_freq_history[core_id];
  hist.push_back(f);
  while ((int)hist.size() > m_freq_history_len)
    hist.pop_front();
}

/**
 * isThrashing
 *
 * A core is considered thermally thrashing when BOTH of the following hold:
 *
 *   (1) Low-IPC stall signature: IPC < 0.5 AND stall_fraction > 0.6.
 *
 *   (2) Sustained frequency downtrend over the last m_freq_history_len epochs:
 *       the average frequency in the second half of the history window is
 *       lower than in the first half, AND the total drop from oldest to newest
 *       exceeds one DVFS step.
 *
 * Criterion (2) guards against false positives from compute-bound HP threads
 * whose IPC is low for non-thermal reasons (branch pressure, pipeline stalls)
 * rather than DVFS throttling.
 */
bool DtmAdaptive::isThrashing(int core_id, CoreStats coreStats) const {
  // Criterion 1: stall signature.
  if (!(coreStats.ipc < 0.5f && coreStats.stall_fraction > 0.6f))
    return false;

  // Criterion 2: frequency must be on a sustained downward trend.
  if (core_id < 0 || core_id >= m_num_cores)
    return false;
  const auto &hist = m_freq_history[core_id];
  if ((int)hist.size() < m_freq_history_len)
    return false; // not enough history — be conservative

  // Symmetric split: first half = oldest [0, half), second half = newest
  // [size-half, size).  For odd window lengths this skips the centre sample,
  // preventing it from biasing the second-half average toward the middle of
  // the history rather than the recent end.
  int half = (int)hist.size() / 2;            // e.g. 5/2 = 2
  int second_start = (int)hist.size() - half; // e.g. 5-2 = 3 → indices [3,4]
  double first_half = 0.0, second_half = 0.0;
  for (int hi = 0; hi < half; ++hi)
    first_half += hist[hi];
  for (int hi = second_start; hi < (int)hist.size(); ++hi)
    second_half += hist[hi];
  first_half /= (double)half;
  second_half /= (double)half; // same denominator — symmetric averages

  // Softer rule: average of the second half must be lower than the first half
  // by at least one DVFS step, regardless of individual non-monotone points.
  bool freq_declining = (first_half - second_half) >= m_freq_step;

  if (freq_declining && m_log.is_open())
    m_log << "[DTM-Adaptive] thrashing-check core=" << core_id
          << " ipc=" << coreStats.ipc
          << " stall_frac=" << coreStats.stall_fraction
          << " freq_old=" << hist.front() << " freq_now=" << hist.back()
          << " first_half_avg=" << first_half
          << " second_half_avg=" << second_half
          << " -> THRASHING" << std::endl;

  return freq_declining;
}

/**
 * getAdjacentLpNeighbors (2D core topology)
 *
 * Returns adjacent (Manhattan) neighbours in the 2D mesh, but *only* those
 * that are eligible LP heat-sinks for Branch 4 steal-throttling.
 *
 * Eligibility:
 *  - idle neighbour (tid == -1) OR
 *  - running a low-priority thread (weight <= median_weight)
 *
 * No wrap-around across edges.
 */
std::vector<int> DtmAdaptive::getAdjacentLpNeighbors(
    int core_id, const std::vector<int> &core_thread_running,
    const std::map<int, double> &thread_weights, double median_weight) const {
  std::vector<int> neighbors;
  if (m_cores_in_x <= 0 || m_cores_in_y <= 0)
    return neighbors;
  if (core_id < 0 || core_id >= m_num_cores)
    return neighbors;

  int cores_per_layer = m_cores_in_x * m_cores_in_y;
  if (cores_per_layer <= 0)
    return neighbors;

  // Clamp to the 2D mesh footprint even if m_num_cores is smaller.
  int pos_in_layer = core_id % cores_per_layer;
  int x = pos_in_layer % m_cores_in_x;
  int y = pos_in_layer / m_cores_in_x;
  int layer_base = (core_id / cores_per_layer) * cores_per_layer;

  auto is_lp_or_idle = [&](int nid) -> bool {
    if (nid < 0 || nid >= m_num_cores)
      return false;
    if (nid >= (int)core_thread_running.size())
      return false;

    int tid = core_thread_running[nid];
    if (tid == -1)
      return true; // idle => safe sink

    double w = 0.0;
    auto it = thread_weights.find(tid);
    if (it != thread_weights.end())
      w = it->second;
    return (w <= median_weight);
  };

  auto push_if_eligible = [&](int nid) {
    if (is_lp_or_idle(nid))
      neighbors.push_back(nid);
  };

  // West/East
  if (x - 1 >= 0)
    push_if_eligible(layer_base + y * m_cores_in_x + (x - 1));
  if (x + 1 < m_cores_in_x)
    push_if_eligible(layer_base + y * m_cores_in_x + (x + 1));

  // North/South
  if (y - 1 >= 0)
    push_if_eligible(layer_base + (y - 1) * m_cores_in_x + x);
  if (y + 1 < m_cores_in_y)
    push_if_eligible(layer_base + (y + 1) * m_cores_in_x + x);

  return neighbors;
}

/**
 * getCoolestTargetCore
 * Scan all cores, returning a cooler core than the current source core
 * that is running a task with a weight lower than the average weight.
 * (Idle cores are naturally valid targets as well).
 */
int DtmAdaptive::getCoolestTargetCore(
    int source_core, const std::vector<int> &core_thread_running,
    const std::map<int, double> &thread_weights, double median_weight) const {
  if (!m_perf)
    return -1;

  double src_temp = m_perf->getTemperatureOfCore(source_core);
  int best = -1;
  double best_temp =
      src_temp; // Only consider cores strictly cooler than source

  for (int c = 0; c < m_num_cores; ++c) {
    if (c == source_core)
      continue;

    int tid = core_thread_running[c];

    if (tid != -1) {
      double weight_c = 0.0;
      auto it = thread_weights.find(tid);
      if (it != thread_weights.end())
        weight_c = it->second;

      // Target must be running a low-priority task (weight < median_weight)
      if (weight_c >= median_weight)
        continue;
    }

    double T = m_perf->getTemperatureOfCore(c);
    if (T > 0.0 && T < best_temp) {
      best_temp = T;
      best = c;
    }
  }
  return best;
}

/**
 * getCurrentFreq
 * Read the current operating frequency from performance counters.
 * Falls back to m_min_freq if not yet available.
 */
int DtmAdaptive::getCurrentFreq(int core_id) const {
  if (!m_perf)
    return m_min_freq;
  int f = m_perf->getFreqOfCore(core_id);
  return (f > 0) ? f : m_min_freq;
}

/**
 * getBanksForCore
 * Returns the DRAM bank IDs that are associated with core_id in a 2.5D
 * stacked system.  Assumes banks are distributed evenly across cores using
 * a strided (interleaved) mapping:
 *
 *   banks_per_core = ceil(num_banks / num_cores)
 *   banks for core c: c, c + num_cores, c + 2*num_cores, ...
 *
 * This mirrors the default DRAM interleaving used in CoMeT where each
 * memory bank is associated with the core directly below it in the stack.
 */
std::vector<int> DtmAdaptive::getBanksForCore(int core_id) const {
  std::vector<int> banks;
  if (m_num_banks <= 0 || m_num_cores <= 0)
    return banks;
  for (int b = core_id; b < m_num_banks; b += m_num_cores)
    banks.push_back(b);
  return banks;
}

// Phase 2 helpers ─────────────────────────────────────────────────────────────

std::vector<int> DtmAdaptive::banksForChannel(int ch) const {
  std::vector<int> banks;
  // In an interleaved system, Bank ID 'b' belongs to Channel 'ch'
  // if (b % m_num_channels == ch).
  for (int b = ch; b < m_num_banks; b += m_num_channels) {
    banks.push_back(b);
  }
  return banks;
}

std::vector<int> DtmAdaptive::coresForChannel(int ch) const {
  std::vector<int> cores;
  for (int c = ch; c < m_num_cores; c += m_num_channels)
    cores.push_back(c);
  return cores;
}

double DtmAdaptive::channelUtilization(int ch) const {
  if (!m_perf)
    return 0.0;
  std::vector<int> cores = coresForChannel(ch);
  if (cores.empty())
    return 0.0;
  double sum = 0.0;
  for (int c : cores)
    sum += m_perf->getUtilizationOfCore(c);
  return sum / (double)cores.size();
}

int DtmAdaptive::throttleMagnitude(double T) const {
  double slack = m_mem_t_crit - T;
  if (slack <= 0.0)
    return m_k_max;
  int k = (int)std::round((double)m_k_max *
                          std::exp(-slack / (double)m_slack_scale));
  return std::max(0, std::min(k, m_k_max));
}

double DtmAdaptive::bankScore(int bank_id) const {
  if (!m_perf)
    return 0.0;
  // Strided interleaved mapping: bank b -> core (b % num_cores).
  // Multiple banks share the same associated core so they get the same
  // access-weight factor; differentiation within a core's bank group is
  // provided by individual bank temperatures.
  int assoc_core = bank_id % m_num_cores;
  double thermal_slack =
      (double)m_mem_t_crit - m_perf->getTemperatureOfBank(bank_id);
  // Use smoothed per-core MPKI as memory-access-intensity proxy.
  // (m_prev_mpki is updated every DTM tick by isMemoryBound().)
  double access_weight = std::log(m_prev_mpki[assoc_core] + 1.0);
  return thermal_slack * access_weight;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main policy entry point
// ─────────────────────────────────────────────────────────────────────────────

std::vector<DtmDecision>
DtmAdaptive::getDecisions(const std::vector<int> &core_thread_running,
                          const std::map<int, double> &thread_weights,
                          const std::vector<bool> &core_rq_empty) {
  std::vector<DtmDecision> decisions;

  if (!m_perf)
    return decisions;

  // Top-level arbitration: run only one DTM side per epoch.
  // Select the side with smaller thermal slack to critical temperature.
  //   core_slack = core_t_crit - max(core temps)
  //   mem_slack  = mem_t_crit - max(bank temps)
  // Lower slack means closer to critical and gets priority this epoch.
  double max_core_temp = 0.0;
  for (int c = 0; c < m_num_cores; ++c) {
    double t = m_perf->getTemperatureOfCore(c);
    if (t > max_core_temp)
      max_core_temp = t;
  }

  double max_mem_temp = 0.0;
  for (int b = 0; b < m_num_banks; ++b) {
    double t = m_perf->getTemperatureOfBank(b);
    if (t > max_mem_temp)
      max_mem_temp = t;
  }

  bool have_core_temp = (max_core_temp > 0.0);
  bool have_mem_temp = (max_mem_temp > 0.0);
  bool run_core_dtm = false;
  bool run_mem_dtm = false;

  if (have_core_temp && have_mem_temp) {
    double core_slack = m_core_t_crit - max_core_temp;
    double mem_slack = m_mem_t_crit - max_mem_temp;
    if (core_slack <= mem_slack)
      run_core_dtm = true;
    else
      run_mem_dtm = true;
  } else if (have_core_temp) {
    run_core_dtm = true;
  } else if (have_mem_temp) {
    run_mem_dtm = true;
  } else {
    return decisions;
  }

  // ── Cache Memory-Bound Status ───────────────────────────────────────────
  // Cache to avoid side-effects (updating instr/miss counters) from multiple
  // calls per tick in both Phase 1 and Phase 2.
  std::vector<bool> core_is_mem_bound(m_num_cores, false);
  for (int i = 0; i < m_num_cores; ++i) {
    core_is_mem_bound[i] = isMemoryBound(i);
  }

  if (run_core_dtm) {
    // ── Calculate relative priority baseline ────────────────────────────────
    // Use median of ALL threads (both running and queued) to avoid skewed
    // averages
    std::vector<double> w;
    for (const auto &w_pair : thread_weights) {
      if (w_pair.second > 0.0) {
        w.push_back(w_pair.second);
      }
    }

    double median_weight =
        0.0; // We keep the variable name median_weight but it stores the median
    if (!w.empty()) {
      std::sort(w.begin(), w.end());
      median_weight = w[w.size() / 2];
    }

    // if (m_log.is_open()) {
    //   m_log << "[DTM-Adaptive] tick thermal weights: ";
    //   for (double weight_val : w)
    //     m_log << weight_val << " ";
    //   m_log << " | median=" << median_weight << std::endl;
    // }

    for (int i = 0; i < m_num_cores; ++i) {
    // Update frequency history for this core at the start of every tick.
    recordFreq(i);

    // // ── Per-epoch diagnostic: core, running thread, weight, median
    // ───────── if (m_epoch_log.is_open()) {
    //   int dbg_tid = core_thread_running[i];
    //   double dbg_w = 0.0;
    //   if (dbg_tid != -1) {
    //     auto it = thread_weights.find(dbg_tid);
    //     if (it != thread_weights.end())
    //       dbg_w = it->second;
    //   }
    //   m_epoch_log << "[DTM][EPOCH] core=" << i << " tid=" << dbg_tid
    //         << " weight=" << dbg_w << " median_weight=" << median_weight <<
    //         std::endl;
    // }

    // ── Phase A: Thermal check ─────────────────────────────────────────────
    double T = m_perf->getTemperatureOfCore(i);

    if (T <= 0.0)
      continue; // temperature data not yet available (first epoch)

    // ── Bank recovery: if this core has cooled, restore its banks ─────────
    if (T < m_core_t_warn) {
      m_recently_yielded_tid[i] =
          -1; // Core has cooled — clear yielded-thread record

      for (int b : getBanksForCore(i)) {
        if (m_bank_throttled[b]) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] bank recovery core " << i << " bank " << b
                  << " -> normal  T=" << T << std::endl;
          DtmDecision d;
          d.action = DtmAction::DRAM_MODE;
          d.bank_id = b;
          d.bank_mode = 1; // NORMAL_POWER
          decisions.push_back(d);
          m_bank_throttled[b] = false;
        }
      }

      // Frequency restoration: gradually step up if below max frequency
      int cur_f = getCurrentFreq(i);
      if (cur_f < m_max_freq) {
        int new_f = std::min(cur_f + m_freq_step, m_max_freq);
        if (m_log.is_open())
          m_log << "[DTM-Adaptive] core cooled, restoring DVFS core " << i
                << " " << cur_f << " -> " << new_f << " MHz  T=" << T
                << std::endl;
        DtmDecision d;
        d.action = DtmAction::DVFS;
        d.core_id = i;
        d.target_freq = new_f;
        decisions.push_back(d);
      }

      continue; // core is cool — no further action
    }

    if (T > m_core_t_crit) {
      // Emergency: slam frequency to hardware minimum
      if (m_log.is_open())
        m_log << "[DTM-Adaptive] EMERGENCY throttle core " << i << " T=" << T
              << " > T_crit=" << m_core_t_crit << std::endl;
      DtmDecision d;
      d.action = DtmAction::DVFS;
      d.core_id = i;
      d.target_freq = m_min_freq;
      decisions.push_back(d);
      continue;
    }

    // ── Phase B: Workload classification (T_warn ≤ T ≤ T_crit) ───────────
    int thread_id = core_thread_running[i];
    if (thread_id == -1)
      continue; // idle core — nothing to act on

    // Look up thread weight (default 0.0 if not found)
    double weight = 0.0;
    auto it = thread_weights.find(thread_id);
    if (it != thread_weights.end())
      weight = it->second;

    double hp_threshold = median_weight;
    bool is_hp = (weight >= hp_threshold);
    bool is_lp = (weight < hp_threshold);

    // if (m_log.is_open() && is_hp) {
    //   m_log << "[DTM-Adaptive] HP core " << i << " tid=" << thread_id
    //         << " weight=" << weight << std::endl;
    // }

    bool rq_empty = core_rq_empty[i];

    // ── Memory-bound path ─────────────────────────────────────────────────
    // Goal: throttle the rate of DRAM accesses (memory bandwidth), not compute.
    // Mechanism: switch the core's associated DRAM banks to low-power mode,
    // which raises bank access latency from ~15 ns to ~600 ns.  This creates
    // back-pressure that reduces the number of memory requests the core can
    // issue per unit time, effectively limiting bandwidth by a factor of α_mem.

    CoreStats stats;
    double current_cpi = m_perf->getCPIOfCore(i);
    stats.ipc = (current_cpi > 0.0) ? (1.0f / (float)current_cpi) : 0.0f;
    stats.stall_fraction =
        (current_cpi > 1.0) ? (float)((current_cpi - 1.0) / current_cpi) : 0.0f;
    stats.llc_mpki = (float)m_prev_mpki[i];

    if (core_is_mem_bound[i]) {
      for (int b : getBanksForCore(i)) {
        if (!m_bank_throttled[b]) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] mem-throttle core " << i << " bank " << b
                  << " -> lowpower  T=" << T << std::endl;
          DtmDecision d;
          d.action = DtmAction::DRAM_MODE;
          d.bank_id = b;
          d.bank_mode = 0;
          decisions.push_back(d);
          m_bank_throttled[b] = true;
        }
      }
      continue;
    }

    // ── Compute-bound path ────────────────────────────────────────────────

    if (is_lp && !rq_empty) {
      // Branch 1: Low-priority thread on hot core, alternatives waiting.
      if (m_recently_yielded_tid[i] != thread_id) {
        // First yield attempt for this specific thread on this core.
        if (m_log.is_open())
          m_log << "[DTM-Adaptive] Branch1 yield core " << i
                << " tid=" << thread_id << " weight=" << weight
                << " median_weight=" << median_weight << " T=" << T << std::endl;
        DtmDecision d;
        d.action = DtmAction::YIELD;
        d.thread_id = thread_id;
        decisions.push_back(d);
        m_recently_yielded_tid[i] = thread_id; // key by thread, not just core
      } else {
        // Escalate: same thread was already yielded but core remains hot.
        // YIELD is not progressing; fall back to DVFS.
        int cur_f = getCurrentFreq(i);
        int new_f = std::max(cur_f - m_freq_step, m_min_freq);
        if (new_f < cur_f) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] Branch1 ESCALATE DVFS core " << i << " "
                  << cur_f << " -> " << new_f << " MHz (yield ineffective)"
                  << " T=" << T << std::endl;
          DtmDecision d;
          d.action = DtmAction::DVFS;
          d.core_id = i;
          d.target_freq = new_f;
          decisions.push_back(d);
        }
      }
    } else if (is_lp && rq_empty) {
      // Branch 2: Low-priority, run-queue empty → local DVFS scale-down
      int cur_f = getCurrentFreq(i);
      int new_f = std::max(cur_f - m_freq_step, m_min_freq);
      if (new_f < cur_f) {
        if (m_log.is_open())
          m_log << "[DTM-Adaptive] Branch2 DVFS core " << i << " " << cur_f
                << " -> " << new_f << " MHz"
                << " T=" << T << std::endl;
        DtmDecision d;
        d.action = DtmAction::DVFS;
        d.core_id = i;
        d.target_freq = new_f;
        decisions.push_back(d);
      }
    } else if (is_hp && isThrashing(i, stats)) {
      // Branch 3: High-priority + thrashing → migrate to a cooler valid core
      if (m_log.is_open())
        m_log << "[DTM-Adaptive] Branch3 ---------------------" << std::endl;
      int dst = getCoolestTargetCore(i, core_thread_running, thread_weights,
                                     median_weight);
      if (dst != -1 && dst != i) {
        if (m_log.is_open())
          m_log << "[DTM-Adaptive] Branch3 migrate HP tid=" << thread_id
                << " core " << i << " -> " << dst << " T_src=" << T
                << std::endl;
        DtmDecision d;
        d.action = DtmAction::MIGRATE;
        d.thread_id = thread_id;
        d.target_core = dst;
        decisions.push_back(d);
      } else {
        // Fallback: apply local DVFS if no cool LP/idle core is found
        int cur_f = getCurrentFreq(i);
        int new_f = std::max(cur_f - m_freq_step, m_min_freq);
        if (new_f < cur_f) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] Branch3 fallback DVFS core " << i << " "
                  << cur_f << " -> " << new_f << " MHz"
                  << " T=" << T << std::endl;
          DtmDecision d;
          d.action = DtmAction::DVFS;
          d.core_id = i;
          d.target_freq = new_f;
          decisions.push_back(d);
        }
      }
    } else if (is_hp && !isThrashing(i, stats)) {
      // Branch 4: High-priority, not yet thrashing
      // Steal-throttle the vertical neighbor: if it is cool and runs a
      // low-priority task, reduce its frequency to create a local thermal
      // sink — heat from the HP core dissipates into the cooler neighbour.
      std::vector<int> neighbors = getAdjacentLpNeighbors(
          i, core_thread_running, thread_weights, median_weight);
      bool throttled_neighbor = false;
      for (int j : neighbors) {
        double T_j = m_perf->getTemperatureOfCore(j);
        if (!(T_j > 0.0 && T_j < m_core_t_warn))
          continue;

        int cur_f_j = getCurrentFreq(j);
        int new_f_j = std::max(cur_f_j - m_freq_step, m_min_freq);
        if (new_f_j < cur_f_j) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] Branch4 steal-throttle neighbor core " << j
                  << " " << cur_f_j << " -> " << new_f_j << " MHz"
                  << " (HP core=" << i << " T=" << T << ")" << std::endl;
          DtmDecision d;
          d.action = DtmAction::DVFS;
          d.core_id = j;
          d.target_freq = new_f_j;
          decisions.push_back(d);
          throttled_neighbor = true;
          break; // Only steal-throttle one neighbor to avoid over-throttling
        }
      }

      // Fallback: if no suitable LP+cool neighbor exists, throttle this HP core
      // locally (same behavior style as Branch 3 fallback).
      if (!throttled_neighbor) {
        int cur_f = getCurrentFreq(i);
        int new_f = std::max(cur_f - m_freq_step, m_min_freq);
        if (new_f < cur_f) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] Branch4 fallback DVFS core " << i << " "
                  << cur_f << " -> " << new_f << " MHz"
                  << " T=" << T << std::endl;
          DtmDecision d;
          d.action = DtmAction::DVFS;
          d.core_id = i;
          d.target_freq = new_f;
          decisions.push_back(d);
        }
      }
    }
    } // for each core  [Phase 1 end]
  }

  // ── Phase 2: Per-channel Memory DTM ─────────────────────────────────────
  if (run_mem_dtm) {
    for (int ch = 0; ch < m_num_channels; ++ch) {
    std::vector<int> ch_banks = banksForChannel(ch);
    if (ch_banks.empty())
      continue;

    // Step 1: Channel temperature = max bank temperature.
    double T_ch = 0.0;
    for (int b : ch_banks) {
      double t = m_perf->getTemperatureOfBank(b);
      if (t > T_ch)
        T_ch = t;
    }
    if (T_ch <= 0.0)
      continue;

    // Recovery: channel has cooled below t_warn — restore throttled banks.
    if (T_ch < m_mem_t_warn) {
      for (int b : ch_banks) {
        if (m_bank_throttled[b]) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] MemDTM ch" << ch << " bank " << b
                  << " restored  T=" << T_ch << std::endl;
          DtmDecision d;
          d.action = DtmAction::DRAM_MODE;
          d.bank_id = b;
          d.bank_mode = 1; // NORMAL_POWER
          decisions.push_back(d);
          m_bank_throttled[b] = false;
        }
      }
      continue;
    }

    if (T_ch < m_mem_t_warn)
      continue; // below warning — no new action

    if (T_ch > m_mem_t_crit) {
      // Emergency: all banks in this channel to LPM.
      if (m_log.is_open())
        m_log << "[DTM-Adaptive] MemDTM EMERGENCY ch" << ch << "  T=" << T_ch
              << std::endl;
      for (int b : ch_banks) {
        if (!m_bank_throttled[b]) {
          DtmDecision d;
          d.action = DtmAction::DRAM_MODE;
          d.bank_id = b;
          d.bank_mode = 0; // LOW_POWER
          decisions.push_back(d);
          m_bank_throttled[b] = true;
        }
      }
      continue;
    }

    // T_warn <= T_ch <= T_crit

    // Step 2: Workload intensity analysis.
    std::vector<int> mapped_cores = coresForChannel(ch);
    int mem_bound_count = 0;
    for (int c : mapped_cores) {
      if (core_is_mem_bound[c])
        mem_bound_count++;
    }
    // Majority decision: true if strictly more than half the cores are memory
    // bound.
    bool mem_intensive = mem_bound_count > (int)(mapped_cores.size() / 2);

    if (!mem_intensive) {
      // Heat is from cores via vertical coupling, not memory traffic.
      // Request DVFS on associated cores; leave memory banks unchanged.
      for (int c : coresForChannel(ch)) {
        int cur_f = getCurrentFreq(c);
        int new_f = std::max(cur_f - m_freq_step, m_min_freq);
        if (new_f < cur_f) {
          if (m_log.is_open())
            m_log << "[DTM-Adaptive] MemDTM ch" << ch << " coupling: DVFS c"
                  << c << " " << cur_f << "->" << new_f << " MHz"
                  << "  T=" << T_ch << std::endl;
          DtmDecision d;
          d.action = DtmAction::DVFS;
          d.core_id = c;
          d.target_freq = new_f;
          decisions.push_back(d);
        }
      }
      continue;
    }

    // Step 3: Fine-grained bank LTM selection.
    int k = throttleMagnitude(T_ch);
    if (m_log.is_open())
      m_log << "[DTM-Adaptive] MemDTM ch" << ch << " mem-intensive T=" << T_ch
            << " mb_cores=" << mem_bound_count << "/" << mapped_cores.size()
            << " k=" << k << std::endl;
    if (k == 0)
      continue;

    // Score banks and sort ascending (lowest score throttled first).
    struct BS {
      int bid;
      double sc;
    };
    std::vector<BS> scored;
    for (int b : ch_banks)
      scored.push_back({b, bankScore(b)});
    std::sort(scored.begin(), scored.end(),
              [](const BS &a, const BS &b_) { return a.sc < b_.sc; });

    for (int idx = 0; idx < (int)scored.size(); ++idx) {
      int b = scored[idx].bid;
      bool ltm = (idx < k);
      int mode = ltm ? 0 : 1; // LOW_POWER=0, NORMAL_POWER=1
      if (ltm != m_bank_throttled[b]) {
        if (m_log.is_open())
          m_log << "[DTM-Adaptive] MemDTM ch" << ch << " bank " << b
                << (ltm ? " -> LTM" : " <- restored")
                << "  score=" << scored[idx].sc << std::endl;
        DtmDecision d;
        d.action = DtmAction::DRAM_MODE;
        d.bank_id = b;
        d.bank_mode = mode;
        decisions.push_back(d);
        m_bank_throttled[b] = ltm;
      }
    }
    } // for each channel  [Phase 2 end]
  }

  return decisions;
}
