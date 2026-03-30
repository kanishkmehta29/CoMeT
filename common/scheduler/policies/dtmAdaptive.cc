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
 *      Branch 3: High-priority, thrashing (T > T_warn+3)→ MIGRATE to coolest idle core
 *      Branch 4: High-priority, not thrashing           → DVFS on vertical neighbor
 *                                                         (steal-throttle: cool LP neighbor
 *                                                          to act as a thermal heat-sink)
 */

#include "dtmAdaptive.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

DtmAdaptive::DtmAdaptive(const PerformanceCounters* perf_counters,
                         int   num_cores,
                         int   cores_in_x,
                         int   cores_in_y,
                         int   num_banks,
                         int   num_channels,
                         float t_warn,
                         float t_crit,
                         float alpha_mem,
                         int   min_freq_mhz,
                         int   freq_step_mhz,
                         float t_recover,
                         int   k_max,
                         float slack_scale,
                         float mem_intensity_threshold)
   : m_perf(perf_counters)
   , m_num_cores(num_cores)
   , m_cores_in_x(cores_in_x)
   , m_cores_in_y(cores_in_y)
   , m_num_banks(num_banks)
   , m_num_channels(std::max(1, num_channels))
   , m_banks_per_channel(std::max(1, num_banks / std::max(1, num_channels)))
   , m_cores_per_channel(std::max(1, num_cores / std::max(1, num_channels)))
   , m_t_warn(t_warn)
   , m_t_crit(t_crit)
   , m_t_recover(t_recover)
   , m_alpha_mem(alpha_mem)
   , m_min_freq(min_freq_mhz)
   , m_freq_step(freq_step_mhz)
   , m_k_max(k_max)
   , m_slack_scale(slack_scale)
   , m_mem_intensity_threshold(mem_intensity_threshold)
   , m_bank_throttled(num_banks, false)
{
   std::cout << "[DTM-Adaptive] initialized"
             << " t_warn=" << m_t_warn << " t_crit=" << m_t_crit
             << " t_recover=" << m_t_recover
             << " alpha_mem=" << m_alpha_mem
             << " min_freq=" << m_min_freq << " MHz"
             << " num_banks=" << m_num_banks
             << " channels=" << m_num_channels
             << " k_max=" << m_k_max
             << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * isMemoryBound
 * Compute utilisation < 0.6 → more than 40 % of cycles are memory/sync stalls.
 */
bool DtmAdaptive::isMemoryBound(int core_id) const
{
   if (!m_perf) return false;
   double util = m_perf->getUtilizationOfCore(core_id);
   return (util >= 0.0 && util < 0.6);
}

/**
 * isThrashing
 * Core temperature has exceeded T_warn by more than 3 °C.  At this point
 * a compute-bound HP task is better migrated than throttled in place.
 */
bool DtmAdaptive::isThrashing(int core_id) const
{
   if (!m_perf) return false;
   double T = m_perf->getTemperatureOfCore(core_id);
   return (T > 0.0 && T > (m_t_warn + 3.0));
}

/**
 * getVerticalNeighbors
 * Returns the cores directly above and below in the Y dimension of the XY grid,
 * wrapping around. Returns an empty vector for single-row layouts.
 */
std::vector<int> DtmAdaptive::getVerticalNeighbors(int core_id) const
{
   std::vector<int> neighbors;
   if (m_cores_in_y <= 1)
      return neighbors;

   int x        = core_id % m_cores_in_x;
   int y        = core_id / m_cores_in_x;

   int y_below  = (y + 1) % m_cores_in_y;
   int n_below  = y_below * m_cores_in_x + x;
   if (n_below != core_id && n_below >= 0 && n_below < m_num_cores)
      neighbors.push_back(n_below);

   int y_above  = (y - 1 + m_cores_in_y) % m_cores_in_y;
   int n_above  = y_above * m_cores_in_x + x;
   if (n_above != core_id && n_above >= 0 && n_above < m_num_cores && n_above != n_below)
      neighbors.push_back(n_above);

   return neighbors;
}

/**
 * getCoolestIdleCore
 * Scan all cores, return the one with the lowest temperature that has no
 * thread currently running.  Used to pick a migration destination.
 */
int DtmAdaptive::getCoolestIdleCore(const std::vector<int>& core_thread_running) const
{
   if (!m_perf) return -1;

   int    best = -1;
   double best_temp = std::numeric_limits<double>::max();

   for (int c = 0; c < m_num_cores; ++c)
   {
      if (core_thread_running[c] != -1) // -1 == INVALID / idle
         continue;

      double T = m_perf->getTemperatureOfCore(c);
      if (T > 0.0 && T < best_temp)
      {
         best_temp = T;
         best      = c;
      }
   }
   return best;
}

/**
 * getCurrentFreq
 * Read the current operating frequency from performance counters.
 * Falls back to m_min_freq if not yet available.
 */
int DtmAdaptive::getCurrentFreq(int core_id) const
{
   if (!m_perf) return m_min_freq;
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
std::vector<int> DtmAdaptive::getBanksForCore(int core_id) const
{
   std::vector<int> banks;
   if (m_num_banks <= 0 || m_num_cores <= 0) return banks;
   for (int b = core_id; b < m_num_banks; b += m_num_cores)
      banks.push_back(b);
   return banks;
}

// Phase 2 helpers ─────────────────────────────────────────────────────────────

std::vector<int> DtmAdaptive::banksForChannel(int ch) const
{
   std::vector<int> banks;
   int start = ch * m_banks_per_channel;
   int end   = std::min(start + m_banks_per_channel, m_num_banks);
   for (int b = start; b < end; ++b) banks.push_back(b);
   return banks;
}

std::vector<int> DtmAdaptive::coresForChannel(int ch) const
{
   std::vector<int> cores;
   for (int c = ch; c < m_num_cores; c += m_num_channels)
      cores.push_back(c);
   return cores;
}

double DtmAdaptive::channelUtilization(int ch) const
{
   if (!m_perf) return 0.0;
   std::vector<int> cores = coresForChannel(ch);
   if (cores.empty()) return 0.0;
   double sum = 0.0;
   for (int c : cores) sum += m_perf->getUtilizationOfCore(c);
   return sum / (double)cores.size();
}

int DtmAdaptive::throttleMagnitude(double T) const
{
   double slack = m_t_crit - T;
   if (slack <= 0.0) return m_k_max;
   int k = (int)std::round((double)m_k_max * std::exp(-slack / (double)m_slack_scale));
   return std::max(0, std::min(k, m_k_max));
}

double DtmAdaptive::bankScore(int bank_id) const
{
   if (!m_perf) return 0.0;
   double N   = (double)m_t_crit;
   double C   = m_perf->getTemperatureOfBank(bank_id);
   double MAC = m_perf->getIPSOfCore(bank_id % m_num_cores);
   return (N - C) * std::log(MAC + 1.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main policy entry point
// ─────────────────────────────────────────────────────────────────────────────

std::vector<DtmDecision> DtmAdaptive::getDecisions(
      const std::vector<int>&    core_thread_running,
      const std::map<int,int>&   thread_priorities,
      const std::vector<bool>&   core_rq_empty)
{
   std::vector<DtmDecision> decisions;

   if (!m_perf)
      return decisions;

   for (int i = 0; i < m_num_cores; ++i)
   {
      // ── Phase A: Thermal check ─────────────────────────────────────────────
      double T = m_perf->getTemperatureOfCore(i);

      if (T <= 0.0)
         continue;   // temperature data not yet available (first epoch)

      // ── Bank recovery: if this core has cooled, restore its banks ─────────
      if (T < m_t_warn)
      {
         for (int b : getBanksForCore(i))
         {
            if (m_bank_throttled[b])
            {
               std::cout << "[DTM-Adaptive] bank recovery core " << i
                         << " bank " << b << " -> normal  T=" << T << std::endl;
               DtmDecision d;
               d.action    = DtmAction::DRAM_MODE;
               d.bank_id   = b;
               d.bank_mode = 0;   // normal latency
               decisions.push_back(d);
               m_bank_throttled[b] = false;
            }
         }
         continue;   // core is cool — no further action
      }

      if (T > m_t_crit)
      {
         // Emergency: slam frequency to hardware minimum
         std::cout << "[DTM-Adaptive] EMERGENCY throttle core " << i
                   << " T=" << T << " > T_crit=" << m_t_crit << std::endl;
         DtmDecision d;
         d.action     = DtmAction::DVFS;
         d.core_id    = i;
         d.target_freq = m_min_freq;
         decisions.push_back(d);
         continue;
      }

      // ── Phase B: Workload classification (T_warn ≤ T ≤ T_crit) ───────────
      int thread_id = core_thread_running[i];
      if (thread_id == -1)
         continue;   // idle core — nothing to act on

      // Look up thread priority (default 0 = normal if not found)
      int prio = 0;
      auto it  = thread_priorities.find(thread_id);
      if (it != thread_priorities.end())
         prio = it->second;

      bool is_hp    = (prio < 0);   // negative nice-value → high priority
      bool rq_empty = core_rq_empty[i];

      // ── Memory-bound path ─────────────────────────────────────────────────
      // Goal: throttle the rate of DRAM accesses (memory bandwidth), not compute.
      // Mechanism: switch the core's associated DRAM banks to low-power mode,
      // which raises bank access latency from ~15 ns to ~600 ns.  This creates
      // back-pressure that reduces the number of memory requests the core can
      // issue per unit time, effectively limiting bandwidth by a factor of α_mem.
      if (isMemoryBound(i))
      {
         for (int b : getBanksForCore(i))
         {
            if (!m_bank_throttled[b])
            {
               std::cout << "[DTM-Adaptive] mem-throttle core " << i
                         << " bank " << b << " -> lowpower  T=" << T << std::endl;
               DtmDecision d;
               d.action    = DtmAction::DRAM_MODE;
               d.bank_id   = b;
               d.bank_mode = 1;   // low-power / high-latency = bandwidth throttled
               decisions.push_back(d);
               m_bank_throttled[b] = true;
            }
         }
         continue;
      }

      // ── Compute-bound path ────────────────────────────────────────────────

      if (!is_hp && !rq_empty)
      {
         // Branch 1: Low-priority, alternatives waiting → cooperative yield
         std::cout << "[DTM-Adaptive] Branch1 yield core " << i
                   << " tid=" << thread_id
                   << " prio=" << prio
                   << " T=" << T << std::endl;
         DtmDecision d;
         d.action    = DtmAction::YIELD;
         d.thread_id = thread_id;
         decisions.push_back(d);
      }
      else if (!is_hp && rq_empty)
      {
         // Branch 2: Low-priority, run-queue empty → local DVFS scale-down
         int cur_f = getCurrentFreq(i);
         int new_f = std::max(cur_f - m_freq_step, m_min_freq);
         if (new_f < cur_f)
         {
            std::cout << "[DTM-Adaptive] Branch2 DVFS core " << i
                      << " " << cur_f << " -> " << new_f << " MHz"
                      << " T=" << T << std::endl;
            DtmDecision d;
            d.action      = DtmAction::DVFS;
            d.core_id     = i;
            d.target_freq = new_f;
            decisions.push_back(d);
         }
      }
      else if (is_hp && isThrashing(i))
      {
         // Branch 3: High-priority + thrashing → migrate to coolest idle core
         int dst = getCoolestIdleCore(core_thread_running);
         if (dst != -1 && dst != i)
         {
            std::cout << "[DTM-Adaptive] Branch3 migrate HP tid=" << thread_id
                      << " core " << i << " -> " << dst
                      << " T_src=" << T << std::endl;
            DtmDecision d;
            d.action      = DtmAction::MIGRATE;
            d.thread_id   = thread_id;
            d.target_core = dst;
            decisions.push_back(d);
         }
         else
         {
            // Fallback: apply local DVFS if no cool idle core is found
            int cur_f = getCurrentFreq(i);
            int new_f = std::max(cur_f - m_freq_step, m_min_freq);
            if (new_f < cur_f)
            {
               std::cout << "[DTM-Adaptive] Branch3 fallback DVFS core " << i
                         << " " << cur_f << " -> " << new_f << " MHz"
                         << " T=" << T << std::endl;
               DtmDecision d;
               d.action      = DtmAction::DVFS;
               d.core_id     = i;
               d.target_freq = new_f;
               decisions.push_back(d);
            }
         }
      }
      else if (is_hp && !isThrashing(i))
      {
         // Branch 4: High-priority, not yet thrashing
         // Steal-throttle the vertical neighbor: if it is cool and runs a
         // low-priority task, reduce its frequency to create a local thermal
         // sink — heat from the HP core dissipates into the cooler neighbour.
         std::vector<int> neighbors = getVerticalNeighbors(i);
         for (int j : neighbors)
         {
            if (j >= (int)core_thread_running.size()) continue;

            int tid_j = core_thread_running[j];
            if (tid_j != -1)
            {
               int prio_j  = 0;
               auto jt     = thread_priorities.find(tid_j);
               if (jt != thread_priorities.end()) prio_j = jt->second;

               bool j_is_lp = (prio_j >= 0);
               double T_j   = m_perf->getTemperatureOfCore(j);

               if (j_is_lp && T_j > 0.0 && T_j < m_t_warn)
               {
                  int cur_f_j = getCurrentFreq(j);
                  int new_f_j = std::max(cur_f_j - m_freq_step, m_min_freq);
                  if (new_f_j < cur_f_j)
                  {
                     std::cout << "[DTM-Adaptive] Branch4 steal-throttle neighbor core " << j
                               << " " << cur_f_j << " -> " << new_f_j << " MHz"
                               << " (HP core=" << i << " T=" << T << ")"
                               << std::endl;
                     DtmDecision d;
                     d.action      = DtmAction::DVFS;
                     d.core_id     = j;
                     d.target_freq = new_f_j;
                     decisions.push_back(d);
                     break; // Only steal-throttle one neighbor to avoid over-throttling
                  }
               }
            }
         }
      }
   } // for each core  [Phase 1 end]

   // ── Phase 2: Per-channel Memory DTM ─────────────────────────────────────
   for (int ch = 0; ch < m_num_channels; ++ch)
   {
      std::vector<int> ch_banks = banksForChannel(ch);
      if (ch_banks.empty()) continue;

      // Step 1: Channel temperature = max bank temperature.
      double T_ch = 0.0;
      for (int b : ch_banks) {
         double t = m_perf->getTemperatureOfBank(b);
         if (t > T_ch) T_ch = t;
      }
      if (T_ch <= 0.0) continue;

      // Recovery: channel has cooled below t_recover — restore throttled banks.
      if (T_ch < m_t_recover)
      {
         for (int b : ch_banks) {
            if (m_bank_throttled[b]) {
               std::cout << "[DTM-Adaptive] MemDTM ch" << ch << " bank " << b
                         << " restored  T=" << T_ch << std::endl;
               DtmDecision d; d.action = DtmAction::DRAM_MODE;
               d.bank_id = b; d.bank_mode = 1;   // NORMAL_POWER
               decisions.push_back(d);
               m_bank_throttled[b] = false;
            }
         }
         continue;
      }

      if (T_ch < m_t_warn) continue;   // below warning — no new action

      if (T_ch > m_t_crit)
      {
         // Emergency: all banks in this channel to LPM.
         std::cout << "[DTM-Adaptive] MemDTM EMERGENCY ch" << ch
                   << "  T=" << T_ch << std::endl;
         for (int b : ch_banks) {
            if (!m_bank_throttled[b]) {
               DtmDecision d; d.action = DtmAction::DRAM_MODE;
               d.bank_id = b; d.bank_mode = 0;   // LOW_POWER
               decisions.push_back(d);
               m_bank_throttled[b] = true;
            }
         }
         continue;
      }

      // T_warn <= T_ch <= T_crit

      // Step 2: Workload intensity analysis.
      double util = channelUtilization(ch);
      bool mem_intensive = (util < m_mem_intensity_threshold);

      if (!mem_intensive)
      {
         // Heat is from cores via vertical coupling, not memory traffic.
         // Request DVFS on associated cores; leave memory banks unchanged.
         for (int c : coresForChannel(ch))
         {
            int cur_f = getCurrentFreq(c);
            int new_f = std::max((int)(cur_f * 0.9), m_min_freq);
            if (new_f < cur_f) {
               std::cout << "[DTM-Adaptive] MemDTM ch" << ch
                         << " coupling: DVFS c" << c
                         << " " << cur_f << "->" << new_f << " MHz"
                         << "  T=" << T_ch << std::endl;
               DtmDecision d; d.action = DtmAction::DVFS;
               d.core_id = c; d.target_freq = new_f;
               decisions.push_back(d);
            }
         }
         continue;
      }

      // Step 3: Fine-grained bank LTM selection.
      int k = throttleMagnitude(T_ch);
      std::cout << "[DTM-Adaptive] MemDTM ch" << ch
                << " mem-intensive T=" << T_ch
                << " util=" << util << " k=" << k << std::endl;
      if (k == 0) continue;

      // Score banks and sort ascending (lowest score throttled first).
      struct BS { int bid; double sc; };
      std::vector<BS> scored;
      for (int b : ch_banks) scored.push_back({b, bankScore(b)});
      std::sort(scored.begin(), scored.end(), [](const BS& a, const BS& b_){ return a.sc < b_.sc; });

      for (int idx = 0; idx < (int)scored.size(); ++idx)
      {
         int b    = scored[idx].bid;
         bool ltm = (idx < k);
         int mode = ltm ? 0 : 1;   // LOW_POWER=0, NORMAL_POWER=1
         if (ltm != m_bank_throttled[b]) {
            std::cout << "[DTM-Adaptive] MemDTM ch" << ch << " bank " << b
                      << (ltm ? " -> LTM" : " <- restored")
                      << "  score=" << scored[idx].sc << std::endl;
            DtmDecision d; d.action = DtmAction::DRAM_MODE;
            d.bank_id = b; d.bank_mode = mode;
            decisions.push_back(d);
            m_bank_throttled[b] = ltm;
         }
      }
   } // for each channel  [Phase 2 end]

   return decisions;
}
