#ifndef __DTM_ADAPTIVE_H
#define __DTM_ADAPTIVE_H

/**
 * dtmAdaptive.h
 *
 * Priority-Aware Adaptive Thermal Management (DTM) Policy.
 * Implements the DtmPolicy interface for use with SchedulerCFSLite.
 *
 * Enable in base.cfg:
 *   [scheduler/cfs_lite/dtm]
 *   logic = adaptive
 */

#include "dtmpolicy.h"
#include "performance_counters.h"
#include <deque>
#include <fstream>

class DtmAdaptive : public DtmPolicy {
public:
  /**
   * @param perf_counters  Live performance/temperature counters.
   * @param num_cores      Total application cores in the system.
   * @param cores_in_x     Core-grid X dimension (from memory/cores_in_x).
   * @param cores_in_y     Core-grid Y dimension (from memory/cores_in_y).
   * @param t_warn         Warning temperature threshold (°C).
   * @param t_crit         Critical/emergency temperature threshold (°C).
   * @param min_freq_mhz   Absolute minimum frequency the policy will request
   * (MHz).
   * @param max_freq_mhz   Absolute maximum baseline frequency for recovery (MHz).
   * @param freq_step_mhz  Single DVFS step size (MHz).
   */
  DtmAdaptive(const PerformanceCounters *perf_counters, int num_cores,
              int cores_in_x, int cores_in_y, int num_banks, int num_channels,
              float core_t_warn, float core_t_crit,
              float mem_t_warn, float mem_t_crit, int min_freq_mhz,
              int max_freq_mhz,
              int freq_step_mhz, int k_max, float slack_scale,
              float mem_intensity_threshold, float mpki_threshold,
              int freq_history_len);

  virtual ~DtmAdaptive() = default;

  /**
   * Compute a list of thermal management actions.
   * Not const: updates m_bank_throttled state for DRAM recovery tracking.
   */
  virtual std::vector<DtmDecision>
  getDecisions(const std::vector<int> &core_thread_running,
               const std::map<int, double> &thread_weights,
               const std::vector<bool> &core_rq_empty) override;

private:
  const PerformanceCounters *m_perf;
  int m_num_cores;
  int m_cores_in_x;
  int m_cores_in_y;
  int m_num_banks;         ///< Total DRAM banks in the system.
  int m_num_channels;      ///< Number of memory channels.
  int m_banks_per_channel; ///< num_banks / num_channels (floored).
  int m_cores_per_channel; ///< num_cores / num_channels (floored).
  float m_core_t_warn;
  float m_core_t_crit; ///< Core temperature (°C) emergency threshold.
  float m_mem_t_warn;
  float m_mem_t_crit; ///< Memory temperature (°C) emergency threshold.
  int m_min_freq;
  int m_max_freq;
  int m_freq_step;
  int m_k_max;         ///< Max banks throttled per channel per call.
  float m_slack_scale; ///< Exponential curve steepness (°C).
  float m_mem_intensity_threshold; ///< Core-util threshold for memory-intensive
                                   ///< classification.
  float
      m_mpki_threshold; ///< MPKI above which a core is considered memory-bound.
  int m_freq_history_len; ///< Number of past epochs to retain for thrashing
                          ///< detection.

  /** Log file — all [DTM-Adaptive] messages go here instead of stdout. */
  mutable std::ofstream m_log;
  mutable std::ofstream m_epoch_log; ///< Separate file for per-core per-epoch weight diagnostics.

  /// Tracks which banks are currently in low-power/LTM mode.
  std::vector<bool> m_bank_throttled;

  /// Delta MPKI state trackers per core
  std::vector<uint64_t> m_prev_instr;
  std::vector<uint64_t> m_prev_miss;
  std::vector<double> m_prev_mpki;

  /**
   * Per-core record of the thread_id most recently yielded by Branch 1.
   * -1 means no pending yield on this core.
   * Reset to -1 when the core cools below t_warn.
   * Keyed by (core, thread) so a different thread landing on the same
   * hot core is not incorrectly skipped to DVFS escalation.
   */
  std::vector<int> m_recently_yielded_tid;

  /**
   * Per-core ring buffer of recent observed frequencies (MHZ).
   * Updated each DTM tick via recordFreq().
   * Used by isThrashing() to detect sustained frequency suppression.
   */
  std::vector<std::deque<int>> m_freq_history;

  /// Append current freq for core to its history ring-buffer.
  void recordFreq(int core_id);

  // ── Core-side helpers ────────────────────────────────────────────────────
  bool isMemoryBound(int core_id);

  struct CoreStats {
    float ipc;
    float stall_fraction;
    float llc_mpki;
  };
  bool isThrashing(int core_id, CoreStats coreStats) const;

  std::vector<int> getAdjacentLpNeighbors(
      int core_id, const std::vector<int> &core_thread_running,
      const std::map<int, double> &thread_weights, double median_weight) const;
  int getCoolestTargetCore(int source_core,
                           const std::vector<int> &core_thread_running,
                           const std::map<int, double> &thread_weights,
                           double median_weight) const;
  int getCurrentFreq(int core_id) const;
  std::vector<int> getBanksForCore(int core_id) const;

  // ── Memory-channel helpers (Phase 2) ─────────────────────────────────────

  /// Banks belonging to channel ch (contiguous block).
  std::vector<int> banksForChannel(int ch) const;

  /// Cores associated with channel ch (strided: core c → ch = c %
  /// num_channels).
  std::vector<int> coresForChannel(int ch) const;

  /// Average compute utilisation of cores in channel ch.
  double channelUtilization(int ch) const;

  /**
   * Throttle magnitude for channel at temperature T.
  * k = round(k_max × exp(−(mem_T_crit − T) / slack_scale))
   * Clamped to [0, k_max].
   */
  int throttleMagnitude(double T) const;

  /**
   * Score a bank for LTM selection.
   * score(b) = (N − C) × log(MAC + 1)
  *   N   = m_mem_t_crit  (thermal capacity limit)
   *   C   = T_bank        (current bank temperature)
   *   MAC = IPS_core      (memory access count proxy)
   * Lower score → throttled first (hot bank, low-utility core).
   */
  double bankScore(int bank_id) const;
};

#endif // __DTM_ADAPTIVE_H
