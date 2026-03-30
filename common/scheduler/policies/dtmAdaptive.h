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

class DtmAdaptive : public DtmPolicy {
public:
    /**
     * @param perf_counters  Live performance/temperature counters.
     * @param num_cores      Total application cores in the system.
     * @param cores_in_x     Core-grid X dimension (from memory/cores_in_x).
     * @param cores_in_y     Core-grid Y dimension (from memory/cores_in_y).
     * @param t_warn         Warning temperature threshold (°C).
     * @param t_crit         Critical/emergency temperature threshold (°C).
     * @param alpha_mem      Frequency scale factor for memory-bound cores (0–1).
     * @param min_freq_mhz   Absolute minimum frequency the policy will request (MHz).
     * @param freq_step_mhz  Single DVFS step size (MHz).
     */
    DtmAdaptive(const PerformanceCounters* perf_counters,
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
                float mem_intensity_threshold);

    virtual ~DtmAdaptive() = default;

    /**
     * Compute a list of thermal management actions.
     * Not const: updates m_bank_throttled state for DRAM recovery tracking.
     */
    virtual std::vector<DtmDecision> getDecisions(
        const std::vector<int>&    core_thread_running,
        const std::map<int,int>&   thread_priorities,
        const std::vector<bool>&   core_rq_empty) override;

private:
    const PerformanceCounters* m_perf;
    int   m_num_cores;
    int   m_cores_in_x;
    int   m_cores_in_y;
    int   m_num_banks;                  ///< Total DRAM banks in the system.
    int   m_num_channels;               ///< Number of memory channels.
    int   m_banks_per_channel;          ///< num_banks / num_channels (floored).
    int   m_cores_per_channel;          ///< num_cores / num_channels (floored).
    float m_t_warn;
    float m_t_crit;
    float m_t_recover;                  ///< Temperature below which throttled banks are restored.
    float m_alpha_mem;
    int   m_min_freq;
    int   m_freq_step;
    int   m_k_max;                      ///< Max banks throttled per channel per call.
    float m_slack_scale;                ///< Exponential curve steepness (°C).
    float m_mem_intensity_threshold;    ///< Core-util threshold for memory-intensive classification.

    /// Tracks which banks are currently in low-power/LTM mode.
    std::vector<bool> m_bank_throttled;

    // ── Core-side helpers ────────────────────────────────────────────────────
    bool isMemoryBound(int core_id) const;
    bool isThrashing(int core_id) const;
    std::vector<int> getVerticalNeighbors(int core_id) const;
    int  getCoolestIdleCore(const std::vector<int>& core_thread_running) const;
    int  getCurrentFreq(int core_id) const;
    std::vector<int> getBanksForCore(int core_id) const;

    // ── Memory-channel helpers (Phase 2) ─────────────────────────────────────

    /// Banks belonging to channel ch (contiguous block).
    std::vector<int> banksForChannel(int ch) const;

    /// Cores associated with channel ch (strided: core c → ch = c % num_channels).
    std::vector<int> coresForChannel(int ch) const;

    /// Average compute utilisation of cores in channel ch.
    double channelUtilization(int ch) const;

    /**
     * Throttle magnitude for channel at temperature T.
     * k = round(k_max × exp(−(T_crit − T) / slack_scale))
     * Clamped to [0, k_max].
     */
    int throttleMagnitude(double T) const;

    /**
     * Score a bank for LTM selection.
     * score(b) = (N − C) × log(MAC + 1)
     *   N   = m_t_crit      (thermal capacity limit)
     *   C   = T_bank        (current bank temperature)
     *   MAC = IPS_core      (memory access count proxy)
     * Lower score → throttled first (hot bank, low-utility core).
     */
    double bankScore(int bank_id) const;
};

#endif // __DTM_ADAPTIVE_H
