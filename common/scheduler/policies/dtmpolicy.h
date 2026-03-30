/**
 * dtmpolicy.h
 *
 * Abstract interface for pluggable Dynamic Thermal Management (DTM) policies
 * used by SchedulerCFSLite.
 *
 * To add a new DTM policy:
 *   1. Create a new class in the policies/ folder that inherits DtmPolicy.
 *   2. Implement getDecisions().
 *   3. Add an `else if (logic == "your_name")` branch in
 *      SchedulerCFSLite::initDtmPolicy().
 *   4. Add `logic = your_name` under [scheduler/cfs_lite/dtm] in base.cfg.
 */

#ifndef __DTMPOLICY_H
#define __DTMPOLICY_H

#include <vector>
#include <map>

// ─────────────────────────────────────────────────────────────────────────────
// Shared decision types (used by every DtmPolicy implementation)
// ─────────────────────────────────────────────────────────────────────────────

/** Action types that a DTM policy can request from the scheduler. */
enum class DtmAction {
    NONE,       ///< No action required for this entry.
    DVFS,       ///< Change frequency of core_id to target_freq (MHz).
    YIELD,      ///< Cooperatively yield thread_id off its core.
    MIGRATE,    ///< Move thread_id to target_core.
    DRAM_MODE   ///< Set DRAM bank bank_id to bank_mode (0=normal, 1=lowpower).
                ///<   Low-power mode has higher latency, limiting effective memory bandwidth.
};

/**
 * A single scheduling/thermal action returned by a DtmPolicy.
 * The scheduler inspects the 'action' field and executes the relevant fields.
 */
struct DtmDecision {
    DtmAction action      = DtmAction::NONE;
    int       core_id     = -1;   ///< Core affected (DVFS) or currently running thread (YIELD/MIGRATE).
    int       thread_id   = -1;   ///< Thread to yield or migrate  (-1 = N/A).
    int       target_core = -1;   ///< Destination core             (MIGRATE only).
    int       target_freq = -1;   ///< New frequency in MHz         (DVFS only).
    int       bank_id     = -1;   ///< DRAM bank to change          (DRAM_MODE only).
    int       bank_mode   = -1;   ///< New bank mode: 0=normal, 1=lowpower (DRAM_MODE only).
};

// ─────────────────────────────────────────────────────────────────────────────
// Abstract policy interface
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Abstract base class for DTM policies used with SchedulerCFSLite.
 *
 * Each policy receives a lightweight snapshot of the scheduler's state and
 * returns a list of DtmDecision structs.  The scheduler executes each decision
 * via its own threadYield / migrateThread / setCoreFrequency APIs, keeping
 * the policy fully decoupled from Sniper internals.
 *
 * @param core_thread_running  Vector of size num_cores.
 *                             core_thread_running[c] = thread_id running on core c,
 *                             or -1 if the core is idle.
 * @param thread_priorities    Map from thread_id to nice-style priority
 *                             (lower value = higher priority).
 * @param core_rq_empty        core_rq_empty[c] = true when the per-core
 *                             run-queue for core c has no waiting threads.
 * @return                     List of DtmDecision structs; at most one
 *                             non-NONE action per core per call.
 */
class DtmPolicy {
public:
    virtual ~DtmPolicy() {}

    virtual std::vector<DtmDecision> getDecisions(
        const std::vector<int>&    core_thread_running,
        const std::map<int,int>&   thread_priorities,
        const std::vector<bool>&   core_rq_empty) = 0;
};

#endif // __DTMPOLICY_H
