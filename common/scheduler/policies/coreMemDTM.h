#ifndef __CORE_MEM_DTM_H
#define __CORE_MEM_DTM_H

#include "dvfspolicy.h"
#include "drampolicy.h"
#include "performance_counters.h"
#include "subsecond_time.h"
#include <vector>
#include <map>
#include <string>
#include <fstream> // Required for std::ofstream

// Forward declaration
class DtmStateManager;

// ENUM for last decision made
enum LastDecision { NONE, CORE, MEMORY };


// ############################################################################
// ## 1. Coordinated Core DTM Policy (DVFSCoreMem)
// ############################################################################
class DVFSCoreMem : public DVFSPolicy {
public:
    DVFSCoreMem(
        const PerformanceCounters *performanceCounters,
        int numberOfCores,
        int minFrequency,
        int maxFrequency,
        int frequencyStepSize,
        DtmStateManager *dtmManager,
        const std::vector<float>& dvfs_temp_margins,
        const std::vector<int>& dvfs_frequencies,
        const std::vector<float>& sb_temp_thresholds_C,
        const std::vector<int>& sb_stall_counts_S,
        float alpha,
        float beta
    );

    virtual std::vector<int> getFrequencies(
        const std::vector<int> &oldFrequencies,
        const std::vector<bool> &activeCores
    );

private:
    // Pointers to simulation objects
    const PerformanceCounters *m_performanceCounters;
    DtmStateManager *m_dtmManager;

    // System configuration
    int m_numberOfCores;
    int m_minFrequency;
    int m_maxFrequency;
    int m_frequencyStepSize;

    // --- Parameters for Coarse-Grained DVFS (get_baseline_frequency) ---
    std::vector<float> m_dvfs_temp_margins;
    std::vector<int> m_dvfs_frequencies;
    bool m_dvfs_initialized = false;

    // --- Parameters for Fine-Grained Stall Balancing (get_SB_count) ---
    std::vector<float> m_sb_temp_thresholds_C;
    std::vector<int> m_sb_stall_counts_S;
    float m_alpha;
    float m_beta;

    int get_baseline_frequency(float T_dvfs, float T_P);
    int get_SB_count(float T_max, float T_avg, float T_P);
};


// ############################################################################
// ## 2. Coordinated Memory DTM Policy (DramCoreMem)
// ############################################################################
class DramCoreMem : public DramPolicy {
public:
    DramCoreMem(
        const PerformanceCounters *performanceCounters,
        int numberOfBanks,
        DtmStateManager *dtmManager,
        const std::vector<float>& tempThresholds_M,
        const std::vector<int>& channelCounts_L
    );

    virtual std::map<int, int> getNewBankModes(std::map<int, int> old_bank_modes);

private:
    const PerformanceCounters *m_performanceCounters;
    DtmStateManager *m_dtmManager;
    int m_numberOfBanks;
    std::vector<float> m_tempThresholds_M;
    std::vector<int> m_channelCounts_L;
};


// ############################################################################
// ## 3. The Singleton State Manager
// ############################################################################
class DtmStateManager {
public:
    static DtmStateManager* getInstance();

    void makeCoordinatedDecision(
        float peakCoreTemp,
        float peakMemTemp
    );

    // --- Getters for config limits ---
    float getCoreTempLimit() const;
    float getMemTempLimit() const;

    // --- Public logging function ---
    void logCurrentState(SubsecondTime currentTime, float peakCoreTemp, float peakMemTemp);


    // --- K-Epoch State Management for Core DTM ---
    int getLastStallBalancedCore() const;
    void setLastStallBalancedCore(int coreId);
    void incrementEpoch();
    bool isKthEpoch() const;
    void updateKEpochMaxTemps(const std::vector<float>& current_temps);
    void resetKEpochMaxTemps();
    const std::vector<float>& getKEpochMaxTemps() const;
    const std::vector<int>& getBaselineFrequencies() const;
    void setBaselineFrequencies(const std::vector<int>& freqs);
    
    // --- Current Epoch Temperature Tracking ---
    void updateCurrentEpochMaxTemps(const std::vector<float>& current_temps);
    void resetCurrentEpochMaxTemps();
    const std::vector<float>& getCurrentEpochMaxTemps() const;
    float getCurrentEpochPeakTemp() const;

    // Fetch state
    LastDecision getLastDecision() const;

private:
    DtmStateManager();
    ~DtmStateManager();

    // ===== 1. Configuration Parameters =====
    float coreTempLimit;
    float memTempLimit;
    float coreDtmThreshold;
    float memDtmThreshold;

    // ===== 2. Per-Epoch Coordinated Decision State =====
    bool coreHeated;
    bool memHeated;

    // ===== 3. State for Fine-Grained Stall-Balancing =====
    int lastStallBalancedCoreIndex;

    // ===== 4. State for Coarse-Grained DVFS =====
    unsigned long long m_currentDvfsEpoch;
    int m_dvfsIntervalK;
    std::vector<float> m_k_epoch_max_temps;
    std::vector<float> m_current_epoch_max_temps;  // Track current epoch max temps for logging
    std::vector<int> m_baseline_frequencies;

    // ===== 5. State for Debugging and History Logging =====
    LastDecision lastAction;
    SubsecondTime lastActionTime;
    std::ofstream m_logFile;

    // ===== Singleton instance pointer =====
    static DtmStateManager* instance;
};

#endif // __CORE_MEM_DTM_H

