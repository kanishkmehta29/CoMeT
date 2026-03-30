#include "coreMemDTM.h"
#include "config.hpp"     // For Sim()->getCfg()
#include "simulator.h"    // For Sim()
#include <numeric>   // For std::accumulate
#include <algorithm> // For std::sort, std::max_element
#include <vector>

// Initialize the static instance pointer for the singleton to null
DtmStateManager* DtmStateManager::instance = nullptr;

// ============================================================================
// 1. Constructor & Destructor
// ============================================================================

DtmStateManager::DtmStateManager()
    // Initialize member variables to default values
    : coreHeated(false)
    , memHeated(false)
    , lastStallBalancedCoreIndex(0)
    , lastAction(NONE)
    , lastActionTime(SubsecondTime::Zero())
{
    // --- Load configuration parameters from the config file ---
    // Note: The paths used here are examples. You should match them to your actual base.cfg file.
    coreTempLimit = Sim()->getCfg()->getFloat("scheduler/open/dtm/coreMemDTM/core_temp_limit");
    memTempLimit = Sim()->getCfg()->getFloat("scheduler/open/dram/dtm/coreMemDTM/mem_temp_limit");
    coreDtmThreshold = Sim()->getCfg()->getFloat("scheduler/open/dtm/coreMemDTM/core_dtm_threshold");
    memDtmThreshold = Sim()->getCfg()->getFloat("scheduler/open/dram/dtm/coreMemDTM/mem_dtm_threshold");

    // --- Initialize the K-epoch tracking for the Core DTM ---
    m_dvfsIntervalK = Sim()->getCfg()->getInt("scheduler/open/dvfs/coreMemDTM/k_interval"); // ***********ABSENT IN THE PAPER***********
    int numCores = Sim()->getConfig()->getApplicationCores();
    m_k_epoch_max_temps.resize(numCores, 0.0f);
    m_current_epoch_max_temps.resize(numCores, 0.0f);
    m_baseline_frequencies.resize(numCores, (int)(Sim()->getCfg()->getFloat("perf_model/core/max_frequency") * 1000 + 0.5));

    // --- Open the log file and write the CSV header ---
    std::string logFileName(Sim()->getCfg()->getString("scheduler/open/dtm/coreMemDTM/log_file_name").c_str());
    m_logFile.open(logFileName.c_str());
    if (m_logFile.is_open()) {
        std::cout << "[DtmStateManager] Logging to " << logFileName << std::endl;
        m_logFile << "Timestamp,PeakCoreTemp,PeakMemTemp,Decision,LastStallBalancedCore\n";
    }
}

DtmStateManager::~DtmStateManager()
{
    // Ensure the log file is properly closed when the simulation ends
    if (m_logFile.is_open()) {
        m_logFile.close();
    }
}

// ============================================================================
// 2. Singleton Accessor
// ============================================================================

DtmStateManager* DtmStateManager::getInstance()
{
    // If the instance doesn't exist yet, create it.
    if (instance == nullptr) {
        instance = new DtmStateManager();
    }
    return instance;
}

// ============================================================================
// 3. Core DTM Logic
// ============================================================================

void DtmStateManager::makeCoordinatedDecision(
    float peakCoreTemp, float peakMemTemp)
{
    lastAction = NONE; // Default to no action

    // Step 1: Determine the thermal state of each component for this epoch
    coreHeated = (peakCoreTemp > coreDtmThreshold);
    memHeated = (peakMemTemp > memDtmThreshold);

    // Step 2: Implement the decision logic from Table I in the paper
    if (coreHeated && !memHeated) {
        // Case 1: Only the core is hot. Apply CoreDTM.
        lastAction = CORE;
    }
    else if (!coreHeated && memHeated) {
        // Case 2: Only the memory is hot. Apply MemoryDTM.
        lastAction = MEMORY;
    }
    else if (coreHeated && memHeated) {
        // Case 3: Both are hot. Prioritize the one with less thermal slack.
        float coreSlack = coreTempLimit - peakCoreTemp;
        float memSlack = memTempLimit - peakMemTemp;

        if (coreSlack <= memSlack) {
            // Core is in more danger, or equally so. Prioritize CoreDTM.
            lastAction = CORE;
        } else {
            // Memory is in more danger. Prioritize MemoryDTM.
            lastAction = MEMORY;
        }
    }
    else {
        // Case 4: Neither is hot. Do nothing.
        lastAction = NONE;
    }
    
    // Update the timestamp for the last action taken for logging purposes
    if (lastAction != NONE) {
        lastActionTime = Sim()->getClockSkewMinimizationServer()->getGlobalTime();
    }
}


// ============================================================================
// 4. Getters and Setters for Policy State
// ============================================================================

int DtmStateManager::getLastStallBalancedCore() const {
    return lastStallBalancedCoreIndex;
}

void DtmStateManager::setLastStallBalancedCore(int coreId) {
    lastStallBalancedCoreIndex = coreId;
}

void DtmStateManager::incrementEpoch() {
    m_currentDvfsEpoch++;
}

bool DtmStateManager::isKthEpoch() const {
    return (m_currentDvfsEpoch % m_dvfsIntervalK == 0);
}

void DtmStateManager::updateKEpochMaxTemps(const std::vector<float>& current_temps) {
    for (size_t i = 0; i < current_temps.size(); ++i) {
        if (current_temps[i] > m_k_epoch_max_temps[i]) {
            m_k_epoch_max_temps[i] = current_temps[i];
        }
    }
}

void DtmStateManager::resetKEpochMaxTemps() {
    std::fill(m_k_epoch_max_temps.begin(), m_k_epoch_max_temps.end(), 0.0f);
}

const std::vector<float>& DtmStateManager::getKEpochMaxTemps() const {
    return m_k_epoch_max_temps;
}

void DtmStateManager::updateCurrentEpochMaxTemps(const std::vector<float>& current_temps) {
    for (size_t i = 0; i < current_temps.size(); ++i) {
        if (current_temps[i] > m_current_epoch_max_temps[i]) {
            m_current_epoch_max_temps[i] = current_temps[i];
        }
    }
}

void DtmStateManager::resetCurrentEpochMaxTemps() {
    std::fill(m_current_epoch_max_temps.begin(), m_current_epoch_max_temps.end(), 0.0f);
}

const std::vector<float>& DtmStateManager::getCurrentEpochMaxTemps() const {
    return m_current_epoch_max_temps;
}

float DtmStateManager::getCurrentEpochPeakTemp() const {
    return *std::max_element(m_current_epoch_max_temps.begin(), m_current_epoch_max_temps.end());
}

const std::vector<int>& DtmStateManager::getBaselineFrequencies() const {
    return m_baseline_frequencies;
}

void DtmStateManager::setBaselineFrequencies(const std::vector<int>& freqs) {
    m_baseline_frequencies = freqs;
}

float DtmStateManager::getCoreTempLimit() const {
    return coreTempLimit;
}

float DtmStateManager::getMemTempLimit() const {
    return memTempLimit;
}

LastDecision DtmStateManager::getLastDecision() const {
    return lastAction;
}

// ============================================================================
// 5. Logging Function
// ============================================================================

void DtmStateManager::logCurrentState(SubsecondTime currentTime, float peakCoreTemp, float peakMemTemp)
{
    if (m_logFile.is_open()) {
        std::string decision_str;
        switch (lastAction) {
            case CORE:   decision_str = "CORE";   break;
            case MEMORY: decision_str = "MEMORY"; break;
            case NONE:   decision_str = "NONE";   break;
        }
        m_logFile << currentTime.getNS() << ","
                  << peakCoreTemp << ","
                  << peakMemTemp << ","
                  << decision_str << ","
                  << lastStallBalancedCoreIndex << "\n";
        m_logFile.flush(); // Ensure data is written promptly
        std::cout << "[DtmStateManager] Logged state at " << currentTime.getNS() << " ns: "
             << "PeakCoreTemp=" << peakCoreTemp << "C, "
             << "PeakMemTemp=" << peakMemTemp << "C, "
             << "Decision=" << decision_str << ", "
             << "LastStallBalancedCore=" << lastStallBalancedCoreIndex << std::endl;
    }
}

// ============================================================================
// DVFSCoreMem Implementation
// ============================================================================

DVFSCoreMem::DVFSCoreMem(
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
    float beta)
    : m_performanceCounters(performanceCounters)
    , m_dtmManager(dtmManager)
    , m_numberOfCores(numberOfCores)
    , m_minFrequency(minFrequency)
    , m_maxFrequency(maxFrequency)
    , m_frequencyStepSize(frequencyStepSize)
    , m_dvfs_temp_margins(dvfs_temp_margins)
    , m_dvfs_frequencies(dvfs_frequencies)
    , m_sb_temp_thresholds_C(sb_temp_thresholds_C)
    , m_sb_stall_counts_S(sb_stall_counts_S)
    , m_alpha(alpha)
    , m_beta(beta)
    , m_dvfs_initialized(false)
{}

std::vector<int> DVFSCoreMem::getFrequencies(
    const std::vector<int> &oldFrequencies,
    const std::vector<bool> &activeCores)
{
    // --- Step 1: Gather Current System State ---
    std::vector<float> current_temps(m_numberOfCores);
    std::cout << "[DVFSCoreMem] Current core temperatures: ";
    for (int i = 0; i < m_numberOfCores; ++i) {
        current_temps[i] = m_performanceCounters->getTemperatureOfCore(i);
        std::cout << i << ":" << current_temps[i] << " ";
    }
    std::cout << std::endl;

    float T_max = *std::max_element(current_temps.begin(), current_temps.end());
    float T_avg = std::accumulate(current_temps.begin(), current_temps.end(), 0.0f) / m_numberOfCores;
    float T_P = m_dtmManager->getCoreTempLimit();

    // Update both K-epoch and current epoch max temperatures
    m_dtmManager->updateKEpochMaxTemps(current_temps);
    m_dtmManager->updateCurrentEpochMaxTemps(current_temps);
    m_dtmManager->incrementEpoch();

    // --- Step 2: Coarse-Grained DVFS (runs every K epochs) or first run ---
    if (m_dtmManager->isKthEpoch() || !m_dvfs_initialized) {
        const std::vector<float>& k_epoch_temps = m_dtmManager->getKEpochMaxTemps();
        std::vector<int> new_baselines(m_numberOfCores);
        for (int i = 0; i < m_numberOfCores; ++i) {
            new_baselines[i] = get_baseline_frequency(k_epoch_temps[i], T_P);
        }
        m_dtmManager->setBaselineFrequencies(new_baselines);
        m_dtmManager->resetKEpochMaxTemps();
        // Don't reset current epoch temps here - they reset at end of each epoch

        // --- Coordinated Decision with Memory DTM ---
        // Find Peak Mem Temp
        int m_numberOfBanks = Sim()->getCfg()->getInt("memory/num_banks");
        std::vector<std::pair<float, int>> bank_temps; // {temp, bank_id}
        float T_mem_max = 0.0f;
        for (int i = 0; i < m_numberOfBanks; ++i) {
            float temp = m_performanceCounters->getTemperatureOfBank(i);
            bank_temps.push_back({temp, i});
            if (temp > T_mem_max) {
                T_mem_max = temp;
            }
        }
        m_dtmManager->makeCoordinatedDecision(T_max, T_mem_max);
        m_dvfs_initialized = true;
    }

    // --- Step 3: Fine-Grained Stall Balancing (runs every epoch) ---
    // Start with the current baseline frequencies from the manager
    std::vector<int> final_frequencies = m_dtmManager->getBaselineFrequencies();

    if(T_max >= m_dtmManager->getCoreTempLimit()){
        std::cout << "[DVFSCoreMem] Peak core temp " << T_max << "C exceeds limit " << m_dtmManager->getCoreTempLimit() << ", applying Low Power Mode" << std::endl;
        for(int i = 0; i < m_numberOfCores; i++){
            final_frequencies[i] = m_minFrequency;
        }
        return final_frequencies;
    }

    if(m_dtmManager->getLastDecision() != CORE) {
        return final_frequencies; // No stall balancing if CoreDTM wasn't applied this epoch
    }

    // Determine how many cores to stall
    int S = get_SB_count(T_max, T_avg, T_P);

    if (S > 0) {
        std::cout << "[DVFSCoreMem] Applying stall balancing with S=" << S << " at T_max=" << T_max << "C" << std::endl;

        int last_index = m_dtmManager->getLastStallBalancedCore();
        int new_last_index = last_index;

        for (int i = 1; i <= S; ++i) {
            int core_to_stall = (last_index + i) % m_numberOfCores;
            
            // Downgrade frequency by one step, ensuring it doesn't go below min
            int new_freq = final_frequencies[core_to_stall] - m_frequencyStepSize;
            if (new_freq < m_minFrequency) {
                new_freq = m_minFrequency;
            }
            final_frequencies[core_to_stall] = new_freq;

            new_last_index = core_to_stall;
        }
        m_dtmManager->setLastStallBalancedCore(new_last_index);
        m_dtmManager->setBaselineFrequencies(final_frequencies);
    }
    
    return final_frequencies;
}

int DVFSCoreMem::get_baseline_frequency(float T_dvfs, float T_P)
{
    // m_dvfs_temp_margins: e.g. [4.0, 1.5, 1.0, -1]  (sentinel < 0)
    // m_dvfs_frequencies: e.g. [3600, 3000, 2400, 1800, ...] (MHz)
    size_t n_margins = m_dvfs_temp_margins.size();
    size_t n_freqs = m_dvfs_frequencies.size();

    // Defensive: frequencies must be at least margins + 1
    if (n_freqs == 0) return 0;
    if (n_margins + 1 > n_freqs) {
        // fallback: return highest available or last
        return m_dvfs_frequencies.back();
    }

    for (size_t i = 0; i < n_margins; ++i) {
        float margin = m_dvfs_temp_margins[i];
        if (margin < 0.0f) break; // sentinel -> stop
        float threshold = T_P - margin;
        // If temperature is <= threshold, we're in the safer (higher-frequency) bin
        if (T_dvfs <= threshold) {
            return m_dvfs_frequencies[i];
        }
    }

    // If temperature exceeded all thresholds, return the last (lowest) frequency
    return m_dvfs_frequencies[n_margins < n_freqs ? n_margins : (n_freqs - 1)];
}

int DVFSCoreMem::get_SB_count(float T_max, float T_avg, float T_P)
{
    float thermal_slack = T_P - T_max;
    int S = 0;

    // Determine base stall count based on thermal slack
    for (size_t i = 0; i < m_sb_temp_thresholds_C.size(); ++i) {
        std::cout << "[DEBUG][DVFSCoreMem] Thermal slack: " << thermal_slack << "C, Threshold: " << m_sb_temp_thresholds_C[i] << "C" << std::endl;
        if (thermal_slack < m_sb_temp_thresholds_C[i]) {
            S = m_sb_stall_counts_S[i];
        } else {
            // Stop at the first threshold we don't cross
            break; 
        }
    }
    
    // If it's a localized hotspot, reduce the number of stalls
    if (S > 0 && (T_max - T_avg) > m_alpha) {
        S = std::max(0, S - (int)m_beta);
    }

    return S;
}


// ============================================================================
// DramCoreMem Implementation
// ============================================================================

DramCoreMem::DramCoreMem(
    const PerformanceCounters *performanceCounters,
    int numberOfBanks,
    DtmStateManager *dtmManager,
    const std::vector<float>& tempThresholds_M,
    const std::vector<int>& channelCounts_L)
    : m_performanceCounters(performanceCounters)
    , m_dtmManager(dtmManager)
    , m_numberOfBanks(numberOfBanks)
    , m_tempThresholds_M(tempThresholds_M)
    , m_channelCounts_L(channelCounts_L)
{}

std::map<int, int> DramCoreMem::getNewBankModes(std::map<int, int> old_bank_modes)
{
    // Start with all banks in normal power mode (1)
    std::map<int, int> new_bank_modes;
    for (int i = 0; i < m_numberOfBanks; ++i) {
        new_bank_modes[i] = 1; 
    }

    // --- Step 1: Gather Current Memory Temperatures ---
    std::vector<std::pair<float, int>> bank_temps; // {temp, bank_id}
    float T_mem_max = 0.0f;
    std::cout << "[DEBUG][DramCoreMem] Bank Temperatures: ";
    for (int i = 0; i < m_numberOfBanks; ++i) {
        float temp = m_performanceCounters->getTemperatureOfBank(i);
        bank_temps.push_back({temp, i});
        if (temp > T_mem_max) {
            T_mem_max = temp;
        }
        std::cout << i << ":" << temp << " ";
    }
    std::cout << std::endl;

    if(m_dtmManager->getLastDecision() == MEMORY || (T_mem_max >= m_dtmManager->getMemTempLimit())) {        
        float T_M = m_dtmManager->getMemTempLimit(); // Get T_M from the manager

        // --- Step 2: Determine Aggressiveness (how many channels to disable) ---
        int channels_to_disable = 0;
        for (size_t i = 0; i < m_tempThresholds_M.size(); ++i) {
            // Check if peak temp has crossed the absolute threshold (T_M - margin)
            if (T_mem_max > (T_M - m_tempThresholds_M[i])) {
                channels_to_disable = m_channelCounts_L[i];
            }
        }

        // --- Step 3: Select Hottest Channels and Apply LPM ---
        if (channels_to_disable > 0) {
            // Sort banks by temperature in descending order
            std::sort(bank_temps.rbegin(), bank_temps.rend());

            // Place the L_i hottest banks into low power mode (0)
            for (int i = 0; i < channels_to_disable; ++i) {
                int bank_id_to_disable = bank_temps[i].second;
                new_bank_modes[bank_id_to_disable] = 0;
            }
        }
    }

    // --- Step 4: Log state using current epoch peak temperatures ---
    SubsecondTime currentTime = Sim()->getClockSkewMinimizationServer()->getGlobalTime();
    float peakCoreTemp = m_dtmManager->getCurrentEpochPeakTemp();
    
    m_dtmManager->logCurrentState(currentTime, peakCoreTemp, T_mem_max);
    
    // Reset current epoch temps after logging (end of epoch)
    m_dtmManager->resetCurrentEpochMaxTemps();

    return new_bank_modes;
}