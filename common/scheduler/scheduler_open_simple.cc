#include "scheduler_open_simple.h"

#include "config.hpp"
#include "simulator.h"
#include "magic_server.h"
#include "policies/coreMemDTM.h"

#include <cstdlib>
#include <iostream>
#include <map>

namespace {
long getCfgLongWithFallback(const char *primary_key, const char *fallback_key, long fallback_value)
{
   try { return atol(Sim()->getCfg()->getString(primary_key).c_str()); }
   catch (...) {}
   try { return atol(Sim()->getCfg()->getString(fallback_key).c_str()); }
   catch (...) {}
   return fallback_value;
}

int getCfgIntWithFallback(const char *primary_key, const char *fallback_key, int fallback_value)
{
   try { return Sim()->getCfg()->getInt(primary_key); }
   catch (...) {}
   try { return Sim()->getCfg()->getInt(fallback_key); }
   catch (...) {}
   return fallback_value;
}

bool getCfgBoolArrayWithFallback(const char *primary_key, const char *fallback_key, int index, bool fallback_value)
{
   try { return Sim()->getCfg()->getBoolArray(primary_key, index); }
   catch (...) {}
   try { return Sim()->getCfg()->getBoolArray(fallback_key, index); }
   catch (...) {}
   return fallback_value;
}
}

SchedulerOpenSimple::SchedulerOpenSimple(ThreadManager *thread_manager)
   : SchedulerPinnedBase(thread_manager, SubsecondTime::NS((UInt64)getCfgIntWithFallback("scheduler/open_simple/quantum", "scheduler/pinned/quantum", 1000000)))
   , m_performance_counters(NULL)
   , m_dvfs_policy(NULL)
   , m_dram_policy(NULL)
   , m_interleaving(getCfgIntWithFallback("scheduler/open_simple/interleaving", "scheduler/pinned/interleaving", 1))
   , m_next_core(0)
   , m_number_of_cores(Sim()->getConfig()->getApplicationCores())
   , m_number_of_banks(Sim()->getCfg()->getInt("memory/num_banks"))
   , m_min_frequency((int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/min_frequency") + 0.5))
   , m_max_frequency((int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/max_frequency") + 0.5))
   , m_frequency_step((int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/frequency_step_size") + 0.5))
   , m_dvfs_epoch(getCfgLongWithFallback("scheduler/open_simple/dvfs/dvfs_epoch", "scheduler/open/dvfs/dvfs_epoch", 1000000))
   , m_dram_epoch(getCfgLongWithFallback("scheduler/open_simple/dram/dram_epoch", "scheduler/open/dram/dram_epoch", 1000000))
{
   if (m_interleaving <= 0)
   {
      std::cout << "[SchedulerOpenSimple] [Error]: scheduler/open_simple/interleaving must be > 0" << std::endl;
      exit(1);
   }

   m_core_mask.resize(m_number_of_cores);
   for (core_id_t core_id = 0; core_id < (core_id_t)m_number_of_cores; ++core_id)
   {
      m_core_mask[core_id] = getCfgBoolArrayWithFallback("scheduler/open_simple/core_mask", "scheduler/open/core_mask", core_id, true);
   }

   bool found_allowed_core = false;
   for (core_id_t core_id = 0; core_id < (core_id_t)m_number_of_cores; ++core_id)
   {
      if (m_core_mask[core_id])
      {
         m_next_core = core_id;
         found_allowed_core = true;
         break;
      }
   }

   if (!found_allowed_core)
   {
      std::cout << "[SchedulerOpenSimple] [Error]: scheduler/open_simple/core_mask disables all cores" << std::endl;
      exit(1);
   }

   m_performance_counters = new PerformanceCounters(
      Sim()->getCfg()->getString("hotspot/log_files/combined_instpower_trace_file").c_str(),
      Sim()->getCfg()->getString("hotspot/log_files/combined_insttemperature_trace_file").c_str(),
      "InstantaneousCPIStack.log",
      Sim()->getCfg()->getString("reliability/log_files/instant_trace_file").c_str(),
      "InstantVdd.log",
      Sim()->getCfg()->getString("reliability/log_files/delta_v_file").c_str());

   initDVFSPolicy(Sim()->getCfg()->getString("scheduler/open_simple/dvfs/logic"));
   initDramPolicy(Sim()->getCfg()->getString("scheduler/open_simple/dram/dtm"));
}

SchedulerOpenSimple::~SchedulerOpenSimple()
{
   if (m_dvfs_policy)
      delete m_dvfs_policy;
   if (m_dram_policy)
      delete m_dram_policy;
   if (m_performance_counters)
      delete m_performance_counters;
}

core_id_t SchedulerOpenSimple::getNextCore(core_id_t core_id)
{
   while (true)
   {
      core_id += m_interleaving;
      if (core_id >= (core_id_t)m_number_of_cores)
      {
         core_id %= m_number_of_cores;
         core_id += 1;
         core_id %= m_interleaving;
      }
      if (m_core_mask[core_id])
         return core_id;
   }
}

core_id_t SchedulerOpenSimple::getFreeCore(core_id_t core_first)
{
   core_id_t core_next = core_first;

   do
   {
      if (m_core_thread_running[core_next] == INVALID_THREAD_ID && m_core_mask[core_next])
         return core_next;

      core_next = getNextCore(core_next);
   }
   while (core_next != core_first);

   return core_first;
}

void SchedulerOpenSimple::threadSetInitialAffinity(thread_id_t thread_id)
{
   core_id_t core_id = getFreeCore(m_next_core);
   m_next_core = getNextCore(core_id);

   m_thread_info[thread_id].setAffinitySingle(core_id);
}

void SchedulerOpenSimple::setFrequency(int core_id, int frequency)
{
   if (frequency < m_min_frequency)
      frequency = m_min_frequency;
   if (frequency > m_max_frequency)
      frequency = m_max_frequency;

   if (Sim()->getMagicServer()->getFrequency(core_id) != frequency)
      Sim()->getMagicServer()->setFrequency(core_id, frequency);
}

void SchedulerOpenSimple::executeDVFSPolicy()
{
   if (!m_dvfs_policy)
      return;

   std::vector<int> old_frequencies;
   std::vector<bool> active_cores;
   old_frequencies.reserve(m_number_of_cores);
   active_cores.reserve(m_number_of_cores);

   for (int core = 0; core < m_number_of_cores; ++core)
   {
      old_frequencies.push_back(Sim()->getMagicServer()->getFrequency(core));
      active_cores.push_back(m_core_thread_running[core] != INVALID_THREAD_ID);
   }

   std::vector<int> new_frequencies = m_dvfs_policy->getFrequencies(old_frequencies, active_cores);
   if ((int)new_frequencies.size() != m_number_of_cores)
   {
      std::cout << "[SchedulerOpenSimple] [Error]: DVFS policy returned " << new_frequencies.size()
                << " frequencies for " << m_number_of_cores << " cores" << std::endl;
      exit(1);
   }

   for (int core = 0; core < m_number_of_cores; ++core)
   {
      setFrequency(core, new_frequencies[core]);
   }

   m_performance_counters->notifyFreqsOfCores(new_frequencies);
}

void SchedulerOpenSimple::initDVFSPolicy(const String &logic)
{
   if (logic == "off")
   {
      m_dvfs_policy = NULL;
      return;
   }

   if (logic != "coreMemDTM")
   {
      std::cout << "[SchedulerOpenSimple] [Error]: Unsupported DVFS logic '" << logic
                << "'. Supported values: off, coreMemDTM" << std::endl;
      exit(1);
   }

   DtmStateManager *dtmManager = DtmStateManager::getInstance();

   std::vector<float> dvfs_temp_margins;
   std::vector<int> dvfs_frequencies;
   for (int i = 0; ; ++i)
   {
      float margin = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/dvfs_temp_margins", i);
      if (margin < 0)
         break;
      dvfs_temp_margins.push_back(margin);
   }
   for (int i = 0; ; ++i)
   {
      float freq_ghz = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/dvfs_frequencies_ghz", i);
      if (freq_ghz < 0)
         break;
      dvfs_frequencies.push_back((int)(freq_ghz * 1000));
   }

   std::vector<float> sb_temp_thresholds_C;
   std::vector<int> sb_stall_counts_S;
   for (int i = 0; ; ++i)
   {
      float threshold = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/sb_temp_thresholds_c", i);
      if (threshold < 0)
         break;
      sb_temp_thresholds_C.push_back(threshold);
   }
   for (int i = 0; ; ++i)
   {
      int count = Sim()->getCfg()->getIntArray("scheduler/coreMemDTM/sb_stall_counts_s", i);
      if (count < 0)
         break;
      sb_stall_counts_S.push_back(count);
   }

   float alpha = Sim()->getCfg()->getFloat("scheduler/coreMemDTM/alpha");
   float beta = Sim()->getCfg()->getFloat("scheduler/coreMemDTM/beta");

   m_dvfs_policy = new DVFSCoreMem(
      m_performance_counters,
      m_number_of_cores,
      m_min_frequency,
      m_max_frequency,
      m_frequency_step,
      dtmManager,
      dvfs_temp_margins,
      dvfs_frequencies,
      sb_temp_thresholds_C,
      sb_stall_counts_S,
      alpha,
      beta);
}

void SchedulerOpenSimple::setMemBankMode(int bank_id, int mode)
{
   Sim()->m_bank_modes[bank_id] = mode;
}

void SchedulerOpenSimple::executeDramPolicy()
{
   if (!m_dram_policy)
      return;

   std::map<int, int> old_bank_modes;
   for (int bank = 0; bank < m_number_of_banks; ++bank)
   {
      old_bank_modes[bank] = Sim()->m_bank_modes[bank];
   }

   std::map<int, int> new_bank_modes = m_dram_policy->getNewBankModes(old_bank_modes);
   for (int bank = 0; bank < m_number_of_banks; ++bank)
   {
      setMemBankMode(bank, new_bank_modes[bank]);
   }
}

void SchedulerOpenSimple::initDramPolicy(const String &logic)
{
   if (logic == "off")
   {
      m_dram_policy = NULL;
      return;
   }

   if (logic != "coreMemDTM")
   {
      std::cout << "[SchedulerOpenSimple] [Error]: Unsupported DRAM logic '" << logic
                << "'. Supported values: off, coreMemDTM" << std::endl;
      exit(1);
   }

   DtmStateManager *dtmManager = DtmStateManager::getInstance();

   std::vector<float> temp_thresholds_m;
   std::vector<int> channel_counts_l;
   for (int i = 0; ; ++i)
   {
      float threshold = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/temp_thresholds_m", i);
      int count = Sim()->getCfg()->getIntArray("scheduler/coreMemDTM/channel_counts_l", i);
      if (threshold < 0 || count < 0)
         break;
      temp_thresholds_m.push_back(threshold);
      channel_counts_l.push_back(count);
   }

   m_dram_policy = new DramCoreMem(
      m_performance_counters,
      m_number_of_banks,
      dtmManager,
      temp_thresholds_m,
      channel_counts_l);
}

void SchedulerOpenSimple::periodic(SubsecondTime time)
{
   if ((m_dvfs_policy != NULL) && m_dvfs_epoch > 0 && (time.getNS() % m_dvfs_epoch == 0))
   {
      executeDVFSPolicy();
   }

   if ((m_dram_policy != NULL) && m_dram_epoch > 0 && (time.getNS() % m_dram_epoch == 0))
   {
      executeDramPolicy();
   }

   SchedulerPinnedBase::periodic(time);
}
