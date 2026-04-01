#include "scheduler_cfs_lite.h"
#include "policies/dtmAdaptive.h"

#include "simulator.h"
#include "config.hpp"
#include "thread.h"
#include "core_manager.h"
#include "performance_model.h"
#include "magic_server.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>

SchedulerCFSLite::ThreadInfo::ThreadInfo()
   : m_has_affinity(false)
   , m_explicit_affinity(false)
   , m_core_affinity(Sim()->getConfig()->getApplicationCores(), false)
   , m_core_running(INVALID_CORE_ID)
   , m_priority(0)
   , m_weight(1024.0)
   , m_vruntime(0.0)
   , m_last_scheduled_in(SubsecondTime::Zero())
   , m_last_scheduled_out(SubsecondTime::Zero())
{}

void SchedulerCFSLite::ThreadInfo::clearAffinity()
{
   for (auto it = m_core_affinity.begin(); it != m_core_affinity.end(); ++it)
      *it = false;
   m_has_affinity = false;
}

void SchedulerCFSLite::ThreadInfo::addAffinity(core_id_t core_id)
{
   m_core_affinity[core_id] = true;
   m_has_affinity = true;
}

void SchedulerCFSLite::ThreadInfo::setAffinitySingle(core_id_t core_id)
{
   clearAffinity();
   addAffinity(core_id);
}

bool SchedulerCFSLite::ThreadInfo::hasAffinity(core_id_t core_id) const { return m_core_affinity[core_id]; }
bool SchedulerCFSLite::ThreadInfo::hasAffinity() const { return m_has_affinity; }
bool SchedulerCFSLite::ThreadInfo::hasExplicitAffinity() const { return m_explicit_affinity; }
void SchedulerCFSLite::ThreadInfo::setExplicitAffinity() { m_explicit_affinity = true; }

void SchedulerCFSLite::ThreadInfo::setCoreRunning(core_id_t core_id) { m_core_running = core_id; }
core_id_t SchedulerCFSLite::ThreadInfo::getCoreRunning() const { return m_core_running; }
bool SchedulerCFSLite::ThreadInfo::isRunning() const { return m_core_running != INVALID_CORE_ID; }

void SchedulerCFSLite::ThreadInfo::setPriority(int priority) { m_priority = priority; }
int SchedulerCFSLite::ThreadInfo::getPriority() const { return m_priority; }
void SchedulerCFSLite::ThreadInfo::setWeight(double weight) { m_weight = weight; }
double SchedulerCFSLite::ThreadInfo::getWeight() const { return m_weight; }

void SchedulerCFSLite::ThreadInfo::setVruntime(double vruntime) { m_vruntime = vruntime; }
double SchedulerCFSLite::ThreadInfo::getVruntime() const { return m_vruntime; }
void SchedulerCFSLite::ThreadInfo::addVruntime(double delta) { m_vruntime += delta; }

void SchedulerCFSLite::ThreadInfo::setLastScheduledIn(SubsecondTime time) { m_last_scheduled_in = time; }
void SchedulerCFSLite::ThreadInfo::setLastScheduledOut(SubsecondTime time) { m_last_scheduled_out = time; }
SubsecondTime SchedulerCFSLite::ThreadInfo::getLastScheduledIn() const { return m_last_scheduled_in; }
SubsecondTime SchedulerCFSLite::ThreadInfo::getLastScheduledOut() const { return m_last_scheduled_out; }

SchedulerCFSLite::SchedulerCFSLite(ThreadManager *thread_manager)
   : SchedulerDynamic(thread_manager)
   , m_target_latency(SubsecondTime::NS(Sim()->getCfg()->getInt("scheduler/cfs_lite/target_latency")))
   , m_min_granularity(SubsecondTime::NS(Sim()->getCfg()->getInt("scheduler/cfs_lite/min_granularity")))
   , m_priority_mode(Sim()->getCfg()->getString("scheduler/cfs_lite/priority_mode"))
   , m_default_priority(Sim()->getCfg()->getInt("scheduler/cfs_lite/default_priority"))
   , m_dtm_enable_migration(Sim()->getCfg()->getBool("scheduler/cfs_lite/dtm/enable_migration"))
   , m_dtm_enable_yield(Sim()->getCfg()->getBool("scheduler/cfs_lite/dtm/enable_yield"))
   , m_performance_counters(nullptr)
   , m_dvfs_policy(nullptr)
   , m_dram_policy(nullptr)
   , m_dtm_policy(nullptr)
   , m_dvfs_epoch(SubsecondTime::NS(Sim()->getCfg()->getInt("scheduler/cfs_lite/dvfs/dvfs_epoch")))
   , m_dram_epoch(SubsecondTime::NS(Sim()->getCfg()->getInt("scheduler/cfs_lite/dram/dram_epoch")))
   , m_last_periodic(SubsecondTime::Zero())
   , m_core_thread_running(Sim()->getConfig()->getApplicationCores(), INVALID_THREAD_ID)
   , m_slice_left(Sim()->getConfig()->getApplicationCores(), SubsecondTime::Zero())
   , m_core_runqueues(Sim()->getConfig()->getApplicationCores())
   , m_debug_logs_enabled(true)
   , m_debug_log_period(SubsecondTime::NS(1000000))
   , m_last_debug_log(SubsecondTime::Zero())
{
   m_core_mask.resize(Sim()->getConfig()->getApplicationCores());
   for (core_id_t core_id = 0; core_id < (core_id_t)Sim()->getConfig()->getApplicationCores(); core_id++)
      m_core_mask[core_id] = Sim()->getCfg()->getBoolArray("scheduler/cfs_lite/core_mask", core_id);

   String priorities_cfg = Sim()->getCfg()->getString("scheduler/cfs_lite/per_task_priorities");
   std::stringstream ss(priorities_cfg.c_str());
   String token;
   while (std::getline(ss, token, ','))
   {
      m_per_task_priorities.push_back(atoi(token.c_str()));
   }

   // Initialize DTM/DVFS policies for cfs_lite
   m_performance_counters = new PerformanceCounters(
       Sim()->getCfg()->getString("hotspot/log_files/combined_instpower_trace_file").c_str(),
       Sim()->getCfg()->getString("hotspot/log_files/combined_insttemperature_trace_file").c_str(),
       "InstantaneousCPIStack.log",
       Sim()->getCfg()->getString("reliability/log_files/instant_trace_file").c_str(),
       "InstantVdd.log",
       Sim()->getCfg()->getString("reliability/log_files/delta_v_file").c_str());

   initDVFSPolicy(Sim()->getCfg()->getString("scheduler/cfs_lite/dvfs/logic"));
   initDramPolicy(Sim()->getCfg()->getString("scheduler/cfs_lite/dram/dtm"));
   initDtmPolicy(Sim()->getCfg()->getString("scheduler/cfs_lite/dtm/logic"));

   std::cout << "[CFS-Lite] initialized (cores="
             << Sim()->getConfig()->getApplicationCores()
             << ", target_latency_ns=" << (unsigned long long)m_target_latency.getNS()
             << ", min_granularity_ns=" << (unsigned long long)m_min_granularity.getNS()
             << ", priority_mode=" << m_priority_mode
             << ")" << std::endl;
}

SchedulerCFSLite::~SchedulerCFSLite()
{
   if (m_dvfs_policy) delete m_dvfs_policy;
   if (m_dram_policy) delete m_dram_policy;
   if (m_dtm_policy)  delete m_dtm_policy;
   if (m_performance_counters) delete m_performance_counters;
}

void SchedulerCFSLite::ensureThreadInfoSize(thread_id_t thread_id)
{
   if (m_thread_info.size() <= (size_t)thread_id)
      m_thread_info.resize(thread_id + 16);

   if (m_thread_home_core.size() <= (size_t)thread_id)
      m_thread_home_core.resize(thread_id + 16, INVALID_CORE_ID);
}

core_id_t SchedulerCFSLite::pickHomeCoreForThread(thread_id_t thread_id) const
{
   core_id_t best_core = INVALID_CORE_ID;
   UInt64 best_load = (UInt64)-1;

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      if (!m_thread_info[thread_id].hasAffinity(core_id))
         continue;

      UInt64 load = (UInt64)m_core_runqueues[core_id].size();
      if (m_core_thread_running[core_id] != INVALID_THREAD_ID)
         ++load;

      if (load < best_load)
      {
         best_load = load;
         best_core = core_id;
      }
   }

   return best_core;
}

core_id_t SchedulerCFSLite::ensureHomeCore(thread_id_t thread_id)
{
   ensureThreadInfoSize(thread_id);

   core_id_t home = m_thread_home_core[thread_id];
   if (home != INVALID_CORE_ID && home < (core_id_t)m_core_thread_running.size() && m_thread_info[thread_id].hasAffinity(home))
      return home;

   home = pickHomeCoreForThread(thread_id);
   m_thread_home_core[thread_id] = home;

   if (home != INVALID_CORE_ID)
   {
      std::cout << "[CFS-Lite] pin t" << (unsigned long long)thread_id
                << " -> c" << (unsigned long long)home
                << std::endl;
   }

   return home;
}

bool SchedulerCFSLite::removeThreadFromRunqueue(core_id_t core_id, thread_id_t thread_id)
{
   if (core_id == INVALID_CORE_ID || core_id >= (core_id_t)m_core_runqueues.size())
      return false;

   std::deque<thread_id_t> &rq = m_core_runqueues[core_id];
   for (std::deque<thread_id_t>::iterator it = rq.begin(); it != rq.end(); ++it)
   {
      if (*it == thread_id)
      {
         rq.erase(it);
         return true;
      }
   }

   return false;
}

void SchedulerCFSLite::enqueueThreadOnHomeCore(thread_id_t thread_id)
{
   core_id_t home = ensureHomeCore(thread_id);
   if (home == INVALID_CORE_ID)
      return;

   if (!removeThreadFromRunqueue(home, thread_id))
   {
      m_core_runqueues[home].push_back(thread_id);
   }
   else
   {
      m_core_runqueues[home].push_back(thread_id);
   }
}

bool SchedulerCFSLite::hasRunnablePinnedThread(core_id_t core_id) const
{
   if (core_id >= (core_id_t)m_core_runqueues.size())
      return false;

   const std::deque<thread_id_t> &rq = m_core_runqueues[core_id];
   for (std::deque<thread_id_t>::const_iterator it = rq.begin(); it != rq.end(); ++it)
   {
      thread_id_t tid = *it;
      if (tid >= 0 && tid < (thread_id_t)m_threads_runnable.size() && m_threads_runnable[tid])
         return true;
   }

   return false;
}

UInt64 SchedulerCFSLite::countRunnableThreads() const
{
   UInt64 runnable = 0;
   for (thread_id_t tid = 0; tid < (thread_id_t)m_threads_runnable.size(); ++tid)
      if (m_threads_runnable[tid])
         ++runnable;
   return runnable;
}

UInt64 SchedulerCFSLite::countRunningThreads() const
{
   UInt64 running = 0;
   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
      if (m_core_thread_running[core_id] != INVALID_THREAD_ID)
         ++running;
   return running;
}

void SchedulerCFSLite::logCoreAssignments(const char *reason, SubsecondTime time, bool force)
{
   if (!m_debug_logs_enabled)
      return;

   if (!force && (time - m_last_debug_log) < m_debug_log_period)
      return;

   // std::ostringstream oss;
   // oss << "[CFS-Lite] map reason=" << reason
   //     << " t=" << time.getNS() << "ns"
   //     << " runnable=" << countRunnableThreads()
   //     << " running=" << countRunningThreads()
   //     << " mapping:";

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      thread_id_t tid = m_core_thread_running[core_id];
      // if (tid == INVALID_THREAD_ID)
      //    oss << " c" << core_id << "=idle";
      // else
      //    oss << " c" << core_id << "=t" << tid;
   }

   //std::cout << oss.str() << std::endl;
   m_last_debug_log = time;
}

int SchedulerCFSLite::getThreadPriority(thread_id_t thread_id) const
{
   if (m_priority_mode == "off")
      return 0;

   if (m_priority_mode == "per_task")
   {
      app_id_t app_id = Sim()->getThreadManager()->getThreadFromID(thread_id)->getAppId();
      if (!m_per_task_priorities.empty())
      {
         size_t idx = std::min((size_t)app_id, m_per_task_priorities.size() - 1);
         return m_per_task_priorities[idx];
      }
   }

   return m_default_priority;
}

double SchedulerCFSLite::priorityToWeight(int priority) const
{
   // CFS-like approximation using nice-like range [-20, 19]
   int clamped = std::max(-20, std::min(19, priority));
   return 1024.0 * std::pow(1.25, -clamped);
}

SubsecondTime SchedulerCFSLite::computeTimeSlice(thread_id_t thread_id) const
{
   double total_weight = 0.0;
   for (thread_id_t tid = 0; tid < (thread_id_t)m_threads_runnable.size(); ++tid)
   {
      if (tid < (thread_id_t)m_thread_info.size() && m_threads_runnable[tid])
         total_weight += m_thread_info[tid].getWeight();
   }

   if (total_weight <= 0.0)
      return m_target_latency;

   double thread_share = m_target_latency.getNS() * (m_thread_info[thread_id].getWeight() / total_weight);
   UInt64 slice_ns = (UInt64)std::max((double)m_min_granularity.getNS(), thread_share);
   return SubsecondTime::NS(slice_ns);
}

void SchedulerCFSLite::threadSetInitialAffinity(thread_id_t thread_id)
{
   m_thread_info[thread_id].clearAffinity();
   for (core_id_t core_id = 0; core_id < (core_id_t)Sim()->getConfig()->getApplicationCores(); ++core_id)
   {
      if (m_core_mask[core_id])
         m_thread_info[thread_id].addAffinity(core_id);
   }
}

void SchedulerCFSLite::setCoreFrequency(int core_id, int frequency)
{
   Sim()->getMagicServer()->setFrequency(core_id, frequency);
}

void SchedulerCFSLite::executeDVFSPolicy()
{
   if (!m_dvfs_policy)
      return;

   int num_cores = Sim()->getConfig()->getApplicationCores();
   std::vector<int> oldFrequencies;
   std::vector<bool> activeCores;
   oldFrequencies.reserve(num_cores);
   activeCores.reserve(num_cores);

   for (int core = 0; core < num_cores; ++core) {
      oldFrequencies.push_back(Sim()->getMagicServer()->getFrequency(core));
      activeCores.push_back(m_core_thread_running[core] != INVALID_THREAD_ID);
   }

   std::vector<int> frequencies = m_dvfs_policy->getFrequencies(oldFrequencies, activeCores);
   for (int core = 0; core < num_cores; ++core) {
      setCoreFrequency(core, frequencies[core]);
   }

   if (m_performance_counters)
      m_performance_counters->notifyFreqsOfCores(frequencies);
}

void SchedulerCFSLite::initDVFSPolicy(const String &logic)
{
   if (logic == "off") {
      m_dvfs_policy = NULL;
      return;
   }
   if (logic == "coreMemDTM") {
      DtmStateManager *dtmManager = DtmStateManager::getInstance();

      std::vector<float> dvfs_temp_margins;
      std::vector<int> dvfs_frequencies;
      for (int i = 0; ; ++i) {
         float margin = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/dvfs_temp_margins", i);
         if (margin < 0) break;
         dvfs_temp_margins.push_back(margin);
      }
      for (int i = 0; ; ++i) {
         float freq_ghz = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/dvfs_frequencies_ghz", i);
         if (freq_ghz < 0) break;
         dvfs_frequencies.push_back((int)(freq_ghz * 1000));
      }

      std::vector<float> sb_temp_thresholds_C;
      std::vector<int> sb_stall_counts_S;
      for (int i = 0; ; ++i) {
         float threshold = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/sb_temp_thresholds_c", i);
         if (threshold < 0) break;
         sb_temp_thresholds_C.push_back(threshold);
      }
      for (int i = 0; ; ++i) {
         int count = Sim()->getCfg()->getIntArray("scheduler/coreMemDTM/sb_stall_counts_s", i);
         if (count < 0) break;
         sb_stall_counts_S.push_back(count);
      }

      float alpha = Sim()->getCfg()->getFloat("scheduler/coreMemDTM/alpha");
      float beta = Sim()->getCfg()->getFloat("scheduler/coreMemDTM/beta");

      int core_min_freq = (int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/min_frequency") + 0.5);
      int core_max_freq = (int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/max_frequency") + 0.5);
      int freq_step = (int)(1000 * Sim()->getCfg()->getFloat("perf_model/core/frequency_step_size") + 0.5);

      m_dvfs_policy = new DVFSCoreMem(
          m_performance_counters,
          Sim()->getConfig()->getApplicationCores(),
          core_min_freq,
          core_max_freq,
          freq_step,
          dtmManager,
          dvfs_temp_margins,
          dvfs_frequencies,
          sb_temp_thresholds_C,
          sb_stall_counts_S,
          alpha,
          beta
      );
   }
   else {
      m_dvfs_policy = NULL;
   }
}

void SchedulerCFSLite::setMemBankMode(int bank_id, int mode)
{
   Sim()->m_bank_modes[bank_id] = mode;
}

void SchedulerCFSLite::executeDramPolicy()
{
   if (!m_dram_policy)
      return;

   int num_banks = Sim()->getCfg()->getInt("memory/num_banks");
   std::map<int,int> old_bank_modes;
   for (int i = 0; i < num_banks; ++i) {
      old_bank_modes[i] = Sim()->m_bank_modes[i];
   }
   std::map<int,int> new_bank_modes = m_dram_policy->getNewBankModes(old_bank_modes);
   for (int i = 0; i < num_banks; ++i) {
      setMemBankMode(i, new_bank_modes[i]);
   }
}

void SchedulerCFSLite::initDramPolicy(const String &logic)
{
   if (logic == "off") {
      m_dram_policy = NULL;
      return;
   }

   if (logic == "coreMemDTM") {
      DtmStateManager *dtmManager = DtmStateManager::getInstance();

      std::vector<float> temp_thresholds_m;
      std::vector<int> channel_counts_l;
      for (int i = 0; ; ++i) {
         float threshold = Sim()->getCfg()->getFloatArray("scheduler/coreMemDTM/temp_thresholds_m", i);
         int count = Sim()->getCfg()->getIntArray("scheduler/coreMemDTM/channel_counts_l", i);
         if (threshold < 0 || count < 0) break;
         temp_thresholds_m.push_back(threshold);
         channel_counts_l.push_back(count);
      }

      m_dram_policy = new DramCoreMem(
          m_performance_counters,
          Sim()->getCfg()->getInt("memory/num_banks"),
          dtmManager,
          temp_thresholds_m,
          channel_counts_l
      );
   }
   else {
      m_dram_policy = NULL;
   }
}

/**
 * initDtmPolicy
 * Instantiate the DTM policy named in base.cfg:
 *   [scheduler/cfs_lite/dtm]
 *   logic = off | adaptive
 *
 * Supported values:
 *   off       — no thermal management (default)
 *   adaptive  — priority-aware adaptive policy (DtmAdaptive)
 *
 * To add a new policy:
 *   1. Implement YourPolicy : public DtmPolicy in policies/.
 *   2. Add an else-if branch here.
 *   3. Add a [scheduler/cfs_lite/dtm/your_name] section to base.cfg.
 */
void SchedulerCFSLite::initDtmPolicy(const String &logic)
{
   if (logic == "off" || logic == "")
   {
      m_dtm_policy = nullptr;
      return;
   }

   if (logic == "adaptive")
   {
      int num_cores  = (int)Sim()->getConfig()->getApplicationCores();
      int cores_in_x = Sim()->getCfg()->getInt("memory/cores_in_x");
      int cores_in_y = Sim()->getCfg()->getInt("memory/cores_in_y");
      int num_banks  = Sim()->getCfg()->getInt("memory/num_banks");
      int num_channels= Sim()->getCfg()->getInt("scheduler/cfs_lite/dtm/adaptive/num_channels");
      float t_warn    = (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/t_warn");
      float t_crit    = (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/t_crit");
      float t_recover = (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/t_recover");
      float alpha_mem = (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/alpha_mem");
      int min_freq    = Sim()->getCfg()->getInt("scheduler/cfs_lite/dtm/adaptive/min_frequency");
      int freq_step   = (int)(1000 * Sim()->getCfg()->getFloat(
                           "perf_model/core/frequency_step_size") + 0.5);
      int   k_max      = Sim()->getCfg()->getInt("scheduler/cfs_lite/dtm/adaptive/k_max");
      float slack_scale= (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/slack_scale");
      float mem_thresh = (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/mem_intensity_threshold");
      float mpki_thresh= (float)Sim()->getCfg()->getFloat("scheduler/cfs_lite/dtm/adaptive/mpki_threshold");
      int   freq_hist  = Sim()->getCfg()->getInt("scheduler/cfs_lite/dtm/adaptive/freq_history_len");

      m_dtm_policy = new DtmAdaptive(
          m_performance_counters,
          num_cores, cores_in_x, cores_in_y,
          num_banks, num_channels,
          t_warn, t_crit, alpha_mem,
          min_freq, freq_step,
          t_recover, k_max, slack_scale, mem_thresh,
          mpki_thresh, freq_hist);

      std::cout << "[CFS-Lite] DTM policy: adaptive"
                << "  t_warn=" << t_warn
                << "  t_crit=" << t_crit
                << "  alpha_mem=" << alpha_mem
                << "  min_freq=" << min_freq << " MHz"
                << "  channels=" << num_channels
                << "  k_max=" << k_max
                << "  mpki_threshold=" << mpki_thresh
                << "  freq_history_len=" << freq_hist
                << std::endl;
      return;
   }

   // else if (logic == "your_new_policy") { ... }

   std::cerr << "[CFS-Lite] [Error]: Unknown DTM policy \""
             << logic << "\". Supported: off, adaptive." << std::endl;
   exit(1);
}

core_id_t SchedulerCFSLite::threadCreate(thread_id_t thread_id)
{
   ensureThreadInfoSize(thread_id);

   if (!m_thread_info[thread_id].hasAffinity())
      threadSetInitialAffinity(thread_id);

   int priority = getThreadPriority(thread_id);
   m_thread_info[thread_id].setPriority(priority);
   m_thread_info[thread_id].setWeight(priorityToWeight(priority));

   std::cout << "[CFS-Lite] threadCreate t" << thread_id
             << " priority=" << priority
             << " weight=" << m_thread_info[thread_id].getWeight()
             << std::endl;

   // Initialize vruntime to current min runnable vruntime to avoid starvation of newly created threads.
   double min_vruntime = std::numeric_limits<double>::max();
   bool found = false;
   for (thread_id_t tid = 0; tid < (thread_id_t)m_threads_runnable.size() && tid < (thread_id_t)m_thread_info.size(); ++tid)
   {
      if (m_threads_runnable[tid])
      {
         min_vruntime = std::min(min_vruntime, m_thread_info[tid].getVruntime());
         found = true;
      }
   }
   if (!found)
      min_vruntime = 0.0;
   m_thread_info[thread_id].setVruntime(min_vruntime);

   core_id_t home = ensureHomeCore(thread_id);
   if (home != INVALID_CORE_ID)
      enqueueThreadOnHomeCore(thread_id);

   SubsecondTime now = Sim()->getClockSkewMinimizationServer()->getGlobalTime();
   core_id_t assigned_core = INVALID_CORE_ID;
   if (home != INVALID_CORE_ID && m_core_thread_running[home] == INVALID_THREAD_ID)
   {
      rescheduleCore(now, home, true);
      if (m_core_thread_running[home] == thread_id)
         assigned_core = home;
      std::cout << "[CFS-Lite] threadCreate assigned t" << thread_id
                << " -> c" << home
                << " slice_ns=" << (unsigned long long)m_slice_left[home].getNS()
                << std::endl;
   }
   else
   {
      std::cout << "[CFS-Lite] threadCreate queued t" << thread_id
                << " home_c=" << home
                << std::endl;
   }

   logCoreAssignments("thread_create", now, true);

   return assigned_core;
}

void SchedulerCFSLite::threadYield(thread_id_t thread_id)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
   if (core_id == INVALID_CORE_ID)
      return;

   Core *core = Sim()->getCoreManager()->getCoreFromID(core_id);
   SubsecondTime time = core->getPerformanceModel()->getElapsedTime();

   m_slice_left[core_id] = SubsecondTime::Zero();
   rescheduleCore(time, core_id, true);
}

bool SchedulerCFSLite::threadSetAffinity(thread_id_t calling_thread_id, thread_id_t thread_id, size_t cpusetsize, const cpu_set_t *mask)
{
   ensureThreadInfoSize(thread_id);
   m_thread_info[thread_id].setExplicitAffinity();

   if (!mask)
   {
      m_thread_info[thread_id].clearAffinity();
      for (core_id_t core_id = 0; core_id < (core_id_t)Sim()->getConfig()->getApplicationCores(); ++core_id)
      {
         if (m_core_mask[core_id])
            m_thread_info[thread_id].addAffinity(core_id);
      }
   }

   core_id_t old_home = (thread_id < (thread_id_t)m_thread_home_core.size()) ? m_thread_home_core[thread_id] : INVALID_CORE_ID;
   core_id_t new_home = ensureHomeCore(thread_id);

   if (old_home != INVALID_CORE_ID && old_home != new_home)
      removeThreadFromRunqueue(old_home, thread_id);

   if (new_home != INVALID_CORE_ID)
      enqueueThreadOnHomeCore(thread_id);
   else
   {
      m_thread_info[thread_id].clearAffinity();
      for (unsigned int cpu = 0; cpu < 8 * cpusetsize; ++cpu)
      {
         if (CPU_ISSET_S(cpu, cpusetsize, mask) && cpu < Sim()->getConfig()->getApplicationCores() && m_core_mask[cpu])
            m_thread_info[thread_id].addAffinity(cpu);
      }
   }

   if (thread_id >= (thread_id_t)Sim()->getThreadManager()->getNumThreads())
      return true;

   if (thread_id == calling_thread_id)
   {
      threadYield(thread_id);
   }
   else if (m_thread_info[thread_id].isRunning() && !m_thread_info[thread_id].hasAffinity(m_thread_info[thread_id].getCoreRunning()))
   {
      core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
      m_slice_left[core_id] = SubsecondTime::Zero();
   }

   return true;
}

bool SchedulerCFSLite::threadGetAffinity(thread_id_t thread_id, size_t cpusetsize, cpu_set_t *mask)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return false;

   if (cpusetsize * 8 < Sim()->getConfig()->getApplicationCores())
      return false;

   CPU_ZERO_S(cpusetsize, mask);
   for (core_id_t core_id = 0; core_id < (core_id_t)Sim()->getConfig()->getApplicationCores(); ++core_id)
   {
      if (m_thread_info[thread_id].hasAffinity(core_id) || !m_thread_info[thread_id].hasExplicitAffinity())
         CPU_SET_S(core_id, cpusetsize, mask);
   }

   return true;
}

void SchedulerCFSLite::updateRunningVruntime(SubsecondTime now)
{
   SubsecondTime delta = now - m_last_periodic;
   double delta_ns = (double)delta.getNS();

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      thread_id_t tid = m_core_thread_running[core_id];
      if (tid == INVALID_THREAD_ID || tid >= (thread_id_t)m_thread_info.size())
         continue;

      double weight = m_thread_info[tid].getWeight();
      if (weight <= 0.0)
         weight = 1024.0;

      // CFS-like vruntime increment: delta_exec * NICE_0_WEIGHT / weight
      m_thread_info[tid].addVruntime(delta_ns * (1024.0 / weight));
   }
}

thread_id_t SchedulerCFSLite::pickNextThread(core_id_t core_id, SubsecondTime time) const
{
   (void)time;

   thread_id_t best = INVALID_THREAD_ID;
   double best_vruntime = std::numeric_limits<double>::max();

   if (core_id >= (core_id_t)m_core_runqueues.size())
      return INVALID_THREAD_ID;

   const std::deque<thread_id_t> &rq = m_core_runqueues[core_id];
   for (std::deque<thread_id_t>::const_iterator it = rq.begin(); it != rq.end(); ++it)
   {
      thread_id_t tid = *it;

      if (tid >= (thread_id_t)m_thread_info.size())
         continue;

      if (tid < 0 || tid >= (thread_id_t)m_threads_runnable.size())
         continue;

      if (!m_threads_runnable[tid])
         continue;

      if (!m_thread_info[tid].hasAffinity(core_id))
         continue;

      if (m_thread_info[tid].isRunning() && m_thread_info[tid].getCoreRunning() != core_id)
         continue;

      double candidate = m_thread_info[tid].getVruntime();
      if (candidate < best_vruntime)
      {
         best = tid;
         best_vruntime = candidate;
      }
      else if (candidate == best_vruntime && best != INVALID_THREAD_ID)
      {
         // Tie-breaker: pick the one waiting longer.
         if (m_thread_info[tid].getLastScheduledOut() < m_thread_info[best].getLastScheduledOut())
            best = tid;
      }
   }

   return best;
}

void SchedulerCFSLite::unscheduleThread(thread_id_t thread_id, SubsecondTime time)
{
   if (thread_id == INVALID_THREAD_ID || thread_id >= (thread_id_t)m_thread_info.size())
      return;

   core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
   if (core_id == INVALID_CORE_ID)
      return;

   m_thread_info[thread_id].setCoreRunning(INVALID_CORE_ID);
   m_thread_info[thread_id].setLastScheduledOut(time);
   m_core_thread_running[core_id] = INVALID_THREAD_ID;

   if (thread_id < (thread_id_t)m_thread_home_core.size() && m_thread_home_core[thread_id] == core_id)
      enqueueThreadOnHomeCore(thread_id);

   moveThread(thread_id, INVALID_CORE_ID, time);
}

void SchedulerCFSLite::rescheduleCore(SubsecondTime time, core_id_t core_id, bool force_reschedule)
{
   thread_id_t current = m_core_thread_running[core_id];

   if (current != INVALID_THREAD_ID && Sim()->getThreadManager()->getThreadState(current) == Core::INITIALIZING)
      return;

   thread_id_t next = pickNextThread(core_id, time);

   // if (current != next)
   // {
   //    if (next == INVALID_THREAD_ID)
   //    {
   //       std::cout << "[CFS-Lite] reschedule c" << core_id
   //                 << " t" << (unsigned long long)current
   //                 << " -> idle @ " << (unsigned long long)time.getNS() << "ns"
   //                 << std::endl;
   //    }
   //    else if (current == INVALID_THREAD_ID)
   //    {
   //       std::cout << "[CFS-Lite] reschedule c" << core_id
   //                 << " idle -> t" << (unsigned long long)next
   //                 << " @ " << (unsigned long long)time.getNS() << "ns"
   //                 << std::endl;
   //    }
   //    else
   //    {
   //       std::cout << "[CFS-Lite] reschedule c" << core_id
   //                 << " t" << (unsigned long long)current
   //                 << " -> t" << (unsigned long long)next
   //                 << " @ " << (unsigned long long)time.getNS() << "ns"
   //                 << std::endl;
   //    }
   // }

   if (!force_reschedule && current != INVALID_THREAD_ID && current == next)
      return;

   if (current != INVALID_THREAD_ID && current != next)
   {
      m_thread_info[current].setCoreRunning(INVALID_CORE_ID);
      m_thread_info[current].setLastScheduledOut(time + SubsecondTime::PS(core_id));
      if (current < (thread_id_t)m_thread_home_core.size() && m_thread_home_core[current] == core_id)
         enqueueThreadOnHomeCore(current);
      moveThread(current, INVALID_CORE_ID, time);
   }

   m_core_thread_running[core_id] = next;

   if (next != INVALID_THREAD_ID)
   {
      removeThreadFromRunqueue(core_id, next);

      if (m_thread_info[next].isRunning() && m_thread_info[next].getCoreRunning() != core_id)
      {
         core_id_t old_core = m_thread_info[next].getCoreRunning();
         m_core_thread_running[old_core] = INVALID_THREAD_ID;
      }

      m_thread_info[next].setCoreRunning(core_id);
      m_thread_info[next].setLastScheduledIn(time);
      moveThread(next, core_id, time);
      m_slice_left[core_id] = computeTimeSlice(next);
   }
   else
   {
      m_slice_left[core_id] = m_min_granularity;
   }

   if (current != next)
      logCoreAssignments("reschedule", time, true);
}

void SchedulerCFSLite::migrateThread(thread_id_t thread_id, core_id_t to_core, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size() || to_core >= (core_id_t)m_core_thread_running.size())
      return;

   if (!m_thread_info[thread_id].hasAffinity(to_core))
      return;

   core_id_t old_home = ensureHomeCore(thread_id);
   if (old_home != INVALID_CORE_ID)
      removeThreadFromRunqueue(old_home, thread_id);

   m_thread_home_core[thread_id] = to_core;
   enqueueThreadOnHomeCore(thread_id);

   std::cout << "[CFS-Lite] migrate pin t" << (unsigned long long)thread_id
             << " c" << (unsigned long long)old_home
             << " -> c" << (unsigned long long)to_core
             << std::endl;

   if (m_core_thread_running[to_core] != INVALID_THREAD_ID)
      return;

   core_id_t from_core = m_thread_info[thread_id].getCoreRunning();
   if (from_core == INVALID_CORE_ID || from_core == to_core)
      return;

   m_core_thread_running[from_core] = INVALID_THREAD_ID;
   m_core_thread_running[to_core] = thread_id;
   m_thread_info[thread_id].setCoreRunning(to_core);

   moveThread(thread_id, to_core, time);
   m_slice_left[to_core] = computeTimeSlice(thread_id);
}

void SchedulerCFSLite::handleDTM(SubsecondTime time)
{
   if (!m_dtm_enable_migration && !m_dtm_enable_yield)
      return;

   if (!m_dtm_policy)
      return;

   // Build a lightweight scheduler-state snapshot for the policy.
   // Using plain ints so the policy stays decoupled from Sniper internals.
   int num_cores = (int)m_core_thread_running.size();

   std::vector<int> running(num_cores, -1);
   for (int c = 0; c < num_cores; ++c)
   {
      thread_id_t tid = m_core_thread_running[c];
      if (tid != INVALID_THREAD_ID)
         running[c] = (int)tid;
   }

   std::map<int,double> weights;
   for (thread_id_t tid = 0; tid < (thread_id_t)m_thread_info.size(); ++tid)
      weights[(int)tid] = m_thread_info[tid].getWeight();

   std::vector<bool> rq_empty(num_cores, true);
   for (int c = 0; c < num_cores; ++c)
      rq_empty[c] = m_core_runqueues[c].empty();

   // Ask the policy for decisions, then execute each one.
   std::vector<DtmDecision> decisions =
      m_dtm_policy->getDecisions(running, weights, rq_empty);

   for (const DtmDecision& d : decisions)
   {
      switch (d.action)
      {
         case DtmAction::DVFS:
            setCoreFrequency(d.core_id, d.target_freq);
            break;
         case DtmAction::YIELD:
            if (m_dtm_enable_yield)
               threadYield((thread_id_t)d.thread_id);
            break;
         case DtmAction::MIGRATE:
            if (m_dtm_enable_migration)
               migrateThread((thread_id_t)d.thread_id,
                             (core_id_t)d.target_core, time);
            break;
         case DtmAction::DRAM_MODE:
            // Throttle or recover a DRAM bank's power mode.
            // mode 0 = normal latency (~15 ns), mode 1 = low-power (~600 ns).
            // High latency creates back-pressure limiting effective bandwidth.
            setMemBankMode(d.bank_id, d.bank_mode);
            break;
         case DtmAction::NONE:
         default:
            break;
      }
   }
}

void SchedulerCFSLite::periodic(SubsecondTime time)
{
   updateRunningVruntime(time);

   handleDTM(time);

   SubsecondTime delta = time - m_last_periodic;

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      if (m_core_thread_running[core_id] == INVALID_THREAD_ID)
      {
         if (hasRunnablePinnedThread(core_id))
            rescheduleCore(time, core_id, true);
         continue;
      }

      if (delta >= m_slice_left[core_id])
      {
         rescheduleCore(time, core_id, true);
      }
      else
      {
         m_slice_left[core_id] -= delta;
      }
   }

   // Execute DTM/DVFS if active
   if ((m_dvfs_policy != NULL) && (time.getNS() % m_dvfs_epoch.getNS() == 0)) {
      executeDVFSPolicy();
   }

   if ((m_dram_policy != NULL) && (time.getNS() % m_dram_epoch.getNS() == 0)) {
      executeDramPolicy();
   }

   logCoreAssignments("periodic", time, false);

   m_last_periodic = time;
}

void SchedulerCFSLite::threadStart(thread_id_t thread_id, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   enqueueThreadOnHomeCore(thread_id);

   core_id_t home = ensureHomeCore(thread_id);
   if (home != INVALID_CORE_ID && m_core_thread_running[home] == INVALID_THREAD_ID)
      rescheduleCore(time, home, true);
}

void SchedulerCFSLite::threadStall(thread_id_t thread_id, ThreadManager::stall_type_t reason, SubsecondTime time)
{
   if (reason == ThreadManager::STALL_UNSCHEDULED)
      return;

   if (thread_id < (thread_id_t)m_thread_info.size() && m_thread_info[thread_id].isRunning())
   {
      core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
      unscheduleThread(thread_id, time);
      rescheduleCore(time, core_id, true);
   }
}

void SchedulerCFSLite::threadResume(thread_id_t thread_id, thread_id_t thread_by, SubsecondTime time)
{
   (void)thread_by;
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   enqueueThreadOnHomeCore(thread_id);

   core_id_t home = ensureHomeCore(thread_id);
   if (home != INVALID_CORE_ID && m_core_thread_running[home] == INVALID_THREAD_ID)
      rescheduleCore(time, home, true);
}

void SchedulerCFSLite::threadExit(thread_id_t thread_id, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   if (thread_id < (thread_id_t)m_thread_home_core.size())
   {
      core_id_t home = m_thread_home_core[thread_id];
      if (home != INVALID_CORE_ID)
         removeThreadFromRunqueue(home, thread_id);
      m_thread_home_core[thread_id] = INVALID_CORE_ID;
   }

   core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
   if (core_id != INVALID_CORE_ID)
   {
      unscheduleThread(thread_id, time);
      rescheduleCore(time, core_id, true);
   }

   logCoreAssignments("thread_exit", time, true);
}
