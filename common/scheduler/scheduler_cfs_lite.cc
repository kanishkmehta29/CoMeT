#include "scheduler_cfs_lite.h"

#include "simulator.h"
#include "config.hpp"
#include "thread.h"
#include "core_manager.h"
#include "performance_model.h"

#include <algorithm>
#include <cmath>
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
   , m_last_periodic(SubsecondTime::Zero())
   , m_core_thread_running(Sim()->getConfig()->getApplicationCores(), INVALID_THREAD_ID)
   , m_slice_left(Sim()->getConfig()->getApplicationCores(), SubsecondTime::Zero())
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
}

void SchedulerCFSLite::ensureThreadInfoSize(thread_id_t thread_id)
{
   if (m_thread_info.size() <= (size_t)thread_id)
      m_thread_info.resize(thread_id + 16);
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

core_id_t SchedulerCFSLite::threadCreate(thread_id_t thread_id)
{
   ensureThreadInfoSize(thread_id);

   if (!m_thread_info[thread_id].hasAffinity())
      threadSetInitialAffinity(thread_id);

   int priority = getThreadPriority(thread_id);
   m_thread_info[thread_id].setPriority(priority);
   m_thread_info[thread_id].setWeight(priorityToWeight(priority));

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

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      if (m_core_thread_running[core_id] == INVALID_THREAD_ID && m_thread_info[thread_id].hasAffinity(core_id))
      {
         SubsecondTime now = Sim()->getClockSkewMinimizationServer()->getGlobalTime();
         m_core_thread_running[core_id] = thread_id;
         m_thread_info[thread_id].setCoreRunning(core_id);
         m_thread_info[thread_id].setLastScheduledIn(now);
         m_slice_left[core_id] = computeTimeSlice(thread_id);
         return core_id;
      }
   }

   return INVALID_CORE_ID;
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
   thread_id_t best = INVALID_THREAD_ID;
   double best_vruntime = std::numeric_limits<double>::max();

   for (thread_id_t tid = 0; tid < (thread_id_t)m_threads_runnable.size(); ++tid)
   {
      if (tid >= (thread_id_t)m_thread_info.size())
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

   moveThread(thread_id, INVALID_CORE_ID, time);
}

void SchedulerCFSLite::rescheduleCore(SubsecondTime time, core_id_t core_id, bool force_reschedule)
{
   thread_id_t current = m_core_thread_running[core_id];

   if (current != INVALID_THREAD_ID && Sim()->getThreadManager()->getThreadState(current) == Core::INITIALIZING)
      return;

   thread_id_t next = pickNextThread(core_id, time);

   if (!force_reschedule && current != INVALID_THREAD_ID && current == next)
      return;

   if (current != INVALID_THREAD_ID && current != next)
   {
      m_thread_info[current].setCoreRunning(INVALID_CORE_ID);
      m_thread_info[current].setLastScheduledOut(time + SubsecondTime::PS(core_id));
      moveThread(current, INVALID_CORE_ID, time);
   }

   m_core_thread_running[core_id] = next;

   if (next != INVALID_THREAD_ID)
   {
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
}

void SchedulerCFSLite::migrateThread(thread_id_t thread_id, core_id_t to_core, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size() || to_core >= (core_id_t)m_core_thread_running.size())
      return;

   if (!m_thread_info[thread_id].hasAffinity(to_core))
      return;

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

   // TODO: call DTM policy module here and consume its decisions.
   // TODO: DTM should be able to request `threadYield(thread_id)` and/or `migrateThread(thread_id, core_id, time)`.
   (void)time;
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

   m_last_periodic = time;
}

void SchedulerCFSLite::threadStart(thread_id_t thread_id, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      if (m_core_thread_running[core_id] == INVALID_THREAD_ID && m_thread_info[thread_id].hasAffinity(core_id))
      {
         rescheduleCore(time, core_id, true);
         break;
      }
   }
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

   for (core_id_t core_id = 0; core_id < (core_id_t)m_core_thread_running.size(); ++core_id)
   {
      if (m_core_thread_running[core_id] == INVALID_THREAD_ID && m_thread_info[thread_id].hasAffinity(core_id))
      {
         rescheduleCore(time, core_id, true);
         return;
      }
   }
}

void SchedulerCFSLite::threadExit(thread_id_t thread_id, SubsecondTime time)
{
   if (thread_id >= (thread_id_t)m_thread_info.size())
      return;

   core_id_t core_id = m_thread_info[thread_id].getCoreRunning();
   if (core_id != INVALID_CORE_ID)
   {
      unscheduleThread(thread_id, time);
      rescheduleCore(time, core_id, true);
   }
}
