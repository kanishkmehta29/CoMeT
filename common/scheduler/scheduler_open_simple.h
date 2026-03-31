#ifndef __SCHEDULER_OPEN_SIMPLE_H
#define __SCHEDULER_OPEN_SIMPLE_H

#include "scheduler_pinned_base.h"
#include "performance_counters.h"
#include "policies/dvfspolicy.h"
#include "policies/drampolicy.h"

class SchedulerOpenSimple : public SchedulerPinnedBase
{
   public:
      SchedulerOpenSimple(ThreadManager *thread_manager);
      virtual ~SchedulerOpenSimple();

      virtual void periodic(SubsecondTime time);
      virtual void threadSetInitialAffinity(thread_id_t thread_id);

   private:
      void initDVFSPolicy(const String &logic);
      void initDramPolicy(const String &logic);
      void executeDVFSPolicy();
      void executeDramPolicy();

      void setFrequency(int core_id, int frequency);
      void setMemBankMode(int bank_id, int mode);

      core_id_t getNextCore(core_id_t core_id);
      core_id_t getFreeCore(core_id_t core_first);

      PerformanceCounters *m_performance_counters;
      DVFSPolicy *m_dvfs_policy;
      DramPolicy *m_dram_policy;

      const int m_interleaving;
      std::vector<bool> m_core_mask;
      core_id_t m_next_core;

      int m_number_of_cores;
      int m_number_of_banks;
      int m_min_frequency;
      int m_max_frequency;
      int m_frequency_step;
      long m_dvfs_epoch;
      long m_dram_epoch;
};

#endif // __SCHEDULER_OPEN_SIMPLE_H
