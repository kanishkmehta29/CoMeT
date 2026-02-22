#ifndef __SCHEDULER_CFS_LITE_H
#define __SCHEDULER_CFS_LITE_H

#include "scheduler_dynamic.h"

class SchedulerCFSLite : public SchedulerDynamic
{
   public:
      SchedulerCFSLite(ThreadManager *thread_manager);

      virtual core_id_t threadCreate(thread_id_t thread_id);
      virtual void threadYield(thread_id_t thread_id);
      virtual bool threadSetAffinity(thread_id_t calling_thread_id, thread_id_t thread_id, size_t cpusetsize, const cpu_set_t *mask);
      virtual bool threadGetAffinity(thread_id_t thread_id, size_t cpusetsize, cpu_set_t *mask);

      virtual void periodic(SubsecondTime time);
      virtual void threadStart(thread_id_t thread_id, SubsecondTime time);
      virtual void threadStall(thread_id_t thread_id, ThreadManager::stall_type_t reason, SubsecondTime time);
      virtual void threadResume(thread_id_t thread_id, thread_id_t thread_by, SubsecondTime time);
      virtual void threadExit(thread_id_t thread_id, SubsecondTime time);

   private:
      class ThreadInfo
      {
         public:
            ThreadInfo();

            void clearAffinity();
            void addAffinity(core_id_t core_id);
            void setAffinitySingle(core_id_t core_id);
            bool hasAffinity(core_id_t core_id) const;
            bool hasAffinity() const;
            bool hasExplicitAffinity() const;
            void setExplicitAffinity();

            void setCoreRunning(core_id_t core_id);
            core_id_t getCoreRunning() const;
            bool isRunning() const;

            void setPriority(int priority);
            int getPriority() const;
            void setWeight(double weight);
            double getWeight() const;

            void setVruntime(double vruntime);
            double getVruntime() const;
            void addVruntime(double delta);

            void setLastScheduledIn(SubsecondTime time);
            void setLastScheduledOut(SubsecondTime time);
            SubsecondTime getLastScheduledIn() const;
            SubsecondTime getLastScheduledOut() const;

         private:
            bool m_has_affinity;
            bool m_explicit_affinity;
            std::vector<bool> m_core_affinity;
            core_id_t m_core_running;
            int m_priority;
            double m_weight;
            double m_vruntime;
            SubsecondTime m_last_scheduled_in;
            SubsecondTime m_last_scheduled_out;
      };

      // Configuration
      SubsecondTime m_target_latency;
      SubsecondTime m_min_granularity;
      String m_priority_mode;
      int m_default_priority;
      std::vector<int> m_per_task_priorities;
      std::vector<bool> m_core_mask;

      bool m_dtm_enable_migration;
      bool m_dtm_enable_yield;

      // State
      SubsecondTime m_last_periodic;
      std::vector<ThreadInfo> m_thread_info;
      std::vector<thread_id_t> m_core_thread_running;
      std::vector<SubsecondTime> m_slice_left;

      // Helpers
      void threadSetInitialAffinity(thread_id_t thread_id);
      void ensureThreadInfoSize(thread_id_t thread_id);
      int getThreadPriority(thread_id_t thread_id) const;
      double priorityToWeight(int priority) const;
      SubsecondTime computeTimeSlice(thread_id_t thread_id) const;

      void updateRunningVruntime(SubsecondTime now);
      void rescheduleCore(SubsecondTime time, core_id_t core_id, bool force_reschedule);
      thread_id_t pickNextThread(core_id_t core_id, SubsecondTime time) const;
      void unscheduleThread(thread_id_t thread_id, SubsecondTime time);

      void migrateThread(thread_id_t thread_id, core_id_t to_core, SubsecondTime time);
      void handleDTM(SubsecondTime time);
};

#endif // __SCHEDULER_CFS_LITE_H
