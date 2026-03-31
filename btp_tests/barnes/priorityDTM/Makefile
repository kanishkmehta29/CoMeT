##---------------IMPORTANT--------------------
## SET SCHEDULER/TYPE TO `cfs_lite` IN BASE.CFG FILE BEFORE RUNNING THIS MAKE FILE
## (dtmAdaptive only works with cfs_lite!)

SNIPER_ROOT = ../../..
SPLASH_ROOT = $(SNIPER_ROOT)/benchmarks/splash2/splash2/codes
BARNES_ROOT = $(SPLASH_ROOT)/apps/barnes

run:
	rm -f run_benchmarks*
	mkfifo run_benchmarks.app0.th0.sift run_benchmarks_response.app0.th0.sift
	mkfifo run_benchmarks.app1.th0.sift run_benchmarks_response.app1.th0.sift
	$(SNIPER_ROOT)/record-trace -o run_benchmarks -e 1 -s 0 -r 1 --follow --routine-tracing -- $(BARNES_ROOT)/BARNES < input & \
	$(SNIPER_ROOT)/record-trace -o run_benchmarks -e 1 -s 1 -r 1 --follow --routine-tracing -- $(BARNES_ROOT)/BARNES < input & \
	$(SNIPER_ROOT)/run-sniper \
		-v \
		-s memTherm_core \
		-c gainestown_2_5D_16core \
		-g --scheduler/type=cfs_lite \
		-g --scheduler/cfs_lite/priority_mode=per_task \
		-g --scheduler/cfs_lite/per_task_priorities=-6,1 \
		-g --traceinput/enabled=true \
		-g --traceinput/emulate_syscalls=true \
		-g --traceinput/num_apps=2 \
		-g --traceinput/trace_prefix=run_benchmarks \
		--trace-manual \
		--nosift ; \
	wait

# --- Cleanup ---
# Extra files to remove when you type `make clean`
CLEAN_EXTRA = viz hotspot/ *.log *.init energystats-temp.* *.trace tmp *.cfg run_benchmarks*

clean:
	$(RM) -r $(CLEAN_EXTRA)
