##---------------IMPORTANT--------------------
## SET SCHEDULER/TYPE TO `cfs_lite` IN BASE.CFG FILE BEFORE RUNNING THIS MAKE FILE
## (dtmAdaptive only works with cfs_lite!)

SNIPER_ROOT = ../../..
SPLASH_ROOT = $(SNIPER_ROOT)/benchmarks/splash2/splash2/codes
BARNES_ROOT = $(SPLASH_ROOT)/apps/barnes

run:
	rm -f run_benchmarks*
	@for i in $$(seq 0 23); do \
		mkfifo run_benchmarks.app$$i.th0.sift run_benchmarks_response.app$$i.th0.sift; \
		$(SNIPER_ROOT)/record-trace -o run_benchmarks -e 1 -s $$i -r 1 --follow --routine-tracing -- $(BARNES_ROOT)/BARNES < input & \
	done
	$(SNIPER_ROOT)/run-sniper \
		-v \
		-s memTherm_core \
		-c gainestown_2_5D_16core \
		-g --scheduler/type=cfs_lite \
		-g --traceinput/enabled=true \
		-g --traceinput/emulate_syscalls=true \
		-g --traceinput/num_apps=24 \
		-g --traceinput/trace_prefix=run_benchmarks \
		--trace-manual \
		--nosift ; \
	wait

# --- Cleanup ---
# Extra files to remove when you type `make clean`
CLEAN_EXTRA = viz hotspot/ *.log *.init energystats-temp.* *.trace tmp *.cfg run_benchmarks*

clean:
	$(RM) -r $(CLEAN_EXTRA)
