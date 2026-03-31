##---------------IMPORTANT--------------------
## SET SCHEDULER/TYPE TO `open` IN BASE.CFG FILE BEFORE RUNNING THIS MAKE FILE

SNIPER_ROOT = ../../..
SPLASH_ROOT = $(SNIPER_ROOT)/benchmarks/splash2/splash2/codes
BARNES_ROOT = $(SPLASH_ROOT)/apps/barnes

run:
	$(SNIPER_ROOT)/run-sniper \
		-v \
		-s memTherm_core \
		-c gainestown_2_5D_16core \
		-g --traceinput/benchmarks=splash2-barnes-native-16\
		-- $(BARNES_ROOT)/BARNES < input

# --- Cleanup ---
# Extra files to remove when you type `make clean`
CLEAN_EXTRA = viz hotspot/ *.log *.init energystats-temp.* *.trace tmp *.cfg

clean:
	$(RM) $(CLEAN_EXTRA)