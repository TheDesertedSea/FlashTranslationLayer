# TEST and CHECKPOINT needs to be set before calling this script
TESTNAME = test_$(CHECKPOINT)_$(TEST)
TESTOBJ = $(BUILDDIR)/$(TESTNAME).o
TESTEXE = $(BUILDDIR)/$(TESTNAME)
TESTDIR = $(TESTSDIR)/checkpoint_$(CHECKPOINT)/$(TESTNAME)

.PHONY: compile run

$(TESTOBJ): $(TESTDIR)/$(TESTNAME).cpp $(HDR) $(CONFIGMK)
	$(vecho) "Compiling $@"
	$(Q)$(CXX) $(CXXFLAGS) -c $< -o $@ || 					\
	{									\
	echo "*********************************************************"; 	\
	echo "*** Make sure ENABLE_LARGE_DATASTORE_PAGE is disabled ***"; 	\
	echo "*********************************************************";	\
	exit 3;									\
	}


$(TESTEXE): $(OBJ) $(TESTOBJ)
	$(vecho) "Compiling $@"
	$(Q)$(CXX) $^ -o $@

compile: $(TESTEXE)

run: $(TESTEXE)
	@# Don't allow the test to read any test data
	@# Students can bypass this easily, but that will be violation of
	@# course policies
	$(Q)mkdir -p $(OUTDIR)
	$(Q)cp $(TESTDIR)/$(TESTNAME).conf $(BUILDDIR)/$(TESTNAME).conf
	$(eval PERM := $(shell stat -c %a $(TESTSDIR)))
	@echo "#########################################################"
	@echo "Running $(TESTNAME)"
	@echo "Config file $(TESTDIR)/$(TESTNAME).conf"
	@echo "Output Log File $(OUTDIR)/$(TESTNAME).log"
	@echo "#########################################################"
	@chmod a-rw $(TESTSDIR)
	@# On any error, restores the permissions of the test folder
	@bash -c "								\
	trap 'trap - SIGINT SIGTERM ERR; chmod $(PERM) $(TESTSDIR); exit 1'	\
	SIGINT SIGTERM ERR;							\
	$(TESTEXE) $(BUILDDIR)/$(TESTNAME).conf $(OUTDIR)/$(TESTNAME).log;	\
	chmod $(PERM) $(TESTSDIR);						\
	"


