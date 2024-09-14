# VERBOSE: 0 - Commands summary printed, 1 - Commands printed
VERBOSE = 0
################################################################################
#                                 -------                                      #
#                                 Caution                                      #
#                                 -------                                      #
#                       Don't edit below this line                             #
#                     Contact course staff if need to                          #
################################################################################

BASEDIR = $(PWD)
CONFIGMK= $(BASEDIR)/config.mk
BUILDDIR = $(BASEDIR)/build
SRCDIR = $(BASEDIR)/src
FUSEDIR = $(BASEDIR)/fuse
OUTDIR = $(BASEDIR)/output
TESTSDIR = $(BASEDIR)/tests
IOZONEDIR = $(BASEDIR)/iozone/src/current
INCLUDE = -I$(BASEDIR) -I$(SRCDIR) -I$(FUSE)

include $(CONFIGMK)

ifeq ("$(VERBOSE)","1")
Q :=
vecho = @true
else
Q := @
vecho = @echo
endif

ifeq ($(CONFIG_TWOPROC),1)
HDR = $(SRCDIR)/common.h $(SRCDIR)/746FlashSim.h $(SRCDIR)/746FTL.h \
      $(SRCDIR)/myFTL.h $(SRCDIR)/memcheck.h $(SRCDIR)/config.h
OBJ = $(BUILDDIR)/common.o $(BUILDDIR)/746FlashSim.o $(BUILDDIR)/memcheck.o
EXE = $(BUILDDIR)/myFTL
EXEOBJ = $(BUILDDIR)/common.o $(BUILDDIR)/746FTL.o $(BUILDDIR)/myFTL.o
else
HDR = $(SRCDIR)/common.h $(SRCDIR)/746FlashSim.h \
      $(SRCDIR)/myFTL.h $(SRCDIR)/config.h
OBJ = $(BUILDDIR)/common.o $(BUILDDIR)/746FlashSim.o \
      $(BUILDDIR)/myFTL.o
EXE =
EXEOBJ =
endif

DEFINES = -DBUILDDIR=\"$(BUILDDIR)\" -DOUTDIR=\"$(OUTDIR)\" \
	  -DCHILD_EXE_PATH=\"$(EXE)\" -DCHILD_EXE_NAME=\"myFTL\" \
	  -DCONFIG_TWOPROC=$(CONFIG_TWOPROC)

CC = /usr/bin/gcc
CXX = /usr/bin/g++
# -Wno-write-strings used to allow use of char * in place of std::string
CFLAGS =  $(INCLUDE) -Wno-deprecated-declarations -Wall -Wextra \
	 -g3 -std=c++11 -O0 -Wno-write-strings $(DEFINES)
CXXFLAGS = $(CFLAGS)

export

.PHONY: all clean veryclean perftest iozone

all: $(OBJ) $(EXE)

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp $(HDR) $(CONFIGMK)
	$(Q)mkdir -p $(BUILDDIR)
	$(vecho) "Compiling $@"
	$(Q)$(CXX) $(CXXFLAGS) -c -o $@ $<

ifeq ($(CONFIG_TWOPROC),1)
$(EXE) : $(EXEOBJ)
	$(vecho) "Linking $(BUILDDIR)/myFTL"
	$(Q)$(CXX) $(EXEOBJ) -o $@
endif

# test_x_y: Checkpoint X, Test Y
get_test_name = $(shell echo "$(1)" | sed -n 's/test_\([0-9]\+\)_\([0-9]\+\)/\2/p')
get_checkpoint = $(shell echo "$(1)" | sed -n 's/test_\([0-9]\+\)_\([0-9]\+\)/\1/p')


test_%: all
	$(eval TEST := $(call get_test_name, test_$*))
	$(eval CHECKPOINT := $(call get_checkpoint, test_$*))
	@if [ "$(CHECKPOINT)" = "" ] || [ "$(TEST)" = "" ]; then		\
		echo "Invalid Test Target. Make Failed."; 			\
		exit 2;								\
	else 									\
		make TEST=$(TEST) CHECKPOINT=$(CHECKPOINT) -C $(TESTSDIR) compile;\
	fi

run_test_%: all
	$(eval TEST := $(call get_test_name, test_$*))
	$(eval CHECKPOINT := $(call get_checkpoint, test_$*))
	@if [ "$(CHECKPOINT)" = "" ] || [ "$(TEST)" = "" ]; then		\
		echo "Invalid Test Target. Make Failed."; 			\
		exit 2;								\
	else 									\
		make TEST=$(TEST) CHECKPOINT=$(CHECKPOINT) -C $(TESTSDIR) run;	\
	fi


# Read README to see how to use fuse feature
# Note: For fuse, it is needed that large page be enabled (see config.h)
# Example run of fuse:
# $(OUTDIR)/myFuse -c $(FUSEDIR)/ref/config.conf -f $(FUSEDIR)/ref/text.txt \
# 	-m $(FUSEDIR)/mount -s $(FUSEDIR)/ref -l $(OUTDIR)/fuse.log
fuse: all
	$(Q)make -C $(FUSEDIR) all


# Change the target to something else if not running on linux
# Simply running make from $(IOZONEDIR) list the architectures that iozone
# supports
iozone:
	@make -C $(IOZONEDIR) linux

# Optional performance test that uses fuse and iozone to run performance
# benchmarks over FTL
perftest: fuse iozone
	@echo "#########################################################"
	@echo "Running fuse in background process and iozone over it"
	@echo "Config file $(FUSEDIR)/ref/config.conf"
	@echo "Output Log File $(OUTDIR)/fuse.log & $(OUTDIR)/fuse_debug.log"
	@echo "Temporary File $(FUSEDIR)/fuse/ref/text.txt"
	@echo "Block Size 4k, File Max Size 1MB"
	@echo "#########################################################"
	$(Q)mkdir -p $(FUSEDIR)/mount;
	$(vecho) echo "Warning:Cleaning up the mount directory"
	-$(Q)rm -r $(FUSEDIR)/mount/*;
	$(Q)$(OUTDIR)/myFuse -c $(FUSEDIR)/ref/config.conf 	\
			     -f $(FUSEDIR)/ref/text.txt 	\
 			     -m $(FUSEDIR)/mount		\
			     -s $(FUSEDIR)/ref 			\
			     -l $(OUTDIR)/fuse.log 		\
			     -d 0 > $(OUTDIR)/fuse_debug.log &
	$(vecho) "Waiting for fuse"
	$(Q)sleep 2
	@# On any error, cleanup
	@bash -c "								\
	trap 'trap - SIGINT SIGTERM ERR;					\
	killall myFuse; fusermount -u $(FUSEDIR)/mount;				\
	rm -f $(FUSEDIR)/ref/text.txt; touch $(FUSEDIR)/ref/text.txt;		\
	exit 1'									\
	SIGINT SIGTERM ERR;							\
	$(IOZONEDIR)/iozone -w -a -s 1024 -r 4 -f $(FUSEDIR)/mount/text.txt;	\
	fusermount -u $(FUSEDIR)/mount;						\
	"

clean:
	$(Q)rm -rf $(BUILDDIR)/*
	$(Q)rm -rf $(OUTDIR)/*.log
	$(Q)rm -rf $(OUTDIR)/*.png
	$(Q)rm -rf $(OUTDIR)/*.dat
	$(Q)make -C $(FUSEDIR) clean
	$(Q)rm -rf *.tar
	$(Q)rm -rf *.tar.gz

veryclean: clean
	$(Q)make -C $(IOZONEDIR) clean
	-$(Q)killall myFuse
	-$(Q)fusermount -u $(FUSEDIR)/mount
	$(Q)rm -rf $(FUSEDIR)/mount

################################################################################
