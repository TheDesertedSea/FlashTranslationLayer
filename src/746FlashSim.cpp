/*
 * 746FlashSim.cpp - Flash translation layer simulator for CMU 18-/15-746
 *
 * This file contains the implementation of CMU's 18746/15746: Storage Systems
 * course project framework
 */
#include "746FlashSim.h"

#include <signal.h>
#include <sys/resource.h>

#include "common.h"
#include "config.h"
#include "memcheck.h"

bool is_inf = 1;

#if ENABLE_TRANS_TRACING
/* File for keeping a track on ftl transcations */
FILE *trans_trace_fp;
#endif

#if (CONFIG_TWOPROC == 1)

/*
 * init_flashsim - Initialize flashsim framework
 *
 * The objective is to fork() a child, and open two pipes between the parent
 * and child to enable IPC.
 * This allows seperation of child and parent memory, while enabling
 * communication between them.
 *
 * This function should be the very first function to be called from main
 *
 * Any failure here indicates inability to continue. So hence printing to
 * stderr and then terminating
 */
void init_flashsim(void) {
  int ret;

  /* Going to use two pipes
   * First pipe - Parent write, child reads
   * Second Pipe - Parent read, child writes
   */
  int parent_write_pipefd[2], parent_read_pipefd[2];

  /* TODO: Make a macro for throwing these errors/exceptions */

  /* Open pipes */
  ret = pipe(parent_write_pipefd);
  if (ret < 0) {
    perror("FATAL: Couldn't open first pipe");
    assert(0 && "Failure in opening first pipe");
  }

  ret = pipe(parent_read_pipefd);
  if (ret < 0) {
    perror("FATAL: Couldn't open second pipe");
    assert(0 && "Failure in opening second pipe");
  }

  /* Fork child */
  Common.child_pid = fork();
  if (Common.child_pid < 0) {
    perror("FATAL: Couldn't fork");
    assert(0 && "Failure in forking child");

  } else if (Common.child_pid == 0) {
    char *newargv[] = {CHILD_EXE_PATH, NULL, NULL, NULL};
    char *newenviron[] = {NULL};

    char rx_pipe_fd[MAX_PIPEFD_STR_LEN];
    char tx_pipe_fd[MAX_PIPEFD_STR_LEN];

#if MEMCHECK_ENABLED
#if (STACK_CHECK == STACK_CHECK_EXPANSION)
    struct rlimit stack_limit;
#endif
#endif
    /*
     * Close pipes not used by child
     * i.e. where parent reads/write
     */
    ret = close(parent_write_pipefd[PIPE_TX_END]);
    if (ret < 0) {
      perror("FATAL: Couldn't close pipe");
      assert(0 && "Failure in closing pipe");
    }

    /* Close old rx pipe in child */
    ret = close(parent_read_pipefd[PIPE_RX_END]);
    if (ret < 0) {
      perror("FATAL: Couldn't close pipe");
      assert(0 && "Failure in closing pipe");
    }

    /*
     * Store the pipes in use by child appropriately
     * Pipe where parent write, we use rx end ther
     * and tx end pn pipe where parent writes
     */
    snprintf(rx_pipe_fd, sizeof(rx_pipe_fd), "%02d",
             parent_write_pipefd[PIPE_RX_END]);
    snprintf(tx_pipe_fd, sizeof(tx_pipe_fd), "%02d",
             parent_read_pipefd[PIPE_TX_END]);

    /*
     * Pass information to child in argv as calling execve
     * so all current memory of child will be discarded
     */
    newargv[CHILD_PIPE_RX_FD_ARGV_OFF] = rx_pipe_fd;
    newargv[CHILD_PIPE_TX_FD_ARGV_OFF] = tx_pipe_fd;

#if MEMCHECK_ENABLED

#if (STACK_CHECK == STACK_CHECK_EXPANSION)
    /*
     * Now try to put limit on the stack size.
     * By default gcc gives 2MB stack space.
     * XXX: Is this info in the ELF? If yes, find compilation flag for g++
     * Now lets place a limit on the stack to be 8KB initially,
     * we will then increase this limit dynamically
     */

    stack_limit.rlim_cur = CHILD_INIT_STACK_SIZE;
    stack_limit.rlim_max = -1;

    ret = setrlimit(RLIMIT_STACK, &stack_limit);
    if (ret < 0) assert(0);

#endif /* STACK_CHECK */

#endif /* MEMCHECK_ENABLED */

    /* Load the child now */
    ret = execve(CHILD_EXE_PATH, newargv, newenviron);
    if (ret < 0) {
      perror("FATAL: Couldn't execve");
      assert(0 && "Failure in running child");
    }

  } else {
    struct pollfd pollfd;
    int ret;

    /*
     * Close pipes not used by parent
     */
    ret = close(parent_write_pipefd[PIPE_RX_END]);
    if (ret < 0) {
      perror("FATAL: Couldn't close pipe");
      assert(0 && "Failure in closing pipe");
    }

    /* Close old rx pipe in child */
    ret = close(parent_read_pipefd[PIPE_TX_END]);
    if (ret < 0) {
      perror("FATAL: Couldn't close pipe");
      assert(0 && "Failure in closing pipe");
    }

    /* Store the pipes in use by parent appropriately */
    Common.pipefd[PIPE_RX_END] = parent_read_pipefd[PIPE_RX_END];
    Common.pipefd[PIPE_TX_END] = parent_write_pipefd[PIPE_TX_END];

    /* First wait for child to be up - Then init memcheck */

    pollfd.fd = Common.pipefd[PIPE_RX_END];
    pollfd.events = POLLIN;

    do {
      ret = poll(&pollfd, 1, -1);

    } while (ret < 0 && errno == EINTR);

#if MEMCHECK_ENABLED
    ret = init_memcheck_parent(Common.child_pid);
    if (ret < 0) assert(0 && "Memcheck not running");
#endif
  }

  /* Parent - Everything is done, now resume execution */
}

/* To be called when wrapping up and exiting */
void deinit_flashsim(void) {
  int ret;

#if MEMCHECK_ENABLED
  /* Deinitialize memcheck before killing child to update one last time */
  ret = deinit_memcheck_parent();
  if (ret < 0) assert(0 && "Memcheck not running");
#endif

#if PRINT_STATS_ENABLE
  struct rusage usage;
  ret = getrusage(RUSAGE_SELF, &usage);
  assert(ret >= 0);

  printf("############## PARENT SIDE STATS START ####################\n");
  printf("Parents's user time %zu.%zu sec\n", usage.ru_utime.tv_sec,
         usage.ru_utime.tv_usec / 1000);
  printf("Parents's system time %zu.%zu sec\n", usage.ru_stime.tv_sec,
         usage.ru_stime.tv_usec / 1000);
  printf("############## PARENT SIDE STATS END  #####################\n");
#endif

  /* Before waking child (by giving it a signal), flush all outputs */
  fflush(stdout);
  fflush(stderr);

#if MEMCHECK_ENABLED

  /* Kill the child  to free up space */
  ret = kill(Common.child_pid, SIGUSR1);
#else
  ret = kill(Common.child_pid, SIGKILL);
#endif
  if (ret < 0) {
    perror("Couldn't kill child. Resources might not be freed");
    assert(0 && "Child still running");
  }
  /* Can close pipes also */
}

FTLBase<TEST_PAGE_TYPE> *FlashSimTest::CreateFlashSimFTL(
    FlashSimTest *fs_test) {
  return new FlashSimFTL<TEST_PAGE_TYPE>(fs_test);
}

#else  /* CONFIG_TWOPROC */
void init_flashsim(void) {}

void deinit_flashsim(void) {}
#endif /* CONFIG_TWOPROC */