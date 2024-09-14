/*
 * @file 746FTL.cpp
 * @brief This file is where that part of FTL code resides whos sole purpose
 * is to communicate with the FlashSim.
 *
 * This file is a seperate executable (The child process) and the FLashSim
 * is the parent. This seperation is required to protect the test code
 * (FlashSim)
 *
 * @author Saksham Jain (sakshamj)
 * @bug No known bugs
 */

#include "746FTL.h"

#include <fcntl.h>
#include <malloc.h>
#include <poll.h>
#include <signal.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include "common.h"
#include "config.h"
#include "memcheck.h"
#include "myFTL.h"

#if (CONFIG_TWOPROC == 1)

FTLBase<TEST_PAGE_TYPE> *ftl;

#if MEMCHECK_ENABLED

static void *stack_start;
static void *stack_end;

#if (STACK_CHECK == STACK_CHECK_CANARY)
static void *get_stack_end(void);
#endif /* STACK_CHECK */

size_t get_cur_stack_size(void);

#endif /* MEMCHECK_ENABLED */

/*
 * Note: pipes for communication have been opened by parent (FlashSim) before
 * forking
 */

/*
 * SendParentBytes - Sends the parent process bytes over pipe (IPC)
 *
 * If failure here, probably can't continue, so terminate
 *
 * buf - Buffer which hold's data to send
 * size - Size of data in buffer in bytes
 */
static void SendParentBytes(void *buf, size_t size) {
  /* Parent's rx is child's tx */
  ssize_t ret = write(Common.pipefd[PIPE_TX_END], buf, size);
  if (ret < 0) {
    perror("FATAL: Couldn't send child data");
    assert(0 && "Failure in writing to child's rx pipe");
  }
}

/*
 * RecvParentBytes - Recevives from parent process bytes over pipe (IPC)
 *
 * If failure here, probably can't continue, so terminate
 *
 * buf - Buffer which hold's data to send
 * size_t - Size of data that buffer can hold
 *
 * Returns the actual bytes reveived
 */
static size_t RecvParentBytes(void *buf, size_t size) {
  ssize_t ret;

  /* Parent's rx is child's tx - Should block if no data */
  ret = read(Common.pipefd[PIPE_RX_END], buf, size);
  if (ret < 0) {
    perror("FATAL: Couldn't recv parent data");
    assert(0 && "Failure in reading from parent's tx pipe");
  }

  return ret;
}

/*
 * IsRecvMsgPending - Indicates if any read messages are pending
 *
 * Returns 1 if pending else 0
 *
 * TODO: Remove if not used.
 */
int IsRecvMsgPending(void) {
  IPC_Format msg;
  struct pollfd pfd;
  int ret;

  pfd.fd = Common.pipefd[PIPE_RX_END];
  pfd.events = POLLIN;

  do {
    ret = poll(&pfd, 1, 0);

  } while (ret < 0 && errno == EINTR);

  if (ret < 0) {
    perror("FATAL: Poll failed");
    assert(0 && "Poll failed on pipe read");
  }

  if (ret == 0) /* empty */
    return 0;
  else
    return 1;
}

/*
 * RecvMsgFromFlashSim - Receives message from the ftl (child)
 *
 * should_block - Block or nonblock
 * rx_msg - Pointer to where to store the received message.
 * Returns empty message if no mesaage is in pipe
 */
static void RecvMsgFromFlashSim(IPC_Format *rx_msg, int should_block) {
  int ret;
  size_t size;
  int timeout;
  struct pollfd pfd;

  pfd.fd = Common.pipefd[PIPE_RX_END];
  pfd.events = POLLIN;

  if (should_block) /* Infinite timeout */
    timeout = -1;
  else
    /* Non blocking */
    timeout = 0;

  do {
    ret = poll(&pfd, 1, timeout);

  } while (ret < 0 && errno == EINTR);

  if (ret < 0) {
    perror("FATAL: Poll failed");
    assert(0 && "Poll failed on pipe read");
  }

  if (ret == 0) {
    /* No data to read - Return empty message */
    assert(should_block == 0 && "No data even on blocking poll");
    rx_msg->type_ = MSG_EMPTY;
    return;

  } else {
    /* Copy data */
    size = RecvParentBytes((void *)rx_msg, sizeof(*rx_msg));
    if (size != sizeof(*rx_msg)) {
      if (size == 0) {
        /*
         * Note: We might get 0 bytes from parent,
         * if parent to closes the pipe (mainly because
         * it gets terminated). Under normal circumstance,
         * parent waits for child to die before it
         * terminates.
         */
        assert(0 && "Parent process shouldn't have died");
      }
      assert(0 && "Unknown message size");
    }

    /* Message should be from parent */
    assert(rx_msg->owner_ == OWNER_FLASHSIM && "Unknown owner_");
    return;
  }
}

/*
 * SendMsgToFlashSim - Send messages to the flashsim (parent)
 *
 * tx_msg - Pointer to the message.
 */
static void SendMsgToFlashSim(IPC_Format *tx_msg) {
  /* The message to be transmitted must come from flashsim here */
  assert((tx_msg->owner_ == OWNER_FTL) && "Unknown owner_");

  SendParentBytes((void *)tx_msg, sizeof(*tx_msg));
}

/*
 * ProcessRequestFromFlashSim - Process and replies to the request pending in
 * pipe from Flashsim
 *
 * ftl - MyFTL object used to fulfill requests - typecast as FTLBase
 * ecb - ExecCallBack object passed to MyFTL functions
 * pending_recv_msg - Any pending previous requests? (Optional)
 * should_block - Should read block if no messages in read pipe?
 *
 * This function ends when either pipe is empty (and call in nonblocking),
 * or pipe now contains a request from Flashsim
 */
static void ProcessRequestFromFlashSim(FTLBase<TEST_PAGE_TYPE> *ftl,
                                       FTLExecCallBack &ecb,
                                       IPC_Format *pending_recv_msg,
                                       int should_block) {
  IPC_Format recv_msg, send_msg;
  size_t lba;
  std::pair<ExecState, Address> read_write_resp;
  ExecState trim_resp;

  send_msg.owner_ = OWNER_FTL;

  if (pending_recv_msg == NULL)
    RecvMsgFromFlashSim(&recv_msg, should_block);
  else
    recv_msg = *pending_recv_msg;

  switch (recv_msg.type_) {
    /* Flashsim asks for FTL services */
    case MSG_FTL_INSTR_READ:

      lba = recv_msg.lba_;
      send_msg.type_ = MSG_FTL_READ_RESP;
      read_write_resp = ftl->ReadTranslate(lba, ecb);
      send_msg.ftl_resp_addr_ = read_write_resp.second;
      send_msg.ftl_resp_execstate_ = read_write_resp.first;

      break;

    case MSG_FTL_INSTR_WRITE:

      lba = recv_msg.lba_;
      send_msg.type_ = MSG_FTL_WRITE_RESP;
      read_write_resp = ftl->WriteTranslate(lba, ecb);
      send_msg.ftl_resp_addr_ = read_write_resp.second;
      send_msg.ftl_resp_execstate_ = read_write_resp.first;

      break;

    case MSG_FTL_INSTR_TRIM:

      lba = recv_msg.lba_;
      send_msg.type_ = MSG_FTL_TRIM_RESP;
      trim_resp = ftl->Trim(lba, ecb);
      send_msg.ftl_resp_execstate_ = trim_resp;

      break;

    case MSG_FTL_STACK_SIZE_REQ:

      send_msg.type_ = MSG_FTL_STACK_SIZE_RESP;
#if MEMCHECK_ENABLED
      send_msg.child_stack_size_ = get_cur_stack_size();
#else  /* MEMCHECK_ENABLED */
      send_msg.child_stack_size_ = 0;
#endif /* MEMCEHCK_ENABLED */
      break;

    default:
      assert(0 && "Unknown message from Flashsim");
  } /* Switch */

  /* Send the response now */
  SendMsgToFlashSim(&send_msg);
}

/*
 * SendReqToFlashSim - Sends request to the flashsim (parent)
 *
 * tx_msg - The message (request) to send to flashsim
 * rx_msg - The response of the message
 *
 * Returns another msg that is the response of the msg from flashsim
 */
void SendReqToFlashSim(IPC_Format *tx_msg, IPC_Format *rx_msg) {
  /* Expected messaage type_ of the response to msg transmitted */
  enum message_type_t exp_rx_typ;

  switch (tx_msg->type_) {
    /* Ask for configuration */
    case MSG_CONF_REQ_SSD_SIZE:

      exp_rx_typ = MSG_CONF_RES_SSD_SIZE;
      break;

    case MSG_CONF_REQ_PACKAGE_SIZE:

      exp_rx_typ = MSG_CONF_RES_PACKAGE_SIZE;
      break;

    case MSG_CONF_REQ_DIE_SIZE:

      exp_rx_typ = MSG_CONF_RES_DIE_SIZE;
      break;

    case MSG_CONF_REQ_PLANE_SIZE:

      exp_rx_typ = MSG_CONF_RES_PLANE_SIZE;
      break;

    case MSG_CONF_REQ_BLOCK_SIZE:

      exp_rx_typ = MSG_CONF_RES_BLOCK_SIZE;
      break;

    case MSG_CONF_REQ_BLOCK_ERASES:

      exp_rx_typ = MSG_CONF_RES_BLOCK_ERASES;
      break;

    case MSG_CONF_REQ_OVERPROVISIONING:

      exp_rx_typ = MSG_CONF_RES_OVERPROVISIONING;
      break;

    case MSG_CONF_REQ_GCPOLICY:

      exp_rx_typ = MSG_CONF_RES_GCPOLICY;
      break;

    /* Ask for any of the flashsim services - Empty message expected */
    case MSG_SIM_REQ_READ:

      exp_rx_typ = MSG_EMPTY;
      break;

    case MSG_SIM_REQ_WRITE:

      exp_rx_typ = MSG_EMPTY;
      break;

    case MSG_SIM_REQ_ERASE:

      exp_rx_typ = MSG_EMPTY;
      break;

    default:
      assert(0 && "Unknown msg typ");
  }

  /* Send the child request */
  SendMsgToFlashSim(tx_msg);

  /* Now wait for response - Blocking wait */
  RecvMsgFromFlashSim(rx_msg, 1);

  if (rx_msg->type_ != exp_rx_typ) assert(0 && "Unknown response received");

  /* Received message is the response */
}

#if MALLOC_TRACE_ENABLED

int malloc_trace_fd;
void *(*old_malloc_hook)(size_t size, const void *caller);
void (*old_free_hook)(void *ptr, const void *caller);
void *(*old_realloc_hook)(void *ptr, size_t size, const void *caller);
void *(*old_memalign_hook)(size_t alignment, size_t size, const void *caller);

static void *my_malloc_hook(size_t size, const void *caller);
static void my_free_hook(void *ptr, const void *caller);
static void *my_realloc_hook(void *ptr, size_t size, const void *caller);
static void *my_memalign_hook(size_t alignment, size_t size,
                              const void *caller);

static void restore_old_hooks() {
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  __realloc_hook = old_realloc_hook;
  __memalign_hook = old_memalign_hook;
}
static void set_new_hooks() {
  __malloc_hook = my_malloc_hook;
  __free_hook = my_free_hook;
  __realloc_hook = my_realloc_hook;
  __memalign_hook = my_memalign_hook;
}

static void *my_malloc_hook(size_t size, const void *caller) {
  struct mallinfo mi;
  void *result;
  char buf[100];
  ssize_t buf_size, wsize;

  (void)caller;

  /* Restore all old hooks */
  restore_old_hooks();

  result = malloc(size);
  mi = mallinfo();

  buf_size = sprintf(buf, "%d\n", mi.uordblks);
  wsize = write(malloc_trace_fd, buf, buf_size);
  assert(wsize >= 0);

  /* Restore our own hooks */
  set_new_hooks();

  return result;
}

static void my_free_hook(void *ptr, const void *caller) {
  struct mallinfo mi;
  char buf[100];
  ssize_t buf_size, wsize;

  (void)caller;

  /* Restore all old hooks */
  restore_old_hooks();

  free(ptr);
  mi = mallinfo();

  buf_size = sprintf(buf, "%d\n", mi.uordblks);
  wsize = write(malloc_trace_fd, buf, buf_size);
  assert(wsize >= 0);

  /* Restore our own hooks */
  set_new_hooks();
}

static void *my_realloc_hook(void *ptr, size_t size, const void *caller) {
  struct mallinfo mi;
  char buf[100];
  ssize_t buf_size, wsize;
  void *result;

  (void)caller;

  /* Restore all old hooks */
  restore_old_hooks();

  result = realloc(ptr, size);
  mi = mallinfo();

  buf_size = sprintf(buf, "%d\n", mi.uordblks);
  wsize = write(malloc_trace_fd, buf, buf_size);
  assert(wsize >= 0);

  /* Restore our own hooks */
  set_new_hooks();

  return result;
}

static void *my_memalign_hook(size_t alignment, size_t size,
                              const void *caller) {
  struct mallinfo mi;
  char buf[100];
  ssize_t buf_size, wsize;
  void *result;

  (void)caller;

  /* Restore all old hooks */
  restore_old_hooks();

  result = memalign(alignment, size);
  mi = mallinfo();

  buf_size = sprintf(buf, "%d\n", mi.uordblks);
  wsize = write(malloc_trace_fd, buf, buf_size);
  assert(wsize >= 0);

  /* Restore our own hooks */
  set_new_hooks();

  return result;
}

/* Calloc calls malloc internally so is not needed to be trapped */

static int init_malloc_trace() {
  malloc_trace_fd =
      open(MALLOC_TRACE_FILE, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);
  if (malloc_trace_fd < 0) return malloc_trace_fd;

  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  old_memalign_hook = __memalign_hook;

  set_new_hooks();

  return 0;
}

#endif /* MALLOC_TRACE_ENABLED */

#if MEMCHECK_ENABLED

/*
 * Returns the current stack size
 * As side effect, sets the global stack_end
 */
size_t get_cur_stack_size(void) {
#if (STACK_CHECK == STACK_CHECK_EXPANSION)
  register void *sp asm("sp");
  stack_end = sp;
#elif (STACK_CHECK == STACK_CHECK_CANARY)
  stack_end = get_stack_end();
#else
#error "Stack check wrongly configured"
#endif

  return (uintptr_t)stack_start - (uintptr_t)stack_end;
}

void term_handler(int signal) {
  assert(signal == SIGUSR1);

  /* TODO: Commenting this out will allow myftl destructor to be called */
  /*
   * if (ftl != NULL)
   *	delete ftl;
   */

#if PRINT_STATS_ENABLE
  struct rusage usage;
  int ret = getrusage(RUSAGE_SELF, &usage);
  get_cur_stack_size();

  assert(ret >= 0);

  printf("############## CHILD SIDE STATS START #####################\n");
  printf("Child's user time %zu.%zu sec\n", usage.ru_utime.tv_sec,
         usage.ru_utime.tv_usec / 1000);
  printf("Child's system time %zu.%zu sec\n", usage.ru_stime.tv_sec,
         usage.ru_stime.tv_usec / 1000);

  struct mallinfo mi;

  mi = mallinfo();

  printf("Detailed heap info:\n");
  printf("Total non-mmapped bytes (arena):       %d\n", mi.arena);
  printf("# of free chunks (ordblks):            %d\n", mi.ordblks);
  printf("# of free fastbin blocks (smblks):     %d\n", mi.smblks);
  printf("# of mapped regions (hblks):           %d\n", mi.hblks);
  printf("Bytes in mapped regions (hblkhd):      %d\n", mi.hblkhd);
  printf("Max. total allocated space (usmblks):  %d\n", mi.usmblks);
  printf("Free bytes held in fastbins (fsmblks): %d\n", mi.fsmblks);
  printf("Total allocated space (uordblks):      %d\n", mi.uordblks);
  printf("Total free space (fordblks):           %d\n", mi.fordblks);
  printf("Topmost releasable block (keepcost):   %d\n", mi.keepcost);

  printf("Detailed stack info:\n");
  printf("END:Stack used %zu\n", (uintptr_t)stack_start - (uintptr_t)stack_end);
  printf("END:Stack start %zu, Stack end %zu\n", (uintptr_t)stack_start,
         (uintptr_t)stack_end);

  printf("############## CHILD SIDE STATS END #######################\n");

  fflush(stdout);
#endif /* PRINT_STATS_ENABLE */

#if MALLOC_TRACE_ENABLED
  close(malloc_trace_fd);
#endif
  /* Simply exit - No need to cleanup */
  exit(0);
}

#if (STACK_CHECK == STACK_CHECK_EXPANSION)

/* Alternate stack for signal handler */
char altstack[2 * MINSIGSTKSZ];

/**
 * @brief signal handler which is triggered when stack expands
 * @param signal Signal which was received
 * XXX: This will trap any sigsegv due to student's code error. Need to
 * distinguish.
 */
void stack_handler(int signal) {
  int ret;
  struct rlimit stack_limit;

  assert(signal == SIGSEGV);

  fprintf(stderr,
          "ATTENTION: Sigsegv received."
          "Assuming reason is stack expansion\n");
  ret = getrlimit(RLIMIT_STACK, &stack_limit);
  if (ret < 0) assert(0);

  stack_limit.rlim_cur += PAGE_SIZE;

  ret = setrlimit(RLIMIT_STACK, &stack_limit);
  if (ret < 0) assert(0);
}

#elif (STACK_CHECK == STACK_CHECK_CANARY)

/**
 * @brief Intialize stack by placing canaries on the stack at some offset
 * Corruption of these canaries indicate stack size (roughly)
 */
static void init_child_stack(void) {
  register void *sp asm("sp");
  int i, j;
  uintptr_t cur;
  int *canary_p;

  /* Rounddown */
  cur = (uintptr_t)sp & (~STACK_CANARY_OFFSET_MASK);

  /* A bit extra then needed but it's okay */
  stack_start = (void *)cur;

  for (i = 1; i < STACK_MIN_OFFSET; i++) cur = cur - STACK_CANARY_OFFSET;

  for (i = STACK_MIN_OFFSET; i < STACK_MAX_OFFSET; i++) {
    cur = cur - STACK_CANARY_OFFSET;

    for (j = 1; j <= STACK_CANARY_BLOCK; j++) {
      canary_p = (int *)(cur - sizeof(int) * j);
      *canary_p = STACK_CANARY;
    }
  }
}

/**
 * @brief Roughly calculates the end of stack based on corruption of canary
 * @return stack end
 * TODO: Can do binary search for faster result
 */
static void *get_stack_end(void) {
  register void *sp asm("sp");
  int i, j;
  uintptr_t cur;
  unsigned int *canary_p;

  /* Rounddown */
  cur = (uintptr_t)sp & (~STACK_CANARY_OFFSET_MASK);
  for (i = 0; i < STACK_MAX_OFFSET; i++) {
    int corrupted = 0;

    for (j = 1; j <= STACK_CANARY_BLOCK; j++) {
      canary_p = (unsigned int *)(cur - sizeof(unsigned int) * j);

      if (*canary_p != STACK_CANARY) {
        corrupted = 1;
        break;
      }
    }

    if (corrupted == 0) return (void *)cur;

    cur = cur - STACK_CANARY_OFFSET;
  }

  assert(0 && "Stack grown more than expected");
}

#else
#error "Stack check wrongly configured"
#endif /* STACK_CHECK */

/**
 * @brief Intialize memcheck on child side
 * @return 0 on success, < 0 on error
 * TODO: Is this file the right place for this?
 */
static int init_memcheck_child(void) {
  struct sigaction new_action;
  sigset_t set;
  int ret;

#if (STACK_CHECK == STACK_CHECK_CANARY)
  init_child_stack();
#elif (STACK_CHECK == STACK_CHECK_EXPANSION)

  stack_t ss;

  register void *sp asm("sp");
  stack_start = sp;

  /*
   * Alternate stack for the exception handler
   * Inside exception handler, stack will be invalid as it is called
   * for statck expansion
   */
  ss.ss_sp = altstack;
  ss.ss_size = sizeof(altstack);
  ss.ss_flags = 0;

  /* Set alternate stack */
  ret = sigaltstack(&ss, NULL);
  if (ret < 0) return ret;

  memset(&new_action, 0, sizeof(new_action));
  new_action.sa_handler = stack_handler;
  new_action.sa_flags = SA_ONSTACK;
  sigemptyset(&new_action.sa_mask);

  ret = sigaction(SIGSEGV, &new_action, NULL);
  if (ret < 0) return ret;

#endif /* STACK_CHECK */

  memset(&new_action, 0, sizeof(new_action));
  new_action.sa_handler = term_handler;
  new_action.sa_flags = 0;
  sigemptyset(&new_action.sa_mask);

  ret = sigaction(SIGUSR1, &new_action, NULL);
  if (ret < 0) return ret;

  /* Unblock signal */
  ret = sigemptyset(&set);
  if (ret < 0) return ret;

#if (STACK_CHECK == STACK_CHECK_EXPANSION)
  ret = sigaddset(&set, SIGSEGV);
  if (ret < 0) return ret;
#endif

  ret = sigaddset(&set, SIGUSR1);
  if (ret < 0) return ret;

  ret = sigprocmask(SIG_UNBLOCK, &set, NULL);
  if (ret < 0) return ret;

  /*
   * Triming the top of heap to give better accuracy of heap usage
   * Increasing this value will make child run faster as less system calls
   * at cost of higher heap usage
   */
  ret = mallopt(M_TRIM_THRESHOLD, 1);
  if (ret == 0) return -1;

  /*
   * TODO: Block mmap calls
   * If we intend to disallow mmap calls, this will ensure that
   * malloc doesn't call mmap - This value corresponds to
   * 16*1024*1024*sizeof(long)
   */
  ret = mallopt(M_MMAP_THRESHOLD, MMAP_THRESHOLD_MAX);
  if (ret == 0) return -1;

  /*
   * Malloc generally reserves some memory when it makes sbrk()
   * sort of overprovisioning (defualt is 128 * 1024)
   * So lets make it page size (can't make malloc do better than that)
   * This too makes the system calls increase in number
   */
  ret = mallopt(M_TOP_PAD, 1);
  if (ret == 0) return -1;

  ret = mallopt(M_MXFAST, 0);
  if (ret == 0) return -1;

  return 0;
}
#endif /* MEMCHECK_ENABLED */

/*
 * This is where code starts after child is forked and execve'd
 */
int main(int argc, char **argv) {
  IPC_Format recv_msg, tx_msg;

#if MEMCHECK_ENABLED || MALLOC_TRACE_ENABLED
  int ret;
#endif

  /* Fetch arguments */
  if ((argc < CHILD_PIPE_RX_FD_ARGV_OFF + 1))
    assert(0 && "Too few arguments");

  /* Child's rx pipefd */
  sscanf(argv[CHILD_PIPE_RX_FD_ARGV_OFF], "%d", &Common.pipefd[PIPE_RX_END]);

  /* Child's tx pipefd */
  sscanf(argv[CHILD_PIPE_TX_FD_ARGV_OFF], "%d", &Common.pipefd[PIPE_TX_END]);

  /* We are the child */
  Common.child_pid = 0;

#if MEMCHECK_ENABLED
  /* Init memcheck */
  ret = init_memcheck_child();
  if (ret < 0) return ret;
#endif

#if MALLOC_TRACE_ENABLED
  /* This needs to be called before first malloc is issued */
  ret = init_malloc_trace();
  if (ret < 0) return ret;
#endif

  /* Creat FTL side of conf */
  FTLConf conf;

  /*
   * Create ExecCallBack object for allowing myFTL to request FlashSim
   * services
   */
  FTLExecCallBack ecb;

  /*
   * Send a empty message to parent so that it knows child is up and
   * ready to work
   */
  tx_msg.type_ = MSG_FTL_WAKEUP;
  tx_msg.owner_ = OWNER_FTL;
  SendMsgToFlashSim(&tx_msg);

  /*
   * FIXME: This is a hack. Before calling constructor of ftl,
   * which might then send requests to FlashSim, we wait for a request
   * to arrive from FlashSim. This allows us to maintain state machine
   * I know I haven't clearly stated the issue, but think about it.
   */
  RecvMsgFromFlashSim(&recv_msg, 1);

  /* Create an object of myFTL typecast as FTLBase */
  ftl = CreateMyFTL(&conf);

  /* FTL has only one job - Process the requests coming from FlashSim */
  ProcessRequestFromFlashSim(ftl, ecb, &recv_msg, 1);

  while (1) {
    ProcessRequestFromFlashSim(ftl, ecb, NULL, 1);
  }

  /* TODO: When do we delete FTL? */
}

#endif /* CONFIG_TWOPROC */
