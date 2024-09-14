/*
 * @file memcheck.cpp
 * @brief This file keeps track of memory usage by the child by periodically
 * checking the proc maps
 *
 * @author Saksham Jain (sakshamj)
 * @author (Tweaked) Ankush Jain (ankushj)
 * @bug No known bugs
 */
#include "memcheck.h"

#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "common.h"
#include "config.h"

/*
 * Used to temporarily disable memusage update
 * Synchronization method as dealing with signal handlers
 */
int disable_update;

/**
 * @brief Global data structure to keep data about all memory usage of child
 */
struct memcheck_glb_t {
  /* Child's pid */
  pid_t pid;

  void *stack_start_addr;

  /* Sizes at last checked time - Can be used for debug*/
  size_t cur_stack_size;
  size_t cur_heap_size;
  size_t cur_annony_size; /* Annoynomous mapping in proc maps */
  size_t cur_data_size;   /* Data section */
  size_t cur_misc_size;   /* Misc like libraries data */

  /* All time high usage */
  size_t max_stack_size;
  size_t max_heap_size;
  size_t max_annony_size;
  size_t max_data_size;
  size_t max_misc_size;

  /* Max usage != sumof maxes */
  size_t max_usage;

  /* Intial usage */
  size_t init_usage;

  /* Number of times updated */
  int update_count;
};

/* All values initialized to zero */
static struct memcheck_glb_t memcheck_glb;

#if PRINT_STATS_ENABLE

/**
 * @brief Prints the memory usage
 * @param stack_size Pass as input the stack size if using another method to
 * calculate stack size else 0
 */
void print_memusage(size_t stack_size) {
  printf("############## MEMCHECK SIDE STATS START ##################\n");
  printf("############## This is used for grading ###################\n");
  if (stack_size == 0) {
    printf("MEMCHECK:Cur Stack size: %zu\n", memcheck_glb.cur_stack_size);
    printf("MEMCHECK: Stack start: %zu\n",
           (uintptr_t)memcheck_glb.stack_start_addr);
  }

  printf("MEMCHECK:Cur heap size: %zu\n", memcheck_glb.cur_heap_size);
  printf("MEMCHECK:Cur data size: %zu\n", memcheck_glb.cur_data_size);

  if (stack_size == 0) {
    printf("MEMCHECK:Max Stack size: %zu\n", memcheck_glb.max_stack_size);
  } else {
    printf("MEMCHECK:Max Stack size: %zu\n", stack_size);
  }

  printf("MEMCHECK:Max heap size: %zu\n", memcheck_glb.max_heap_size);
  printf("MEMCHECK:Max data size: %zu\n", memcheck_glb.max_data_size);

  printf("Additional (Non-essential) stats:\n");
  printf("MEMCHECK:Cur annony size: %zu\n", memcheck_glb.cur_annony_size);
  printf("MEMCHECK:Cur misc size: %zu\n", memcheck_glb.cur_misc_size);
  printf("MEMCHECK:Max annony size: %zu\n", memcheck_glb.max_annony_size);
  printf("MEMCHECK:Max misc size: %zu\n", memcheck_glb.max_misc_size);
  printf("MEMCHECK:Max usage: %zu\n", memcheck_glb.max_usage);
  printf("MEMCHECK:Initial usage %zu\n", memcheck_glb.init_usage);
  printf("MEMCHECK:Update count %d\n", memcheck_glb.update_count);
  printf("MEMCHECK:PID is %d\n", memcheck_glb.pid);

  printf("############## MEMCHECK SIDE STATS END ####################\n");
  fflush(stdout);
}

#endif /* PRINT_STATS_ENABLE */

/*
 * Returns total memory used by child
 * child_stack_size_ - Can be 0 in which case we use the stack size
 * mentioned by the procmap - Might be larger than expected
 */
size_t get_child_total_mem(size_t child_stack_size) {
  /* Can't call from here as race condition with the alarm signal */
  /* update_memusage(); */

#if PRINT_STATS_ENABLE
  print_memusage(child_stack_size_);
#endif
  if (child_stack_size == 0) {
    return memcheck_glb.max_heap_size + memcheck_glb.max_data_size +
           memcheck_glb.max_stack_size;
  } else {
    return memcheck_glb.max_heap_size + memcheck_glb.max_data_size +
           child_stack_size;
  }
}

void get_line(char **buf_p, char *line) {
  char *line_end;
  char *line_start = *buf_p;
  if (line_start == NULL) return;

  line_end = strchr(line_start, '\n');
  if (line_end == NULL) {
    *buf_p = NULL;
    return;
  }

  memcpy(line, line_start, (uintptr_t)line_end - (uintptr_t)line_start);
  line[(uintptr_t)line_end - (uintptr_t)line_start] = '\0';
  *buf_p = line_end + 1;
}

/**
 * @brief Read proc map and updates usage
 * @return 0 on success, < 0 on error
 */
int update_memusage(void) {
  int fd, ret;
  char filename[30];
  char buf[MAX_FILE];
  char line[MAX_LINE];
  char *line_start;
  size_t read_size = 0;
  int is_next_data = 0;

  memcheck_glb.cur_stack_size = 0;
  memcheck_glb.cur_heap_size = 0;
  memcheck_glb.cur_annony_size = 0;
  memcheck_glb.cur_data_size = 0;
  memcheck_glb.cur_misc_size = 0;

  sprintf(filename, "/proc/%d/smaps", memcheck_glb.pid);
  fd = open(filename, O_RDONLY);
  if (fd < 0) {
    perror(filename);
    assert(0);
  }

  while (1) {
    ret = read(fd, &buf[read_size], sizeof(buf));
    read_size += ret;
    if (ret < 0 || read_size == sizeof(buf)) {
      perror("Possible issue with reading proc smaps");
      assert(0);
    } else if (ret == 0) {
      break;
    }
  }

  /**
   * Probably the child died abruptly. Probably issue in student code.
   * FIXME: Give better warning to student than this assert
   */
  if (read_size == 0) assert(0 && "Did the child died?");

  line_start = buf;
  while (line_start != NULL) {
    char sectionname[200], temp_sectionname[200];
    char perm[10];
    int t1, t2, t3;
    void *start_addr, *end_addr, *p1;
    size_t section_size;
    int ret;

    while (1) {
      get_line(&line_start, line);
      if (line_start == NULL) break;

      temp_sectionname[0] = '\0';
      ret = sscanf(line, "%p-%p %s %p %x:%x %x %s", &start_addr, &end_addr,
                   perm, &p1, &t1, &t2, &t3, temp_sectionname);
      if (ret == 8) {
        /* Matched all */
        strncpy(sectionname, temp_sectionname, sizeof(sectionname));
        section_size = (uintptr_t)end_addr - (uintptr_t)start_addr;
        break;
      } else if (ret == 7) {
        /* Section name empty - Annoymous */
        assert(temp_sectionname[0] == '\0');
        sectionname[0] = '\0';
        section_size = (uintptr_t)end_addr - (uintptr_t)start_addr;
        break;

      } else {
        /* Ignore the attributes - Not needed for now */
      }
    }

    if (line_start == NULL) break;

    if (strcmp(sectionname, PROCMAPS_STACK_S) == 0 ||
        (end_addr == memcheck_glb.stack_start_addr)) {
      /*
       * XXX: In proc maps, sometimes the stack is mentioned
       * twice:
       * E.g.
       * 7fff8f8e1000-7fff8f902000 rw-p 00000000 00:00 0  [stack]
       *
       * ff8f8e1000-7fff8f902000 rw-p 00000000 00:00 0 [stack]
       */
      if (memcheck_glb.cur_stack_size == 0) {
        memcheck_glb.stack_start_addr = end_addr;
        memcheck_glb.cur_stack_size = section_size;

        memcheck_glb.max_stack_size =
            MAX(memcheck_glb.cur_stack_size, memcheck_glb.max_stack_size);
      } else {
        /*
         * FIXME: Can't identify why this assert fails
         * Not an issue currently as we don't use
         * procmaps for stack
         */
        // assert(memcheck_glb.cur_stack_size ==
        //	(size_t)section_size);
      }

    } else if (strcmp(sectionname, PROCMAPS_HEAP_S) == 0) {
      /*
       * There should be only one heap or
       * same heap mapped more than once?
       */
      assert(memcheck_glb.cur_heap_size == 0 ||
             memcheck_glb.cur_heap_size == (size_t)section_size);

      memcheck_glb.cur_heap_size += section_size;

      memcheck_glb.max_heap_size =
          MAX(memcheck_glb.cur_heap_size, memcheck_glb.max_heap_size);

    } else if (strstr(sectionname, CHILD_EXE_NAME) != NULL) {
      /* Check data section */
      if (strstr(perm, "w")) {
        /*
         * There should be only one data section
         * or same data section mapped more than once?
         */
        assert(memcheck_glb.cur_data_size == 0 ||
               memcheck_glb.cur_data_size == (size_t)section_size);

        memcheck_glb.cur_data_size += section_size;

        memcheck_glb.max_data_size =
            MAX(memcheck_glb.cur_data_size, memcheck_glb.max_data_size);

        /* Next might be possibly data */
        is_next_data = 1;
        continue;
      }

    } else if ((sectionname[0] == '\0') && (is_next_data == 1) &&
               (strstr(perm, "w") != NULL)) {
      memcheck_glb.cur_data_size += section_size;

      memcheck_glb.max_data_size =
          MAX(memcheck_glb.cur_data_size, memcheck_glb.max_data_size);

    } else if (strcmp(sectionname, PROCMAPS_ANNONY_S) == 0) {
      /* Only count writtable annony mappings */
      if (strstr(perm, "w")) {
        memcheck_glb.cur_annony_size += section_size;

        memcheck_glb.max_annony_size =
            MAX(memcheck_glb.cur_annony_size, memcheck_glb.max_annony_size);
      }

    } else if (strstr(perm, "w")) {
      /*
       * XXX: This can be libraries read/write or someone
       * mmap'ing
       * Anything malloced by libraries - Where is that
       * data?
       */
      memcheck_glb.cur_misc_size += section_size;

      memcheck_glb.max_misc_size =
          MAX(memcheck_glb.cur_misc_size, memcheck_glb.max_misc_size);
    }

    is_next_data = 0;
  }
  close(fd);

  assert(memcheck_glb.cur_stack_size > 0);
  /* At very start, heap is not assinged if not used */
  /*assert(memcheck_glb.cur_heap_size > 0); */
  assert(memcheck_glb.cur_annony_size > 0);
  assert(memcheck_glb.cur_data_size > 0);
  assert(memcheck_glb.cur_misc_size > 0);

  memcheck_glb.max_usage =
      MAX(memcheck_glb.max_usage,
          memcheck_glb.cur_stack_size + memcheck_glb.cur_heap_size +
              memcheck_glb.cur_annony_size + memcheck_glb.cur_data_size +
              memcheck_glb.cur_misc_size);

  memcheck_glb.update_count++;

  return 0;
}

/**
 * @brief signal handler which updates memory usage stats
 * @param signal Signal which was received
 */
void timer_handler(int signal) {
  /* Save and restore errno */
  int old_errno = errno;

  assert(signal == SIGALRM);
  if (!disable_update) assert(update_memusage() == 0);

  errno = old_errno;
}
/**
 * @brief Initializes the memcheck functionality on parent side
 * @param pid Pid of the child. So call this after child is forked
 * @return 0 on success, < 0 on error
 */
int init_memcheck_parent(pid_t pid) {
  int ret;
  struct itimerval time_period;
  sighandler_t old_sighandler;
  sigset_t set;

  time_period.it_interval.tv_sec = 0;
  time_period.it_interval.tv_usec = PERIOD_US_MEMCHECK;
  time_period.it_value.tv_sec = 0;
  time_period.it_value.tv_usec = PERIOD_US_MEMCHECK;

  memcheck_glb.pid = pid;

  /* Protect from simuntaneous update from signal handler */
  disable_update = 1;

  /* Do an update - To early detect any error */
  ret = update_memusage();
  if (ret < 0) return ret;

  disable_update = 0;

  memcheck_glb.init_usage = memcheck_glb.max_usage;

  /* Install timer handler */
  old_sighandler = signal(SIGALRM, timer_handler);
  if (old_sighandler == SIG_ERR) return -1;

  /* Install timer to periodically update */
  ret = setitimer(ITIMER_REAL, &time_period, NULL);
  if (ret < 0) return ret;

  /* Unblock signal */
  ret = sigemptyset(&set);
  if (ret < 0) return ret;

  ret = sigaddset(&set, SIGALRM);
  if (ret < 0) return ret;

  ret = sigprocmask(SIG_UNBLOCK, &set, NULL);
  if (ret < 0) return ret;

  return 0;
}

/**
 * @brief Gives the maximum memory used by child
 */
size_t get_max_memusage(void) { return memcheck_glb.max_usage; }

/**
 * @brief Gives the maximum stack used by child
 */
size_t get_max_stacksize(void) { return memcheck_glb.max_stack_size; }

/**
 * @brief Gives the maximum heap used by child
 */
size_t get_max_heapsize(void) { return memcheck_glb.max_heap_size; }

/**
 * @brief Gives the maximum size used by annonymous mapping in child
 */
size_t get_max_annonysize(void) { return memcheck_glb.max_annony_size; }

/**
 * @brief Gives the maximum size used by data section
 */
size_t get_max_datasize(void) { return memcheck_glb.max_data_size; }

/**
 * @brief Gives the maximum size used by libraries etc.
 */
size_t get_max_miscsize(void) { return memcheck_glb.max_misc_size; }

/**
 * @brief Deinitializes the memcheck functionality
 * @return 0 on success, < 0 on error
 */
int deinit_memcheck_parent(void) {
  sigset_t set;

  /* Do an update - Before leaving */
  int ret;

  /* Stop timer interrupt */
  ret = sigemptyset(&set);
  if (ret < 0) return ret;

  ret = sigaddset(&set, SIGALRM);
  if (ret < 0) return ret;

  ret = sigprocmask(SIG_BLOCK, &set, NULL);
  if (ret < 0) return ret;

  ret = update_memusage();
  if (ret < 0) return ret;

  assert(memcheck_glb.max_stack_size > 0);
  /*assert(memcheck_glb.max_heap_size > 0);*/
  assert(memcheck_glb.max_annony_size > 0);
  assert(memcheck_glb.max_data_size > 0);
  assert(memcheck_glb.max_misc_size > 0);

  /* Uncomment for debugging (Note: This might not be the place you
   * want to see the memory stats)*/
  // #if PRINT_STATS_ENABLE
  // print_memusage(0);
  // #endif
  return 0;
}
