#pragma once

/*
 * @file memcheck.cpp
 * @brief Header file for memcheck.h
 *
 * @author Saksham Jain (sakshamj)
 * @author (Tweaked) Ankush Jain (ankushj)
 * @bug No known bugs
 */

#include <sys/types.h>

/* Various field's keyword in proc maps */
#define PROCMAPS_STACK_S "[stack]"
#define PROCMAPS_HEAP_S "[heap]"
/* This is the annonynomous mappings in the procmap */
#define PROCMAPS_ANNONY_S ""

/* Threshold over which malloc does mmap */
#define MMAP_THRESHOLD_MAX (16 * 1024 * 1024)

/* Max characters expected in a line in smaps file and max file size*/
#define MAX_LINE 300
#define MAX_FILE 10 * 4096

/* /proc/<pid>/smaps entries */
struct smaps_sizes {
  int KernelPageSize;
  int MMUPageSize;
  int Private_Clean;
  int Private_Dirty;
  int Pss;
  int Referenced;
  int Rss;
  int Shared_Clean;
  int Shared_Dirty;
  int Size;
  int Swap;
  int Locked;
  int Anonymous;
  int AnonHugePages;
};

int update_memusage(void);
int init_memcheck_parent(pid_t pid);
int deinit_memcheck_parent(void);
size_t get_max_memusage(void);
size_t get_max_annonysize(void);
size_t get_max_heapsize(void);
size_t get_max_stacksize(void);
size_t get_max_datasize(void);
size_t get_max_miscsize(void);
void print_memusage(void);
size_t get_child_total_mem(size_t child_stack_size);