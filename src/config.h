#pragma once

/**
 * @file config.h
 * @brief Contains various configuration paramters
 * @author Saksham Jain (sakshamj)
 * @author (Tweaked) Ankush Jain (ankushj)
 */

/******************************************************************************/
/*                       Macros to disable/enable features                    */
/*                 Modifying this file has no effect on autolab               */
/*                         Autolab uses default settings                      */
/*     Before submitting to autolab, test your FTL with default settings      */
/******************************************************************************/

/* Should we track child's memory? */
#define MEMCHECK_ENABLED 1

/*
 * Do you want trace of malloc - Useful for only debugging
 * Helps in gathering malloc trace and gives a visual chart of
 * how memory is being used along on heap using malloc
 *
 * FIXME: Currently when this option is enabled, you might see lots of warnings
 * given out by gcc. This is because we are trapping malloc using
 * deprecated functions. A better way exists and should be implemented,
 * but logically there shouldn't be any issue with current implementation
 */
#define MALLOC_TRACE_ENABLED 0

/* Do you wish to print stats (memory and runtime) of the child */
#define PRINT_STATS_ENABLE 0

/*
 * Methods to stack check
 * Expansion - Uses sigsegv to trap when we detect that stack is to be expanded
 * Canary - Places canary on the stack to see how stack is changing
 * Enable one and only one
 */
#define STACK_CHECK_EXPANSION 0
#define STACK_CHECK_CANARY 1

/* Which method to use to track stack consumption */
#define STACK_CHECK STACK_CHECK_CANARY

/* Enable's large page for the datastore - 4k */
#define ENABLE_LARGE_DATASTORE_PAGE 0

/* Enables tracing of all reads/writes (transcations) requested/performed */
#define ENABLE_TRANS_TRACING 0

/******************************************************************************/
/*                         Don't modify below this                            */
/******************************************************************************/

/*
 * The period in useconds, when parents checks on child's memory usage
 */
#define PERIOD_US_MEMCHECK (10 * 1000)

/* File to use to output malloc tracing data (if feature enabled) */
#define MALLOC_TRACE_FILE OUTDIR "/malloc_trace.dat"

/* Should be enough to cover the initialization without need for expansion */
#define CHILD_INIT_STACK_SIZE 3 * 4096

/* Offset at multiple of which to place guard pages */
#define STACK_CANARY_OFFSET (0x400)
#define STACK_CANARY_OFFSET_MASK (STACK_CANARY_OFFSET - 1)
/* The stack offset at which to start (2KB) */
#define STACK_MIN_OFFSET (2)
/* We don't expect the stack to grow more than 100kb */
#define STACK_MAX_OFFSET (100)

/* At each offset, we place a block of words of canary */
#define STACK_CANARY_BLOCK (10)
/* Random canary that we place */
#define STACK_CANARY (0xFACEDEAD)

/* File to use to output transaction tracing data (if feature enabled) */
#define TRANS_TRACE_FILE OUTDIR "/trans_trace.log"

/*
 * If Two proc is not enabled, some of the features become unaccessible
 * CONFIG_TWOPROC indicated whether flashsim and ftl runs as two seperate
 * process or one. This flag is enabled/disabled via Makfile
 */
#if (CONFIG_TWOPROC == 0)

#undef MEMCHECK_ENABLED
#define MEMCHECK_ENABLED 0

#undef MALLOC_TRACE_ENABLED
#define MALLOC_TRACE_ENABLED 0

#undef PRINT_STATS_ENABLE
#define PRINT_STATS_ENABLE 0

#undef STACK_CHECK_EXPANSION
#define STACK_CHECK_EXPANSION 0

#undef STACK_CHECK_CANARY
#define STACK_CHECK_CANARY 0

#endif /* CONFIG_TWOPROC */