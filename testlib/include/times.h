/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
 * @file          times.h
 * @ingroup       testlib
 * @author        Min Xu
 *
 * A timer
 */
 
#ifndef __LIBTIME_H
#define __LIBTIME_H

/******************************************************************
 * Functions
 *****************************************************************/

/**
 *astime generate the time string
 */
extern void show_time1(void);

/**
 *ctime generate the time string
 */
extern void show_time2(void);

/**
 *gettimeofday generate the sec usec and time zone
 */
extern void show_time_and_zone(void);

/**
 *gmtime 1900  generate the date and time string
 */
extern void show_time3(void);

/**
 *localtime 1900 generate the local time (zone)
 */
extern void show_time4(void);

/**
 *time get the sec form 1970
 */
extern void show_time1_1970sec(void);

/**
 *mktime get the sec form 1970
 */
extern void show_time2_1970sec(void);

/**
 *Get the some type system clock value
 */
extern void show_timer_clock(void);

/**
 *show difftime generate the different time value
 */
extern void show_diff_time(void);
#endif
