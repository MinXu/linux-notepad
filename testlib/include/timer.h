/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          timer.h
* @ingroup       testlib
* @author        Min Xu
*
* A timer
*/
#ifndef __TIMER_H__
#define __TIMER_H__
#include <semaphore.h>
/******************************************************************
 * Defines
 *****************************************************************/
/*default timer interval second*/
#define ALARM_INTERVAL 		1

/**
 * Timer type.
 * @param ALARM                   signal alarm.
 * @param SETITIMER_TIMER_REAL    setitimer timer_real.
 * @param SETITIMER_TIMER_VRITUAL setitimer timer_virtrual.
 * @param SETITIMER_TIMER_PROF    setitimer timer_prof.
 * @param SECLECT                 select.
 * @param PROSIX_TIMER_PTHREAD    prosix timer pthread event.
 * @param PROSIX_TIMER_SIGNAL     prosix timer signal event.
 */
typedef enum
{
	ALARM,
	SETITIMER_TIMER_REAL,
	SETITIMER_TIMER_VRITUAL,
	SETITIMER_TIMER_PROF,
	SECLECT,
	PROSIX_TIMER_PTHREAD,
	PROSIX_TIMER_SIGNAL,
	END_TYPE
}timerType;

/**
 * Timer Module type.
 * @param type    Timer type.
 * @param desc    describe pointer.
 */
typedef struct
{
        timerType type;
        char* desc;
}timerModule;

/******************************************************************
 * Functions
 *****************************************************************/
/**
 * Timer Module type and describe.
 * @param ALARM                   signal alarm.
 * @param SETITIMER_TIMER_REAL    setitimer timer_real.
 * @param SETITIMER_TIMER_VRITUAL setitimer timer_virtrual.
 * @param SETITIMER_TIMER_PROF    setitimer timer_prof.
 * @param SECLECT                 select.
 * @param PROSIX_TIMER_PTHREAD    prosix timer pthread event.
 * @param PROSIX_TIMER_SIGNAL     prosix timer signal event.
 */
extern timerModule timerM[];

 /**
 * some signal routine.
 * @param[in] signo       signal ID.
 */
void sigroutine(int signo);

/**
 * Invoke some method and make the process or thread sleep a time interval or send a signal after a time interval.
 * @param[in] sem              semaphore pointer for lock the process or thread.
 * @param[in] type             choice of the timer method.
 * @param[in] alarmInterval    setting of the time interval.
 */
void startTimer(sem_t *sem,timerType type,int alarmInterval);
#endif
