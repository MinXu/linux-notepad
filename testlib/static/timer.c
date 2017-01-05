/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          timer.c
* @ingroup       testlib
* @author        Min Xu
*
* A timer
*/

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include "timer.h"

/*---------------------------------------------------------------------
 * Global variables
 *--------------------------------------------------------------------*/
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
timerModule timerM[]={
        {ALARM," signal alarm"},
        {SETITIMER_TIMER_REAL," setitimer timer_real"},
        {SETITIMER_TIMER_VRITUAL," setitimer timer_virtrual"},
        {SETITIMER_TIMER_PROF," setitimer timer_prof"},
        {SECLECT," select"},
        {PROSIX_TIMER_PTHREAD," prosix timer pthread event"},
        {PROSIX_TIMER_SIGNAL," prosix timer signal event"},
	{END_TYPE,NULL}
};

/* Global semaphore share invoke by sigroutine and startTimer*/
sem_t *gpSem;
/* Timer id*/
timer_t timerid;

 /*---------------------------------------------------------------------
 * Functions
 *--------------------------------------------------------------------*/
 /**
 * some signal routine.
 * @param[in] signo       signal ID.
 */
void sigroutine(int signo)
{
	switch(signo)
	{
		case SIGALRM:
			printf("\r\nCatch a signal -- SIGALRM\n");
			setitimer(ITIMER_REAL,0,0);
			sem_post(gpSem);
//			alarm(ALARM_INTERVAL);
	        break;
        	case SIGVTALRM:
			printf("\r\nCatch a signal -- SIGVTALRM\n");
			setitimer(ITIMER_VIRTUAL,0,0);
			sem_post(gpSem);
		break;
		case SIGPROF:
			printf("\r\nCatch a signal --  SIGPROF\n");
			setitimer(ITIMER_PROF,0,0);
			sem_post(gpSem);
		break;
		case SIGUSR1:
			printf("\r\nCatch a signal --  SIGUSR1\n");
			timer_delete(timerid);
			sem_post(gpSem);
		break;
		case SIGUSR2:
			printf("\r\nCatch a signal --  SIGUSR2\n");
			timer_delete(timerid);
			sem_post(gpSem);
                break;
		case SIGABRT:
                        printf("\r\nCatch a signal --  SIGABRT\n");
                        sem_post(gpSem);
                break;
		case SIGTRAP:
			printf("\r\nCatch a signal --  SIGTRAP\n");
			sem_post(gpSem);
	}

}

/**
 * timer_thread will creat and running the thread after a time interval,the thread will send a signal to stop the timer.
 * @param[in] v          the signal paramter value.
 */
void timer_thread(union sigval v)
{
	printf("timer_thread function! %d\n",v.sival_int);
	kill(getpid(),SIGUSR2);
}

/**
 *两个方法安装信号sigaction 和 signal
 *五个方法发送信号kill,raise,alarm,abort,setitimer，其实还有一个sigqueue没试过:-(
 */

/**
 * Invoke some method and make the process or thread sleep a time interval or send a signal after a time interval.
 * @param[in] sem              semaphore pointer for lock the process or thread.
 * @param[in] type             choice of the timer method.
 * @param[in] alarmInterval    setting of the time interval.
 */
void startTimer(sem_t *sem,timerType type,int alarmInterval)
{
	struct timeval tv;
	struct itimerval value,ovalue;
        struct sigevent evp;
	struct itimerspec it,oit;

	struct sigaction act;

	gpSem = sem;
	switch(type)
	{
		case ALARM:
			printf("\r\nalarm sleep %d sec......\n",alarmInterval);
			signal(SIGALRM,sigroutine);
			alarm(alarmInterval);
		break;
		case SETITIMER_TIMER_REAL:
			printf("\r\nsetitimer  real sleep %d sec......\n",alarmInterval);
			signal(SIGALRM,sigroutine);
			value.it_value.tv_sec = alarmInterval;
			value.it_value.tv_usec = 0;
			value.it_interval.tv_sec = alarmInterval;
			value.it_interval.tv_usec = 0;
			setitimer(ITIMER_REAL,&value,&ovalue);
		break;
		case SETITIMER_TIMER_VRITUAL:
			printf("\r\nsetitimer virturl  sleep %d sec......\n",alarmInterval);
			signal(SIGVTALRM,sigroutine);
			value.it_value.tv_sec = alarmInterval;
                        value.it_value.tv_usec = 0;
                        value.it_interval.tv_sec = alarmInterval;
                        value.it_interval.tv_usec = 0;
                        setitimer(ITIMER_VIRTUAL,&value,&ovalue);
		break;
		case SETITIMER_TIMER_PROF:
                        printf("\r\nsetitimer prof  sleep %d sec......\n",alarmInterval);
                        signal(SIGPROF,sigroutine);
                        value.it_value.tv_sec = alarmInterval;
                        value.it_value.tv_usec = 0;
                        value.it_interval.tv_sec = alarmInterval;
                        value.it_interval.tv_usec = 0;
                        setitimer(ITIMER_PROF,&value,&ovalue);
                break;
		case SECLECT:
			printf("\r\nSECLECT  sleep %d sec......\n",alarmInterval);
			signal(SIGABRT,sigroutine);
			tv.tv_sec = alarmInterval;
			tv.tv_usec= 0;
			select(0,NULL,NULL,NULL,&tv);
			abort();
			printf("\r\nSECLECT sleep ..........end\n");
		break;
		case PROSIX_TIMER_PTHREAD:
			signal(SIGUSR2,sigroutine);
                        memset(&evp,0,sizeof(struct sigevent));
                        evp.sigev_value.sival_int = 111;
                        evp.sigev_notify = SIGEV_THREAD;
                        evp.sigev_notify_function = timer_thread;
                        if(timer_create(CLOCK_REALTIME,&evp,&timerid)==-1)
                        {
                                perror("fail to timer_create");
                                exit(-1);
                        }
                        it.it_interval.tv_sec=alarmInterval;
                        it.it_interval.tv_nsec=0;
                        it.it_value.tv_sec=alarmInterval;
                        it.it_value.tv_nsec=0;
                        printf("\r\nPROSIX_TIMER1  sleep %d sec......\n",alarmInterval);
                        if(timer_settime(timerid,0,&it,&oit)==-1)
                        {
                                perror("fail to timer_settime");
                                exit(-1);
                        }
		break;
		case PROSIX_TIMER_SIGNAL:
                        memset(&evp,0,sizeof(struct sigevent));
                        memset(&act,0,sizeof(struct sigaction));
                        act.sa_handler = sigroutine;
                        act.sa_flags = 0;
                        sigemptyset(&act.sa_mask);
                        if(sigaction(SIGUSR1,&act,NULL) == -1)
                        {
                                perror("fail to sigaction");
                                exit(-1);
                        }
                        evp.sigev_signo = SIGUSR1;
                        evp.sigev_notify = SIGEV_SIGNAL;
                        if(timer_create(CLOCK_MONOTONIC,&evp,&timerid)==-1)
                        {
                                perror("fail to timer_create");
                                exit(1);
                        }

                        it.it_interval.tv_sec=alarmInterval;
                        it.it_interval.tv_nsec=0;
                        it.it_value.tv_sec=alarmInterval;
                        it.it_value.tv_nsec=0;
                        printf("\r\nPROSIX_TIMER2  sleep %d sec......\n",alarmInterval);
                        if(timer_settime(timerid,0,&it,&oit)==-1)
                        {
                                perror("fail to timer_settime");
                                exit(-1);
                        }
		break;	
		default:
			printf("\r\ninput the timer mode error ALARM|SETITIMER|SECLECT\n");
			memset(&act,0,sizeof(struct sigaction));
                        act.sa_handler = sigroutine;
                        act.sa_flags = 0;
                        sigemptyset(&act.sa_mask);
                        if(sigaction(SIGTRAP,&act,NULL) == -1)
                        {
                                perror("fail to sigaction");
                                exit(-1);
                        }
			raise(SIGTRAP);
	}
}


