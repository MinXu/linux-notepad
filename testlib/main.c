/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          main.c
* @ingroup       testlib
* @author        Min Xu
*
* A TEST MAIN
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "times.h"
#include "logger.h"
#include "timer.h"
#include "version.h"
/****************************************************************
 *Functions
 ****************************************************************/
/**
 * Get the current version.
 * @return the version as a const string
 */
static const char *getTestLibVersion(void)
{
   return sTestlibVersion;
}

/**
 * show the command prompt menu.
 */
static void drawMenu()
{
	int i;
	printf("\n\n");
	printf("***test menu***:\n");
	printf("-h:  test help information\n");
	printf("-v:  test version\n");
	printf("-l:  test the log\n");
	printf("-t:  test the times\n");
	printf("-s:  set the timer sleep sec\n");
	printf("-m:  set the timer mode\n");
	for(i=0;timerM[i].type!=END_TYPE;i++)
		printf("     %d:%s\n",timerM[i].type,timerM[i].desc);
	printf("\n\n");

}

/**
 * a linux user layer logger.
 * @param[in] errLevel     register errLevel.
 * @param[in]   va_list  		list the args.
 * @return the log message byte number.
 */
static int appLogger(tLogLevel errLevel, va_list args)
{
   const char* fmt;
   char buffer[512];
   int cnt;

   (void) errLevel;
   fmt = va_arg(args, const char*);
   cnt = vsnprintf(buffer, sizeof(buffer)-2, fmt, args);
   buffer[cnt] = '\n';
   buffer[cnt+1] = '\0';
   printf("%s",buffer);
   return cnt;
}

#define TEST_LOG	0x01
#define TEST_TIMES	0x02
#define TEST_TIMER	0x04

/**
 * a test function entrance.
 * @param[in] argc           paramters number.
 * @param[in] argv  		 paramters list pointer.
 * @return the log message byte number.
 */
int main(int argc,char **argv)
{
	int c;
	unsigned int errflag = 0x0;
	unsigned int options = 0x0;
	unsigned int interval = ALARM_INTERVAL;
	unsigned int type = 0x0;

	while((c = getopt(argc, argv, "hvlts:m:")) != -1)
	{
		switch(c)
		{
			case 'v':
				printf("test version: %s\n",getTestLibVersion());
				exit(0);
			case 'l':
				options |= TEST_LOG;
				errflag = options;
			break;
			case 't':
				options |= TEST_TIMES;
				errflag = options;
			break;
			case 's':
				interval = strtol(optarg, NULL, 0);
			break;
			case 'm':
				options |= TEST_TIMER;
                                type = strtol(optarg, NULL, 0);
				errflag = options;
			break;
			case 'h':
				drawMenu();
				exit(0);
			break;
		}
	}

	if(!errflag)
	{
		drawMenu();
		exit(-1);
	}

	if(options&TEST_LOG)
	{
                int8_t idx;
                #define MODULE_NAME     "logger test"
                loggerRegisterType(MODULE_NAME, DBGMSG, appLogger, &idx);
                logger(idx, INFMSG, "testing............");
                loggerDeregisterType(MODULE_NAME);
	}

	if(options&TEST_TIMES)
	{
		/*astime generate the time string*/
		show_time1();

		/*ctime generate the time string*/
		show_time2();

		/*gettimeofday generate the sec usec and time zone*/
		show_time_and_zone();

		/*gmtime 1900  generate the date and time string*/
		show_time3();

		/*localtime 1900 generate the local time (zone)*/
		show_time4();

		/*time get the sec form 1970*/
		show_time1_1970sec();

		/*mktime get the sec form 1970*/
		show_time2_1970sec();

		/*获取指定时钟的值*/
		show_timer_clock();

		/*show difftime generate the different time value*/
		show_diff_time();
	}
	
	if(options&TEST_TIMER)
	{
		sem_t sem;
		int status;
		const struct timespec tv={
			.tv_sec=0,
			.tv_nsec=1000
		};
	
		status = sem_init(&sem, 0, 0);
		if(status < 0)
		{
			printf("semaphore init error\n");
			exit(-1);
		}

		startTimer(&sem,type,interval);
		if(SETITIMER_TIMER_VRITUAL|SETITIMER_TIMER_PROF)
			while(sem_trywait(&sem));
		else
			while(sem_timedwait(&sem,&tv));
		sem_destroy(&sem);
	}
	return 0;
}
