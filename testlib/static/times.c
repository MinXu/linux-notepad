/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
 * @file          times.c
 * @ingroup       testlib
 * @author        Min Xu
 *
 * A timer
 */
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

/*---------------------------------------------------------------------
 * Functions
 *--------------------------------------------------------------------*/

/* 
 *astime generate the time string
 */
void show_time1(void)
{
	printf("****************************************\n");
	{
	printf("****************************************\n");
	time_t timep;
	printf("****************************************\n");
	time (&timep);
	printf("****************************************\n");
	printf("time()->gmtime()->asctime(): %s\n",asctime(gmtime(&timep))); 
	}
	return;
}

/**
 *ctime generate the time string
 */
void show_time2(void)
{
	time_t timep;
	time (&timep);
	printf("time()->ctime(): %s\n",ctime(&timep));
	return;
}

/**
 *gettimeofday generate the sec usec and time zone
 */
void show_time_and_zone(void)
{
	struct timeval tv;
	struct timezone tz;
	gettimeofday (&tv , &tz);
	printf("gettimeofday().tv_sec: %ld\n", tv.tv_sec) ;/*秒*/ 
	printf("gettimeofday().tv_usec: %ld\n",tv.tv_usec);/*微秒*/ 
	printf("gettimeofday().tz_minuteswest: %d\n", tz.tz_minuteswest);/*和Greenwich 时间差了多少分钟*/ 
	printf("gettimeofday().tz_dsttime: %d\n",tz.tz_dsttime);/*日光节约时间的状态*/ 
	return;
}


/**
 *gmtime 1900  generate the date and time string
 */
void show_time3(void)
{
	char *wday[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);
	printf("time()->gmtime(): %d%d%d ",(1900+p->tm_year), (1+p->tm_mon),p->tm_mday);
	printf("%s%d;%d;%d\n", wday[p->tm_wday], p->tm_hour, p->tm_min, p->tm_sec);
	return;
}

/**
 *localtime 1900 generate the local time (zone)
 */
void show_time4(void)
{
	char *wday[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
	time_t timep;
	struct tm *p;
	time(&timep);
	p=localtime(&timep); /*取得当地时间*/
	printf("time()->localtime(): %d%d%d ",(1900+p->tm_year), (1+p->tm_mon), p->tm_mday);
	printf("%s%d:%d:%d\n", wday[p->tm_wday],p->tm_hour, p->tm_min, p->tm_sec);
	return;
}


/**
 *time get the sec form 1970
 */
void show_time1_1970sec(void)
{
	int seconds= time((time_t*)NULL);
	printf("time(): %d\n",seconds);
	return;
}

/**
 *mktime get the sec form 1970
 */
void show_time2_1970sec(void)
{
	time_t timep;
	struct tm *p;
	time(&timep);
	p=localtime(&timep);
	timep = mktime(p);
	printf("time()->localtime()->mktime(): %ld\n",timep);
	return;
}

/**
 *Get the some type system clock value
 */
void show_timer_clock(void)
{
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME,&tp);
	printf("CLOCK_REALTIME : sec[%ld]  nsec[%ld]\n",tp.tv_sec,tp.tv_nsec);
	clock_gettime(CLOCK_MONOTONIC,&tp);
	printf("CLOCK_MONOTONIC : sec[%ld]  nsec[%ld]\n",tp.tv_sec,tp.tv_nsec);
	/*since Linux 2.6.28; Linux-specific*/
//	clock_gettime(CLOCK_MONOTONIC_RAW,&tp);
//	printf("CLOCK_MONOTONIC_RAW : sec[%ld]  nsec[%ld]\n",tp.tv_sec,tp.tv_nsec);
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&tp);
	printf("CLOCK_PROCESS_CPUTIME_ID : sec[%ld]  nsec[%ld]\n",tp.tv_sec,tp.tv_nsec);
	clock_gettime(CLOCK_THREAD_CPUTIME_ID,&tp);
	printf("CLOCK_THREAD_CPUTIME_ID : sec[%ld]  nsec[%ld]\n",tp.tv_sec,tp.tv_nsec);
	return;
}

/**
 *show difftime generate the different time value
 */
void show_diff_time(void)
{
	time_t first,second;
	first=time(NULL);
	usleep(1000000);
	second=time(NULL);
	printf("time()-usleep(1000000)->time()->difftime(): %fseconds\n",difftime(second,first)); 
	return;
}
