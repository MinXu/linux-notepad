/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          printLine.c
* @ingroup       testmodule
* @author        Min Xu
*
* A PRINT LINE
*/
#include<linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

/****************************************************************
 *Functions
 ****************************************************************/
 /**
 * kernel print the filename and line.
 * @return 0 success.
 */
int printLine(void)
{
	printk("\n===$s: %d\n\n",__FILE__,__LINE__);
	return 0;
}
/*export printLine symbol*/
EXPORT_SYMBOL(printLine);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Generic configurable MTD map driver");

