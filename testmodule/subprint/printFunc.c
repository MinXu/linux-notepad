/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          printFunc.c
* @ingroup       testmodule
* @author        Min Xu
*
* A PRINT FUNCTION
*/
#include <linux/module.h>
#include <linux/init.h>

/****************************************************************
 *Functions
 ****************************************************************/
 /**
 * kernel print the filename and Function name.
 * @return 0 success.
 */
int printFunc(void)
{
	printk("\n====%s: %s()\n\n",__FILE__,__FUNCTION__);
	return 0;
}
/*export printLine symbol*/
EXPORT_SYMBOL(printFunc);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Generic configurable MTD map driver");

