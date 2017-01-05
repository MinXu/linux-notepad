/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          print.h
* @ingroup       testmodule
* @author        Min Xu
*
* A PRINT
*/
#include <linux/module.h>
#include <linux/kernel.h>	
#include <linux/init.h>
#include "../include/printFunc.h"
#include "../include/printLine.h"
#include "../include/version.h"

/****************************************************************
 *Functions
 ****************************************************************/
 /**
 * kernel print module initialization.
 */
static int __init print_init(void)
{
	printFunc();
	printLine();
	printk("version:%s\n",sTestmoduleVersion);
	return 0;
}

 /**
 * kernel print module distory.
 */
static void __exit print_exit(void)
{
	return;
}


module_init(print_init);
module_exit(print_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("Generic configurable MTD map driver");
