/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          logger.h
* @ingroup       testlib
* @author        Min Xu
*
* A timer
*/

#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

#define NO_RET_CODE_ERROR
#include "retCodeDef.h"

/***********
 * Defines *
 ***********/

#define MAX_LOGGER_TYPES   30 ///< Max number of logger types allowed
#define UTILS_IDX          0  ///< Static index for UTILS_MACROS logger calls

/********************
 * Type Definitions *
 ********************/

/**
 * Define multiple debug log levels. The value of each message
 * corresponds to the levels given for QNX's slogf() function.
 * Otherwise, there is no coupling to slogf().  Specific numbers
 * are given so that other error levels may be inserted in the
 * future without changing the relative value of modules
 * previously built with logger.
 */
typedef enum
{
   NOMSG    = 0,  ///< disable logging (used to prevent default print levels)
   ERRMSG   = 2,  ///< error message
   WARNMSG  = 3,  ///< warning message
   INFMSG   = 5,  ///< informational message
   DBGMSG   = 6,  ///< debug message (normal detail)
   DBG2MSG  = 7   ///< debug message (fine detail)
} tLogLevel;

/**
 * Function pointer definition for providing an alternative
 * logging (printing) function than the standard printf(). An
 * example of using loggerPrintFunc would be to provide output
 * to slogger in QNX:
 * @code
 * int useSlogger(tLogLevel errLevel, va_list args)
 * {
 *   const char *fmt;
 *
 *   fmt = va_arg(args, char *);
 *   switch(errLevel)
 *   {
 *   case ERRMSG:
 *     return vslogf(opcode, _SLOG_ERROR, fmt, args);
 *   case WARNMSG:
 *     return vslogf(opcode, _SLOG_WARNING, fmt, args);
 *   case INFMSG:
 *     return vslogf(opcode, _SLOG_INFO, fmt, args);
 *   case DBGMSG:
 *   case DBG2MSG:
 *   default:
 *     return vslogf(opcode, _SLOG_DEBUG1, fmt, args);
 *   }
 * }
 * @endcode
 */
typedef int (*tLoggerPrintFunc)(tLogLevel errLevel, va_list args);

/*****************
 * API Functions *
 *****************/

/**
 * Register a logging type.  Whenever this type is used in
 * conjunction with logger(), the included debug output will be
 * printed. If the type already exists, the previously defined
 * values will be used and the current call will have no effect.
 * To change the logging level of a type, use the
 * loggerModifyType() function.
 * @note there is no built-in protection for multi-threaded (or
 *       interrupt) modification of the type database.  If
 *       (de)registrations can occur asynchronously from
 *       logger() calls, then external protection (i.e. a mutex)
 *       must be provided by the user.
 * @param[in] name a string containing the debug type to
 *       register
 * @param[in] verbosity specify which log levels to print. For
 *                   example, if ERRMSG is used, only ERRMSG
 *                   types will be printed.  If WARNMSG is used,
 *                   then WARNMSG and ERRMSG types will be
 *                   printed, etc.
 * @param[in] printFunc an override print function for this
 *                  type. Set to NULL to use the current global
 *                  default (initially printf()).
 *                  @see description of tLoggerPrintFunc for
 *                  more information.
 * @param[out] idx index to registered type (needed for logger
 *       calls), -1 on error
 * @return J4AVB_OK on success, J4AVB_EBOUNDS if no slots are
 *         available, J4AVB_EARG for invalid verbosity,
 *         J4AVB_EINVAL for a name that is too long, other for
 *         other errors
 */
tReturnCode loggerRegisterType(const char *name, tLogLevel verbosity,
                               tLoggerPrintFunc printFunc, int8_t *idx);

/**
 * Modify the log verbosity or print function for a previously
 * defined type.  If the type has not been defined,
 * J4AVB_ENOTFOUND will be returned (use loggerRegisterType()
 * for new types).
 * @see note in loggerRegisterType()
 * @param name a string containing the debug type to register
 * @param verbosity specify which log levels to print. For
 *                   example, if ERRMSG is used, only ERRMSG
 *                   types will be printed.  If WARNMSG is used,
 *                   then WARNMSG and ERRMSG types will be
 *                   printed, etc.
 * @param printFunc an override print function for this type.
 *                  Set to NULL to use the current global
 *                  default (initially printf()).
 *                  @see description of tLoggerPrintFunc for
 *                  more information.
 * @return J4AVB_OK on success, J4AVB_ENOTFOUND if the type has
 *         not been defined, other for other errors
 */
tReturnCode loggerModifyType(const char *name, tLogLevel verbosity, tLoggerPrintFunc printFunc);

/**
 * Deregister a logging type.  This type will no longer be
 * printed.
 * @see note in loggerRegisterType()
 * @param name a string containing the debug type to deregister
 * @return J4AVB_OK on success, other for other errors
 */
tReturnCode loggerDeregisterType(const char *name);

/**
 * Print the names of the currently registered types, their
 * current print verbosity, and the slot they reside in. The
 * user provides the print function, which is called once for
 * each registered type. The printFunc could be something as
 * simple as printing the list to the console. It could be
 * something more fancy, such as a function that is used to
 * search the current registered types.
 * @see note in loggerRegisterType()
 * @param printFunc user provided print function for printing
 *                  the list. It is called once for each
 *                  registered type. If NULL, the name will just
 *                  be printed using the current global default,
 *                  which is initially printf(). The first
 *                  parameter is the format string; the second
 *                  parameter is the name of the type; the third
 *                  parameter is the current verbosity, and the
 *                  fourth parameter is the index of the type.
 * @note Below is an example of how to delete all currently
 *       registered types. The same idea could be used to
 *       change the verbosity of all types, etc.
 * @code
 *    static int typeKiller(tLogLevel errLevel, va_list args)
 *    {
 *       int res = 0;
 *       const char *format;
 *       const char *name;
 *
 *       (void)errLevel;
 *
 *       format = va_arg(args, char *);
 *       (void)format; // format string not used here
 *       name = va_arg(args, char *);
 *       loggerDeregisterType(name);
 *
 *       return res;
 *    }
 *
 * loggerPrintRegisteredTypes(typeKiller);
 * @endcode
 * @return tReturnCode J4AVB_OK on success, J4AVB_ENOTFOUND if
 *         there are no registered types.
 */
tReturnCode loggerPrintRegisteredTypes(tLoggerPrintFunc printFunc);

/**
 * Change the default printer function. This provides a global
 * method for changing the debug output for all debug types that
 * were registered with "NULL" in the printFunc parameter.
 * Without modification, the default printer function prints to
 * printf(). This provides a way to globally change where debug
 * output goes if the print function is not specified in a
 * logger call. To return to the internal printf() default, set
 * printFunc to NULL.
 * @see code example of tLoggerPrintFunc definition
 * @param printFunc new default printer function, or NULL to
 *                  return to internal printf() default
 * @return tReturnCode J4AVB_OK on success, other on error
 */
tReturnCode loggerChangeDefaultPrinter(tLoggerPrintFunc printFunc);

/**
 * Log a message. This can be a simple as printing to the
 * console, or more advanced like printing to the syslog in QNX.
 * @see note in loggerRegisterType()
 * @param idx the index of the type registered previously with
 *             loggerRegisterType().
 * @param errLevel the type of message to print
 * @param ... variadic parameter list, consisting of format
 *        string, followed by any conversion specifiers
 * @return int the number of characters printed, similar to
 *         standard printf
 */
extern int logger(uint8_t idx, tLogLevel errLevel, ...);

/**
 * Same as calling logger directly, except additional
 * information is printed (with the exception that the actual
 * structure passed instead of a pointer). The function, line,
 * and file of the debug message are printed before the user's
 * message.
 * @see note in loggerRegisterType()
 * @note the additional fmt parameter is needed to make this
 *       macro work properly.  It may still be used as a
 *       one-for-one replacement for logger()
 * @todo if no conversion specifiers (%) are used, the 4th
 *       argument must be NULL so that there is something in
 *       __VA_ARGS__. Is there a way around this?
 */
#ifdef __TI_COMPILER_VERSION__
/*
 * [Dong He]: TI compiler does not suppprt C99, so we have to
 *            change this macro definition to a general function
 */
int LOG(uint8_t idx, tLogLevel errLevel, const char *fmt, ...);
#else
#define LOG(idx, errLevel, fmt, ...)                 \
  logger(idx, errLevel, "%s() in %s, line %i: " fmt, \
         __func__, __FILE__, __LINE__, __VA_ARGS__)
#endif

#endif

