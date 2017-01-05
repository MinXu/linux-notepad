/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          logger.c
* @ingroup       utils
* @author        Min Xu
* @brief         A set of utilities used for debug printing and logging.
*/

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "logger.h"

/***********
 * Defines *
 ***********/

#define MAX_LOG_NAME_LEN   25

/***************
 * Local Types *
 ***************/

/**
 * @param name the name registered for the type.  This is the
 *             field that will be used for registering and
 *             deregistering types.
 * @param verbosity the logging level for print verbosity
 * @param printFunc function that should be called when logger
 *                  is called matching the name parameter. Use
 *                  NULL to use standard printf().
 * @param inUse true if this instance is in use
 */
typedef struct logType
{
   bool              inUse;
   char              name[MAX_LOG_NAME_LEN];
   tLogLevel         verbosity;
   tLoggerPrintFunc  printFunc;
} tLogType;

/***********************
 * Function Prototypes *
 ***********************/

static int usePrintf(tLogLevel errLevel, va_list args);
static void loggerPrintTypesHelper(tLoggerPrintFunc printFunc, ...);
static bool logLevelOkay(tLogLevel level);

/***********
 * Globals *
 ***********/

static tLoggerPrintFunc defaultPrinter = usePrintf;
static tLogType         logArray[MAX_LOGGER_TYPES] = {{false,"",0,NULL}};  /*.inUse = false*/
static uint8_t          startIdx       = 1;

static const char *logLevelNames[] =
{
   "NOMSG",     // NOMSG
   NULL,        // Not defined!!!
   "ERRMSG",    // ERRMSG
   "WARNMSG",   // WARNMSG
   NULL,        // Not defined!!!
   "INFMSG",    // INFMSG
   "DBGMSG",    // DBGMSG
   "DBG2MSG"    // DBG2MSG
};

/*****************
 * API Functions *
 *****************/

tReturnCode loggerRegisterType(const char *name, tLogLevel verbosity,
                               tLoggerPrintFunc printFunc, int8_t *idx)
{
   uint8_t i;
   uint8_t curIdx;

   if (idx == NULL)
   {
      return J4AVB_ENULLPTR;
   }

   if (name == NULL)
   {
      *idx = -1;
      return J4AVB_ENULLPTR;
   }

   if (!logLevelOkay(verbosity))
   {
      return J4AVB_EARG;
   }

   if (strlen(name) > (MAX_LOG_NAME_LEN - 1))
   {
      return J4AVB_EINVAL;
   }

   // see if the name is already registered
   for (i = 0; i < MAX_LOGGER_TYPES; i++)
   {
      if (logArray[i].inUse && !strcmp(name, logArray[i].name))
      {
         *idx = (int8_t)i;
         return J4AVB_OK;
      }
   }

   /* find next available slot */

   // special case (don't want utils.h macros
   // to have to call register function each
   // time since there is no init function for utils)
   if (!strcmp("UTILS_MACROS", name))
   {
      strcpy(logArray[UTILS_IDX].name, name);
      logArray[UTILS_IDX].verbosity = verbosity;
      logArray[UTILS_IDX].printFunc = printFunc;
      logArray[UTILS_IDX].inUse = true;
      *idx = UTILS_IDX;

      return J4AVB_OK;
   }

   curIdx = startIdx;
   for (i = (UTILS_IDX + 1); i < MAX_LOGGER_TYPES; i++)
   {
      if (!logArray[curIdx].inUse)
      {
         strcpy(logArray[curIdx].name, name);
         logArray[curIdx].verbosity = verbosity;
         logArray[curIdx].printFunc = printFunc;
         logArray[curIdx].inUse = true;
         *idx = (int8_t)curIdx;

         startIdx = (uint8_t)(curIdx + 1);
         if (startIdx == MAX_LOGGER_TYPES)
         {
            startIdx = UTILS_IDX + 1;
         }

         break;
      }

      curIdx++;
      if (curIdx == MAX_LOGGER_TYPES)
      {
         curIdx = UTILS_IDX + 1;
      }
   }

   // was a slot found?
   if (i == MAX_LOGGER_TYPES)
   {
      *idx = -1;
      return J4AVB_EBOUNDS;
   }

   return J4AVB_OK;
}

tReturnCode loggerModifyType(const char *name, tLogLevel verbosity, tLoggerPrintFunc printFunc)
{
   uint8_t i;

   if (name == NULL)
   {
      return J4AVB_ENULLPTR;
   }

   if (!logLevelOkay(verbosity))
   {
      return J4AVB_EARG;
   }

   // find the entry and update it
   for (i = 0; i < MAX_LOGGER_TYPES; i++)
   {
      if (logArray[i].inUse && !strcmp(name, logArray[i].name))
      {
         logArray[i].verbosity = verbosity;
         logArray[i].printFunc = printFunc;

         return J4AVB_OK;
      }
   }

   return J4AVB_ENOTFOUND;
}

tReturnCode loggerDeregisterType(const char *name)
{
   uint8_t i;

   if (name == NULL)
   {
      return J4AVB_ENULLPTR;
   }

   // find the entry and remove it
   for (i = 0; i < MAX_LOGGER_TYPES; i++)
   {
      if (logArray[i].inUse && !strcmp(name, logArray[i].name))
      {
         logArray[i].inUse = false;
      }
   }

   return J4AVB_OK;
}

tReturnCode loggerPrintRegisteredTypes(tLoggerPrintFunc printFunc)
{
   uint8_t i;

   // find the entry and remove it
   for (i = 0; i < MAX_LOGGER_TYPES; i++)
   {
      if (logArray[i].inUse)
      {
         loggerPrintTypesHelper(printFunc,
                                "%s: %s verbosity, in slot %u",
                                logArray[i].name,
                                logLevelNames[logArray[i].verbosity],
                                i);
      }
   }

   return J4AVB_OK;
}

tReturnCode loggerChangeDefaultPrinter(tLoggerPrintFunc printFunc)
{
   if (printFunc == NULL)
   {
      defaultPrinter = usePrintf;
   }
   else
   {
      defaultPrinter = printFunc;
   }

   return J4AVB_OK;
}

int logger(uint8_t idx, tLogLevel errLevel, ...)
{
   int ret;
   va_list argp;

   if (idx >= MAX_LOGGER_TYPES)
   {
      return 0;
   }

   if (logArray[idx].inUse == false)
   {
      return 0;
   }

   // only print at the user specified level
   if (logArray[idx].verbosity < errLevel)
   {
      return 0;
   }

   va_start(argp, errLevel);
   if (logArray[idx].printFunc != NULL)
   {
      ret = logArray[idx].printFunc(errLevel, argp);
   }
   else
   {
      ret = defaultPrinter(errLevel, argp);
   }
   va_end(argp);

   return ret;
}

/*******************
 * Local Functions *
 *******************/

/**
 * the tLoggerPrintFunc function pointer used in
 * loggerPrintRegisteredTypes() needs a va_list passed to it.
 * This variadic version takes an undefined number of arguments,
 * converts them to a va_list, then passes that list to the user
 * provided function.  If the provided function is NULL, then
 * the output is passed to the usePrintf() function which just
 * prints to the console.
 * @param printFunc user-provided function that is should be
 *                  called once the va_list is defined. If NULL,
 *                  usePrintf() is called instead.
 */
static void loggerPrintTypesHelper(tLoggerPrintFunc printFunc, ...)
{
   va_list argp;

   va_start(argp, printFunc);
   if (printFunc != NULL)
   {
      printFunc(INFMSG, argp);
   }
   else
   {
      defaultPrinter(INFMSG, argp);
   }
   va_end(argp);
}

/**
 * Use standard console print functions for debug output.  This
 * function is used if the printFunc function pointer is NULL,
 * meaning that the user has not supplied their own function.
 * @param errLevel if ERRMSG, print to stderr, otherwise, print
 *                 to stdout
 * @param args variadic list of arguments.  The first argument
 *             is the format string, and the remaining values
 *             are conversion specifiers, just like normal
 *             printf
 * @return int return the number of characters printed (just
 *         like printf)
 */
static int usePrintf(tLogLevel errLevel, va_list args)
{
   const char *fmt;

   fmt = va_arg(args, char *);
   if (errLevel == ERRMSG)
   {
      return vfprintf(stderr, fmt, args);
   }
   else
   {
      return vprintf(fmt, args);
   }
}

/**
 * Check log level.
 * @param level the log level to check
 * @return true if valid, false if not
 */
static bool logLevelOkay(tLogLevel level)
{
   switch(level)
   {
   case NOMSG:
   case ERRMSG:
   case WARNMSG:
   case DBGMSG:
   case DBG2MSG:
   case INFMSG:
      return true;
   default:
      return false;
   }
}

#ifdef __TI_COMPILER_VERSION__
int LOG(uint8_t idx, tLogLevel errLevel, const char *fmt, ...)
{
   int ret;
   va_list argp;

   va_start(argp, fmt);

   ret = logger(idx, errLevel, fmt, argp);

   va_end(argp);

   return ret;
}
#endif


