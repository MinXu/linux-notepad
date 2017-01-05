/****************************************************************
 * Project        Library Support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          retCodeDef.h
* @ingroup       testlib
* @author        Min Xu
*
* A timer
*/

#ifndef NO_RET_CODE_ERROR
#error retCodeDef.h should not be included manually, use retCodes.h instead.
#endif

#ifndef RET_CODE_DEF_H
#define RET_CODE_DEF_H

/**
 * Standard error/return codes used in all J4AVB modules.
 */
typedef enum
{
   J4AVB_OK = 0,        /**< no error */
   J4AVB_ERR,           /**< unspecified error */
   J4AVB_EINIT,         /**< resource not initialized */
   J4AVB_EREINIT,       /**< resource already initialized */
   J4AVB_EARG,          /**< invalid argument error */
   J4AVB_EALLOC,        /**< memory allocation error */
   J4AVB_EBOUNDS,       /**< dynamic boundary check error */
   J4AVB_ENOTFOUND,     /**< item not found error */
   J4AVB_ENULLPTR,      /**< null pointer */
   J4AVB_EFILE,         /**< problem opening file (see errno.h) */
   J4AVB_ERET,          /**< error in return value */
   J4AVB_EINVAL,        /**< error invalid */
   J4AVB_EOTHER         /**< non-standard return type */
} tReturnCode;

#endif

