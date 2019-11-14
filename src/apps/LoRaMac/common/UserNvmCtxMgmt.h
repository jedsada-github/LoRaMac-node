/*!
 * \file      UserNvmCtxMgmt.h
 *
 * \brief     USER NVM context management implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *        
 *
 * \endcode
 *
 * \author    Anol Paisal (anol.p@emone.co.th)
 *
 * \defgroup  USER NVMCTXMGMT NVM context management implementation
 *            This module implements the NVM context handling
 * \{
 */
#ifndef __USERNVMCTXMGMT_H__
#define __USERNVMCTXMGMT_H__

// #include "LoRaMac.h"

/*!
 * Data structure containing the status of a operation
 */
typedef enum UserNvmCtxMgmtStatus_e
{
    /*!
     * Operation was successful
     */
    USER_NVMCTXMGMT_STATUS_SUCCESS,
    /*!
     * Operation was not successful
     */
    USER_NVMCTXMGMT_STATUS_FAIL
} UserNvmCtxMgmtStatus_t;

/*!
 * \brief Calculates the next datarate to set, when ADR is on or off.
 *
 * \param [IN] adrNext Pointer to the function parameters.
 *
 */
// void UserNvmCtxMgmtEvent( UserNvmCtxModule_t module );

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtStore( void );

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtRestore(void );

#endif // __NVMCTXMGMT_H__
