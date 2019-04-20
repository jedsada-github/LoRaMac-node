/*!
 * \file      UserNvmCtxMgmt.c
 *
 * \brief     USER NVM context management implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code

 *
 * \endcode
 *
 * \author    Anol Paisal (anol.p@emone.co.th)
 */

#include <stdio.h>
#include "UserNvmCtxMgmt.h"
#include "utilities.h"
#include "eeprom.h"
#include "nvmm.h"

#define USER_SETTING_CTX_MGMT_ENABLED       1

#if ( USER_SETTING_CTX_MGMT_ENABLED == 1)
#include "encoder.h"
#define NVM_CTX_STORAGE_MASK               0xFF

static NvmmDataBlock_t UserDataNvmCtxDataBlock;
static NvmmDataBlock_t UserConfigNvmCtxDataBlock;
#endif

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtStore( void )
{
#if defined ( USER_SETTING_CTX_MGMT_ENABLED )
    if( NvmmWrite( &UserDataNvmCtxDataBlock, &flow.fwd_cnt,  8) != NVMM_SUCCESS )
    {
        return USER_NVMCTXMGMT_STATUS_FAIL;
    }

    if( NvmmWrite( &UserConfigNvmCtxDataBlock, &config,  sizeof(flow_config_t))!= NVMM_SUCCESS )
    {
        return USER_NVMCTXMGMT_STATUS_FAIL;
    }
    return USER_NVMCTXMGMT_STATUS_SUCCESS;
#else
    return NVMCTXMGMT_STATUS_FAIL;
#endif
}

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtRestore( void )
{
#if defined ( USER_SETTING_CTX_MGMT_ENABLED )
    if ( NvmmDeclare( &UserDataNvmCtxDataBlock, 8 ) == NVMM_SUCCESS )
    {
        NvmmRead( &UserDataNvmCtxDataBlock, &flow.fwd_cnt, 8 );
    }

    if ( NvmmDeclare( &UserConfigNvmCtxDataBlock, sizeof (flow_config_t) ) == NVMM_SUCCESS )
    {
        NvmmRead( &UserConfigNvmCtxDataBlock, &config, sizeof (flow_config_t) );
    }
    return USER_NVMCTXMGMT_STATUS_SUCCESS;
#else
    return USER_NVMCTXMGMT_STATUS_FAIL;
#endif
}
