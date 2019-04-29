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
#define EEPROM_ENCODER_FWD_REV_ADDR       0x100
#define EEPROM_ENCODER_CONF_ADDR          0x110

// #define NVM_CTX_STORAGE_MASK               0xFF

// static NvmmDataBlock_t UserDataNvmCtxDataBlock;
// static NvmmDataBlock_t UserConfigNvmCtxDataBlock;
#endif

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtStore( void )
{
#if defined ( USER_SETTING_CTX_MGMT_ENABLED )

    CRITICAL_SECTION_BEGIN( );
    
    EepromWriteBuffer( EEPROM_ENCODER_FWD_REV_ADDR, (uint8_t *) &flow.fwd_cnt, 8 );
    
    EepromWriteBuffer( EEPROM_ENCODER_CONF_ADDR, (uint8_t *) &config,  sizeof(flow_config_t) );
    
    CRITICAL_SECTION_END( );
    
    return USER_NVMCTXMGMT_STATUS_SUCCESS;
#else
    return NVMCTXMGMT_STATUS_FAIL;
#endif
}

UserNvmCtxMgmtStatus_t UserNvmCtxMgmtRestore( void )
{
#if defined ( USER_SETTING_CTX_MGMT_ENABLED )

    CRITICAL_SECTION_BEGIN( );

    EepromReadBuffer( EEPROM_ENCODER_FWD_REV_ADDR, (uint8_t *) &flow.fwd_cnt, 8 );

    EepromReadBuffer( EEPROM_ENCODER_CONF_ADDR, (uint8_t *) &config, sizeof (flow_config_t) );

    CRITICAL_SECTION_END( );

    return USER_NVMCTXMGMT_STATUS_SUCCESS;
#else
    return USER_NVMCTXMGMT_STATUS_FAIL;
#endif
}
