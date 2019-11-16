/*!
 * \file      wdt-board.c
 *
 * \brief     Target board WDT driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *              (C)2015-2019 EmOne
 *
 * \endcode
 *
 * \author    Anol Paisal ( EmOne )
 */
#include "stm32l1xx.h"
#include "utilities.h"
#include "board.h"

static WWDG_HandleTypeDef WWdtHandle;
static IWDG_HandleTypeDef IWdtHandle;

void WdtInit( Wdt_t *obj, WdtId_t wdtId)
{
    CRITICAL_SECTION_BEGIN( );

    obj->WdtId = wdtId;

    if( wdtId == WDT_WWDG )
    {
        __HAL_RCC_WWDG_CLK_ENABLE();
        WWdtHandle.Instance = WWDG;
        WWdtHandle.Init.Prescaler = WWDG_PRESCALER_8;
        WWdtHandle.Init.Window = 64;
        WWdtHandle.Init.Counter = 64;
        WWdtHandle.Init.EWIMode = WWDG_EWI_ENABLE;
        if(HAL_WWDG_Init(&WWdtHandle) != HAL_OK)
        {

        }
        HAL_NVIC_SetPriority( WWDG_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( WWDG_IRQn ); 
    } 
    else if( wdtId == WDT_IWDG)
    {
        __HAL_RCC_LSI_ENABLE();
        IWdtHandle.Instance = IWDG;
        IWdtHandle.Init.Prescaler = IWDG_PRESCALER_128;
        IWdtHandle.Init.Reload = 4095;
        if(HAL_IWDG_Init(&IWdtHandle) != HAL_OK)
        {

        }
    }

    CRITICAL_SECTION_END( );
}

void WdtRefresh( Wdt_t *obj )
{
    CRITICAL_SECTION_BEGIN( );
    if(obj->WdtId == WDT_WWDG)
    {
        HAL_WWDG_Refresh(&WWdtHandle);
    }
    else if (obj->WdtId == WDT_IWDG)
    {
        HAL_IWDG_Refresh(&IWdtHandle);
    }
    
    CRITICAL_SECTION_END( );
}

/**
  * @brief This function handles Window watchdog interrupt.
  */
void WWDG_IRQHandler(void)
{
  /* USER CODE BEGIN WWDG_IRQn 0 */

  /* USER CODE END WWDG_IRQn 0 */
  HAL_WWDG_IRQHandler(&WWdtHandle);
  /* USER CODE BEGIN WWDG_IRQn 1 */

  /* USER CODE END WWDG_IRQn 1 */
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    printf("***Hardware fault****\r\n");
}
