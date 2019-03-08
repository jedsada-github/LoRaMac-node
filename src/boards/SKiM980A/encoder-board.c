/*!
 * \file      encoder-board.c
 *
 * \brief     Target board Encoder driver implementation
 *
* \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______          _______
 *               / _____)        /       \
 *              ( (____   ____   |   _   | __  ___   _____
 *               |____ | |     | |  (_)  | | |/_  | (  ___)
 *              ( (____  | | | | |       | |  / | | (  ___)
 *              (______) |_|_| | \_______/ |_|  | | (_____)
 *              (C)2015-2019 EmOne
 *
 * \endcode
 *
 * \author    Anol Paisal ( EmOne )
 */
#include "stm32l1xx.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "encoder-board.h"

EncoderIrqHandler *EncoderIrq[] = { OnTamperingIrq , OnAlarmIrq, NULL};

static TIM_HandleTypeDef TimHandle;

void EncoderInit( Encoder_t *obj, EncoderId_t timId, PinNames pulse, PinNames dir, PinNames tampering, PinNames alarm)
{
    CRITICAL_SECTION_BEGIN( );

     obj->EncoderId = timId;

     if( timId == TIM_2 )
     {
        __HAL_RCC_TIM2_CLK_ENABLE();

        TimHandle.Instance = ( TIM_TypeDef* )TIM2_BASE;

        GpioInit( &obj->Pulse, pulse, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF1_TIM2 );
        GpioInit( &obj->Direction, dir, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF1_TIM2 );
        
        GpioInit( &obj->Tampering, tampering, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
        GpioSetInterrupt( &obj->Tampering, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[0]);
        GpioInit( &obj->Alarm, alarm, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
        GpioSetInterrupt( &obj->Alarm, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[1]);

    }

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    TimHandle.Instance = TIM2;
    TimHandle.Init.Prescaler = 1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.Period = 0xffff;
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 15;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 15;
    HAL_TIM_Encoder_Init(&TimHandle, &sConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);
    
    CRITICAL_SECTION_END( );
}

void EncoderDeInit( Encoder_t *obj )
{
    HAL_TIM_Encoder_DeInit(&TimHandle);
//     HAL_SPI_DeInit( &SpiHandle[obj->SpiId] );

//     GpioInit( &obj->Mosi, obj->Mosi.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//     GpioInit( &obj->Miso, obj->Miso.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
//     GpioInit( &obj->Sclk, obj->Sclk.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//     GpioInit( &obj->Nss, obj->Nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

// void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
// {
//     SpiHandle[obj->SpiId].Init.Direction = SPI_DIRECTION_2LINES;
//     if( bits == SPI_DATASIZE_8BIT )
//     {
//         SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_8BIT;
//     }
//     else
//     {
//         SpiHandle[obj->SpiId].Init.DataSize = SPI_DATASIZE_16BIT;
//     }
//     SpiHandle[obj->SpiId].Init.CLKPolarity = cpol;
//     SpiHandle[obj->SpiId].Init.CLKPhase = cpha;
//     SpiHandle[obj->SpiId].Init.FirstBit = SPI_FIRSTBIT_MSB;
//     SpiHandle[obj->SpiId].Init.TIMode = SPI_TIMODE_DISABLE;
//     SpiHandle[obj->SpiId].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//     SpiHandle[obj->SpiId].Init.CRCPolynomial = 7;

//     if( slave == 0 )
//     {
//         SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_MASTER;
//     }
//     else
//     {
//         SpiHandle[obj->SpiId].Init.Mode = SPI_MODE_SLAVE;
//     }
// }

void OnTamperingIrq( void* context ){
    __NOP();
}

void OnAlarmIrq( void* context ){
    __NOP();
}
