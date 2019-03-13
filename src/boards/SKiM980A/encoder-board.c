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
// #include "timer.h"

EncoderIrqHandler *EncoderIrq[] = { OnTamperingIrq , OnAlarmIrq, NULL};

Encoder_t Encoder;
flow_t flow;
static TIM_HandleTypeDef TimHandle;
/*!
 * Timer to handle the application data transmission duty cycle
 */
// static TimerEvent_t StorePacketTimer;

void EncoderInit( Encoder_t *obj, EncoderId_t timId, PinNames pulse, PinNames dir, PinNames tampering, PinNames alarm)
{
    CRITICAL_SECTION_BEGIN( );

     obj->EncoderId = timId;

     if( timId == TIM_2 )
     {
        __HAL_RCC_TIM2_CLK_ENABLE();

        TimHandle.Instance = ( TIM_TypeDef* )TIM2_BASE;

        GpioInit( &obj->Pulse, pulse, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );
        GpioInit( &obj->Direction, dir, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );

        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        TimHandle.Instance = TIM2;
        TimHandle.Init.Prescaler = 1;
        TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        TimHandle.Init.Period = 0xffff;
        TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        HAL_TIM_Encoder_Init(&TimHandle, &sConfig);

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

        HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);

        GpioInit( &obj->Tampering, tampering, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
        GpioSetInterrupt( &obj->Tampering, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[0]);
        // GpioSetContext(&obj->Tampering, &obj);
        GpioInit( &obj->Alarm, alarm, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
        GpioSetInterrupt( &obj->Alarm, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[1]);
        // GpioSetContext(&obj->Alarm, &obj);
       
        OnTamperingIrq(&obj);
        OnAlarmIrq(&obj);

        // TimerInit( &StorePacketTimer, OnStorePacketTimerEvent );
        // TimerSetValue( &StorePacketTimer, 60000 );
    }

    CRITICAL_SECTION_END( );
}

void EncoderDeInit( Encoder_t *obj )
{
    HAL_TIM_Encoder_DeInit(&TimHandle);
}

void OnTamperingIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
    if (GpioRead(&Encoder.Tampering) == GPIO_PIN_RESET) {
		flow.status |= 0x1;
		HAL_TIM_Encoder_Start_IT(&TimHandle, TIM_CHANNEL_1);

	} else {
		flow.status &= ~0x1;
		HAL_TIM_Encoder_Stop_IT(&TimHandle, TIM_CHANNEL_1);
	}
    __NOP();
    CRITICAL_SECTION_END();
}

void OnAlarmIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
    if (GpioRead(&Encoder.Alarm) == GPIO_PIN_RESET) {
		flow.status |= 0x2;
	} else {
		flow.status &= ~0x2;
	}
    __NOP();
    CRITICAL_SECTION_END();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	CRITICAL_SECTION_BEGIN();
	if (htim->Instance == TIM2) {

		if (GpioRead(&Encoder.Direction) != GPIO_PIN_RESET)
		{
            flow.status |= 0x4;
		 	flow.fwd_cnt++;
		// 	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		// 	memset(buffer, 0x0, sizeof buffer); sprintf(buffer, "FWD:%ld", flow.fwd_cnt);
		// 	BSP_LCD_ClearStringLine(1); BSP_LCD_DisplayStringAtLine(1, (uint8_t*) buffer);
		} else {
            flow.status &= ~0x4;
		 	flow.rev_cnt++;
		// 	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		// 	memset(buffer, 0x0, sizeof buffer); sprintf(buffer, "REV:%ld", flow.rev_cnt);
		// 	BSP_LCD_ClearStringLine(2); BSP_LCD_DisplayStringAtLine(2, (uint8_t*) buffer);
		}
		// uint32_t current = HAL_GetTick();
		// float rt = (1.0f / (float)(current - flow.last)) * 1000.0f;
		// flow.rate = (int16_t) rt;
		// memset(buffer, 0x0, sizeof buffer); sprintf(buffer, "RATE:%d.%d", flow.rate, (int16_t)((rt -flow.rate) * 100.f));
		// BSP_LCD_ClearStringLine(3); BSP_LCD_DisplayStringAtLine(3, (uint8_t*) buffer);
		// flow.cnt = htim->Instance->CNT;
		// flow.last = HAL_GetTick();
        __NOP();
	}
	
	CRITICAL_SECTION_END();
}

void TIM2_IRQHandler( void )
{
    HAL_TIM_IRQHandler( &TimHandle );
}

/*!
 * \brief Function executed on Led 4 Timeout event
 */
static void OnStorePacketTimerEvent( void* context )
{
    // TimerStop( &StorePacketTimer );
    // NvmCtxMgmtStore();
}
