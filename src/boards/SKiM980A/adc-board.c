/*!
 * \file      adc-board.c
 *
 * \brief     Target board ADC driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l1xx.h"
#include "board-config.h"
#include "adc-board.h"
#include "board.h"
#include "encoder.h"

ADC_HandleTypeDef AdcHandle;
ADC_AnalogWDGConfTypeDef awd;

void AdcMcuInit( Adc_t *obj, PinNames adcInput )
{
    AdcHandle.Instance = ( ADC_TypeDef* )ADC1_BASE;

    __HAL_RCC_ADC1_CLK_ENABLE( );

    HAL_ADC_DeInit( &AdcHandle );

    if( adcInput != NC )
    {
        GpioInit( &obj->AdcInput, adcInput, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
}

void AdcMcuDeInit( Adc_t *obj )
{
    AdcHandle.Instance = ( ADC_TypeDef* )ADC1_BASE;

    __HAL_RCC_ADC1_CLK_DISABLE( );

    HAL_ADC_DeInit( &AdcHandle );
}

void AdcMcuConfig( void )
{
    // Configure ADC
#if (USE_ENCODER == 1)
    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_10B;
#else
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
#endif
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    HAL_ADC_Init( &AdcHandle );
}

uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel )
{
    ADC_ChannelConfTypeDef adcConf = { 0 };
    uint16_t adcData = 0;

    // Enable HSI
    __HAL_RCC_HSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    __HAL_RCC_ADC1_CLK_ENABLE( );

    adcConf.Channel = channel;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;

    HAL_ADC_ConfigChannel( &AdcHandle, &adcConf );

    // Enable ADC1
    if( ADC_Enable( &AdcHandle ) == HAL_OK )
    {
        // Start ADC Software Conversion
        HAL_ADC_Start( &AdcHandle );

        HAL_ADC_PollForConversion( &AdcHandle, HAL_MAX_DELAY );

        adcData = HAL_ADC_GetValue( &AdcHandle );
    }

    ADC_ConversionStop_Disable( &AdcHandle );

    if( ( adcConf.Channel == ADC_CHANNEL_TEMPSENSOR ) || ( adcConf.Channel == ADC_CHANNEL_VREFINT ) )
    {
        HAL_ADC_DeInit( &AdcHandle );
    }
    __HAL_RCC_ADC1_CLK_DISABLE( );

    // Disable HSI
    __HAL_RCC_HSI_DISABLE( );

    return adcData;
}

void AdcMcuWatchdog( Adc_t *obj, uint32_t channel, uint32_t high_lvl, uint32_t low_lvl)
{
    awd.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;      /*!< Configures the ADC analog watchdog mode: single/all channels, regular/injected group.
                                   This parameter can be a value of @ref ADC_analog_watchdog_mode. */
    awd.Channel = channel;           /*!< Selects which ADC channel to monitor by analog watchdog.
                                   This parameter has an effect only if watchdog mode is configured on single channel (parameter WatchdogMode)
                                   This parameter can be a value of @ref ADC_channels. */
    awd.ITMode = ENABLE;            /*!< Specifies whether the analog watchdog is configured in interrupt or polling mode.
                                   This parameter can be set to ENABLE or DISABLE */
    awd.HighThreshold = high_lvl;     /*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */
    awd.LowThreshold = low_lvl;      /*!< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF. */
    awd.WatchdogNumber = 0;    /*!< Reserved for future use, can be set to 0 */

    HAL_ADC_AnalogWDGConfig(&AdcHandle, &awd);
}

/**
  * @brief  Analog watchdog callback in non blocking mode. 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    CRITICAL_SECTION_BEGIN();
    if (Encoder.OnSendOneshot != NULL &&  config.analog_alarm > 0)
    {
        awd.ITMode = DISABLE; 
        HAL_ADC_AnalogWDGConfig(&AdcHandle, &awd);
        Encoder.OnSendOneshot();
    }
    CRITICAL_SECTION_END();  
}
