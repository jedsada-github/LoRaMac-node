/*!
 * \file      adc-board.h
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
#ifndef __ADC_BOARD_H__
#define __ADC_BOARD_H__

#include "adc.h"

/*!
 * \brief Initializes the ADC object and MCU peripheral
 *
 * \param [IN] obj      ADC object
 * \param [IN] adcInput ADC input pin
 */
void AdcMcuInit( Adc_t *obj, PinNames adcInput );

/*!
 * \brief DeInitializes the ADC object and MCU peripheral
 *
 * \param [IN] obj      ADC object
 */
void AdcMcuDeInit( Adc_t *obj );

/*!
 * \brief Initializes the ADC internal parameters
 */
void AdcMcuConfig( void );

/*!
 * \brief Reads the value of the given channel
 *
 * \param [IN] obj     ADC object
 * \param [IN] channel ADC input channel
 */
uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel );

/*!
 * \brief Set AWD the value of the given channel
 *
 * \param [IN] obj     ADC object
 * \param [IN] channel ADC input channel
 * \param [IN] high_lvl ADC high level threshold
 * \param [IN] low_lvl ADC low level threshold
 */
void AdcMcuWatchdog( Adc_t *obj, uint32_t channel, uint32_t high_lvl, uint32_t low_lvl);

#endif // __ADC_BOARD_H__
