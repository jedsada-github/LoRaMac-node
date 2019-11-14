/*!
 * \file      wdt-board.h
 *
 * \brief     WDT driver implementation
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
#ifndef __WDT_H__
#define __WDT_H__

/*!
 * WDT peripheral ID
 */
typedef enum
{
    WDT_WWDG,
    WDT_IWDG, 
}WdtId_t;

/*!
 * WDT object type definition
 */
typedef struct Wdt_s
{
    WdtId_t WdtId;
}Wdt_t;

/*!
 * \brief Initializes the WDT object and MCU peripheral
 *
 * \param [IN] obj  WDT object
 */
void WdtInit( Wdt_t *obj, WdtId_t wdtId);

/*!
 * \brief Refresh the WDT object and MCU peripheral
 *
 * \param [IN] obj WDT object
 */
void WdtRefresh( Wdt_t *obj );

#endif // __SPI_H__
