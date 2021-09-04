/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
 *
 * \author    Andreas Pella ( IMST GmbH )
 */

// clang-format off
#include "stm32l1xx.h"
#include "utilities.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "sysIrqHandlers.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "sx1272-board.h"
#include "board.h"
#if ( USE_GPS == 1 )
#include "gps.h"
#endif
#if ( USE_OLED == 1 )
#include <stdlib.h>
#include <string.h>
#include "systime.h"
#include "display-board.h"
#include "fonts.h"

PAINT_TIME sPaint_time = {
    .Year  = 2021, /* 0000   */
    .Month = 1,    /* 1 - 12 */
    .Day   = 30,   /* 1 - 30 */
    .Hour  = 23,   /* 0 - 23 */
    .Min   = 59,   /* 0 - 59 */
    .Sec   = 59    /* 0 - 59 */
};
PAINT_GPS sPaint_gps = {
    .lat = 13.797923575153222 * 1000,
    .lon = 100.60764708923249 * 1000,
    .alt = 21,
    .fix = true
};
PAINT_LoRa sPaint_lora = {
    .pwr    = 16,
    .ulFcnt = 0,
    .class  = 'A',
    .dlFcnt = 0,
    .rssi   = 0,
    .lsnr   = 0,
    .dr     = 2, /* DR5 ~ DR0 */
    .len    = 0, /* 1 - 254   */
    .port   = 0  /* ms        */
};
#endif

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define ID1    ( 0x1FF80050 )
#define ID2    ( 0x1FF80054 )
#define ID3    ( 0x1FF80064 )

// clang-format on
/*!
 * LED GPIO pins objects
 */
#if( USE_POTENTIOMETER == 0 && USE_GPS == 0 )
Gpio_t Led1;
#endif
#if( USE_GPS == 0 )
Gpio_t Led2;
Gpio_t Led3;
Gpio_t Led4;
#endif

/*
 * MCU objects
 */
Adc_t  Adc;
I2c_t  I2c;
Spi_t  Spi;
Uart_t Uart1;

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

// clang-format off
#if( USE_GPS == 0 )
/*!
 * UART2 FIFO buffers size
 */
#define UART1_FIFO_TX_SIZE    1024
#define UART1_FIFO_RX_SIZE    1024
uint8_t Uart1TxBuffer[UART1_FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[UART1_FIFO_RX_SIZE];
#endif

/*!
 * Factory power supply (mV)
 */
#define FACTORY_POWER_SUPPLY      3300

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE             4095U

/*!
 * VREF bandgap value (mV)
 */
#define ADC_VREF_BANDGAP          1224U

/*!
 * Battery thresholds (mV)
 */
#define BATTERY_MAX_LEVEL         3000U
#define BATTERY_MIN_LEVEL         2400U
#define BATTERY_SHUTDOWN_LEVEL    2300U

/*!
 * VREF calibration value
 */
#define VREFINT_CAL               ( *( uint16_t* ) 0x1FF800F8U )

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;
// clang-format on

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void* context )
{
    RtcSetMcuWakeUpTime( );
    SystemWakeupTimeCalibrated = true;
}

void BoardCriticalSectionBegin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t* mask )
{
    /* End critical section */
    __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{
#if( USE_OLED == 1 )
    uint32_t x = 0U;

    DisplayInit( );
    DisplayClear( );
    RtcDelayMs( 20U );
	
	// !"#$%&'()*+,-./0123456789:;<=>?\x0040ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
	//กขฃคฅฆงจฉชซฌญฎฏฐฑฒณดตถทธนบปผฝพฟภมยรฤลฦวศษสหฬอฮฯะัาำิีึืฺุู\x0e3b\x0e3c\x0e3d\x0e3e฿เแโใไๅๆ็่้๊๋์ํ๎๏๐๑๒๓๔๕๖๗๘๙๚๛
	//Paint_DrawString_TH( 0, 2, "เอ็มวัน", &Font16TH, WHITE, BLACK );
    Paint_DrawString_TH( 0, 2, "เอ็มวัน(EmOne)", &Font16TH, WHITE, BLACK );
    Paint_DrawString_TH( 2, 17, "ประเทศไทย", &Font16TH, WHITE, BLACK );
    Paint_DrawString_EN( 5, 36, "LoRaWAN Survey", &Font12, BLACK, WHITE );
    Paint_DrawString_EN( 60, 50, "Ver 0.1.1a", &Font8, BLACK, WHITE );
	DisplayUpdate( );
#if 0
    Paint_DrawString_EN( 0, 32, "EmOne", &Font20, BLACK, WHITE );
    Paint_DrawString_EN( 10, 22, "LoRaWAN Survey", &Font12, BLACK, WHITE );
    Paint_DrawString_EN( 10, 38, "Field test purpose", &Font8, BLACK, WHITE );
    Paint_DrawString_EN( 10, 48, "Ver 0.1.1a", &Font12, BLACK, WHITE );
#endif

    for( x = 1; x <= OLED_WIDTH; x += 16U )
    {
        Paint_DrawLine( 1, 62, x, 62, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID );
        RtcDelayMs( 500 );
        DisplayUpdate( );
    }
#endif
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
        HAL_Init( );

#if( USE_OLED == 0 )
#if( USE_POTENTIOMETER == 0 )
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
#endif
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
#endif
        SystemClockConfig( );

#if( USE_GPS == 0 )
        FifoInit( &Uart1.FifoTx, Uart1TxBuffer, UART1_FIFO_TX_SIZE );
        FifoInit( &Uart1.FifoRx, Uart1RxBuffer, UART1_FIFO_RX_SIZE );

        /*
         * Configure your terminal for 8 Bits data
         * (7 data bit + 1 parity bit), no parity and no flow ctrl
         */
        UartInit( &Uart1, UART_1, UART_TX, UART_RX );
        UartConfig( &Uart1, RX_TX, 921600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
#endif
        RtcInit( );

#if( USE_POTENTIOMETER == 0 && USE_GPS == 0 )
        /* Switch LED 1, 2, 3, 4 OFF */
        GpioWrite( &Led1, 0 );
#endif
#if( USE_GPS == 0 )
        GpioWrite( &Led2, 0 );
        GpioWrite( &Led3, 0 );
        GpioWrite( &Led4, 0 );
#else
        /* Init GPS */
        GpsInit( );
#endif
        BoardUnusedIoInit( );
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
#if( USE_GPS == 0 )
            /* Disables OFF mode - Enables lowest power mode (STOP) */
            LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
#else
            LpmSetStopMode( LPM_APPLI_ID, LPM_DISABLE );
#endif
        }
    }
    else
    {
        SystemClockReConfig( );
    }

#if( USE_GPS == 1 )
    /* Init GPS */
    GpsStart( );

    /* VREF */
    AdcInit( &Adc, NC );
#elif( USE_POTENTIOMETER == 1 )
    AdcInit( &Adc, POTI );
#endif

    SpiInit( &SX1272.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1272IoInit( );

    if( McuInitialized == false )
    {
        McuInitialized = true;
        SX1272IoDbgInit( );
        SX1272IoTcxoInit( );
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            CalibrateSystemWakeupTime( );
        }
    }
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    /* Restart system */
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    Gpio_t ioPin;

#if( USE_GPS == 1 )
    /* DeInit GPS */
    GpsStop( );
#endif
    AdcDeInit( &Adc );

    SpiDeInit( &SX1272.Spi );
    SX1272IoDeInit( );

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* ) ID1 ) ^ ( *( uint32_t* ) ID2 ) ^ ( *( uint32_t* ) ID3 ) );
}

void BoardGetUniqueId( uint8_t* id )
{
    id[7] = ( ( *( uint32_t* ) ID1 ) + ( *( uint32_t* ) ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* ) ID1 ) + ( *( uint32_t* ) ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* ) ID1 ) + ( *( uint32_t* ) ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* ) ID1 ) + ( *( uint32_t* ) ID3 ) );
    id[3] = ( ( *( uint32_t* ) ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* ) ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* ) ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* ) ID2 ) );
}

#if( USE_OLED == 1 )
void BoardDisplayShow( void )
{
    // CRITICAL_SECTION_BEGIN( );
    // DisplayInitReg();
    // DisplayOn();
    //
    Paint_Clear( BLACK );
    DisplayUpdate( );

    char buf[24];

#if( USE_GPS == 1 )
    double    lt, ln;
    struct tm localtime;
    SysTime_t curTime = { .Seconds = 0, .SubSeconds = 0 };
    curTime           = SysTimeGet( );
    sPaint_gps.alt    = GpsGetLatestGpsAltitude( );
    sPaint_gps.fix    = GpsHasFix( );

    /* Paint LoRa information */
    if( sPaint_gps.fix )
    {
        strncpy( buf, NmeaGpsData.NmeaUtcTime, 2 );
        localtime.tm_hour = sPaint_time.Hour = atoi( buf );
        strncpy( buf, NmeaGpsData.NmeaUtcTime + 2, 2 );
        localtime.tm_min = sPaint_time.Min = atoi( buf );
        strncpy( buf, NmeaGpsData.NmeaUtcTime + 4, 2 );
        localtime.tm_sec = sPaint_time.Sec = atoi( buf );

        strncpy( buf, NmeaGpsData.NmeaDate, 2 );
        localtime.tm_mday = sPaint_time.Day = atoi( buf );
        strncpy( buf, NmeaGpsData.NmeaDate + 2, 2 );
        localtime.tm_mon = sPaint_time.Month = atoi( buf );
        strncpy( buf, NmeaGpsData.NmeaDate + 4, 2 );
        sPaint_time.Year  = atoi( buf );
        localtime.tm_year = ( 2000 + sPaint_time.Year ) - 1900;
        curTime.Seconds   = SysTimeMkTime( &localtime );
        SysTimeSet( curTime );
    }
    else
    {
        SysTimeLocalTime( curTime.Seconds, &localtime );
        sPaint_time.Hour  = localtime.tm_hour;
        sPaint_time.Min   = localtime.tm_min;
        sPaint_time.Sec   = localtime.tm_sec;
        sPaint_time.Day   = localtime.tm_mday;
        sPaint_time.Month = localtime.tm_mon;
        sPaint_time.Year  = ( localtime.tm_year + 1900 ) % 100;
    }

    GpsGetLatestGpsPositionDouble( &lt, &ln );
    sPaint_gps.lat = ( int32_t )( lt * 100000 );
    sPaint_gps.lon = ( int32_t )( ln * 100000 );

    /* gps infomation */
    sprintf( buf, "LT:%d.%d LN:%d.%d", ( int ) ( sPaint_gps.lat / 100000 ), ( int ) ( sPaint_gps.lat % 100000 ),
             ( int ) ( sPaint_gps.lon / 100000 ), ( int ) ( sPaint_gps.lon % 100000 ) );
    Paint_DrawString_EN( 5, 1, buf, &Font8, BLACK, WHITE );
    sprintf( buf, "AT:%d FX:%d BT:%3d%%", sPaint_gps.alt, sPaint_gps.fix, BoardGetBatteryLevel( ) * 100 / 254 );
    Paint_DrawString_EN( 5, 1 + Font8.Height, buf, &Font8, BLACK, WHITE );
#endif

    sprintf( buf, "UL->PW:%d CL:%c | DR:%d", ( 16 - sPaint_lora.pwr ), sPaint_lora.class, sPaint_lora.dr );
    Paint_DrawString_EN( 5, 20, buf, &Font8, BLACK, WHITE );
    sprintf( buf, "DL->RS:%d LS:%d | PT:%d", sPaint_lora.rssi, sPaint_lora.lsnr, sPaint_lora.port );
    Paint_DrawString_EN( 5, 20 + Font8.Height, buf, &Font8, BLACK, WHITE );
    sprintf( buf, "UC:%8lu DC:%8lu", sPaint_lora.ulFcnt, sPaint_lora.dlFcnt );
    Paint_DrawString_EN( 5, 20 + Font8.Height * 2, buf, &Font8, BLACK, WHITE );
    if( sPaint_lora.status != NULL )
    {
        sprintf( buf, "S:%s", sPaint_lora.status );
        Paint_DrawString_EN( 5, 20 + Font8.Height * 3, buf, &Font8, BLACK, WHITE );
    }

    /* local time utc */
    sprintf( buf, "%02d:%02d:%02dUTC-%02d/%02d/%02d", sPaint_time.Hour, sPaint_time.Min, sPaint_time.Sec,
             sPaint_time.Day, sPaint_time.Month, ( int ) sPaint_time.Year );
    Paint_DrawString_EN( 20, 52, buf, &Font8, BLACK, WHITE );

    DisplayUpdate( );

    // CRITICAL_SECTION_END( );
}
#endif

/*!
 * Potentiometer max and min levels definition
 */
#define POTI_MAX_LEVEL 900
#define POTI_MIN_LEVEL 10

uint8_t BoardGetPotiLevel( void )
{
    uint8_t  potiLevel = 0;
    uint16_t vpoti     = 0;
#if( USE_GPS == 1 )
    vpoti = AdcReadChannel( &Adc, ADC_CHANNEL_TEMPSENSOR );
#else
    /* Read the current potentiometer setting */
    vpoti = AdcReadChannel( &Adc, ADC_CHANNEL_3 );
#endif
    /* check the limits */
    if( vpoti >= POTI_MAX_LEVEL )
    {
        potiLevel = 100;
    }
    else if( vpoti <= POTI_MIN_LEVEL )
    {
        potiLevel = 0;
    }
    else
    {
        /* if the value is in the area, calculate the percentage value */
        potiLevel = ( ( vpoti - POTI_MIN_LEVEL ) * 100 ) / POTI_MAX_LEVEL;
    }
    return potiLevel;
}

uint16_t BoardBatteryMeasureVoltage( void )
{
    uint16_t vref           = 0U;
    uint32_t batteryVoltage = 0U;

    /* Read the current Voltage */
    vref = AdcReadChannel( &Adc, ADC_CHANNEL_17 );

    /*
     * We don't use the VREF from calibValues here.
     * calculate the Voltage in millivolt
     */
    batteryVoltage = ( uint32_t ) ADC_VREF_BANDGAP * ( uint32_t ) ADC_MAX_VALUE;
    batteryVoltage = batteryVoltage / ( uint32_t ) vref;

    return batteryVoltage;
}

uint32_t BoardGetBatteryVoltage( void )
{
    /* return battery voltage */
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0U;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    if( GetBoardPowerSource( ) == USB_POWER )
    {
        batteryLevel = 0;
    }
    else
    {
        if( BatteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = 254;
        }
        else if( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel =
                ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
        {
            batteryLevel = 1;
        }
        else  // if( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = 255;
        }
    }
    return batteryLevel;
}

static void BoardUnusedIoInit( void )
{
#if( USE_OLED == 0 )
    Gpio_t ioPin;

    if( GetBoardPowerSource( ) == BATTERY_POWER )
    {
        GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
#endif

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

    GpioInit( &ioPin, JTAG_TMS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_NRST, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV     = RCC_PLL_DIV3;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {
            /* Do nothing */
        }
    }
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
        /* Do nothing */
    }

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
        /* Do nothing */
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG( RCC_SYSCLKSOURCE_PLLCLK );

    /* Wait till PLL is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
        /* Do nothing */
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

uint8_t GetBoardPowerSource( void )
{
    /* get board power */
    return BATTERY_POWER;
}

/**
 * \brief Enters Low Power Stop Mode
 *
 * \note ARM exists the function when waking up
 */
void LpmEnterStopMode( void )
{
    CRITICAL_SECTION_BEGIN( );

    BoardDeInitMcu( );

    /* Disable the Power Voltage Detector */
    HAL_PWR_DisablePVD( );

    /* Clear wake up flag */
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    /* Enable Ultra low power mode */
    HAL_PWREx_EnableUltraLowPower( );

    /* Enable the fast wake up from Ultra low power mode */
    HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    /* Enter Stop Mode */
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    /* Disable IRQ while the MCU is not running on HSI */
    CRITICAL_SECTION_BEGIN( );

    /* Initilizes the peripherals */
    BoardInitMcu( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void )
{
    /* Enter to sleep mode */
    HAL_PWR_EnterSLEEPMode( PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI );
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
}

#if !defined( __CC_ARM )

#if( USE_GPS == 0 )
/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void* buf, size_t count )
{
    while( UartPutBuffer( &Uart1, ( uint8_t* ) buf, ( uint16_t ) count ) != 0 )
    {
    };

    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void* buf, size_t count )
{
    size_t bytesRead = 0;
    while( UartGetBuffer( &Uart1, ( uint8_t* ) buf, count, ( uint16_t* ) &bytesRead ) != 0 )
    {
    };

    /* Echo back the character */
    while( UartPutBuffer( &Uart1, ( uint8_t* ) buf, ( uint16_t ) bytesRead ) != 0 )
    {
    };

    return bytesRead;
}
#else
int _write( int c )
{
    return -1;
}
#endif /* USE_GPS */
#else
#include <stdio.h>

/* Keil compiler */
int fputc( int c, FILE* stream )
{
    while( UartPutChar( &Uart1, ( uint8_t ) c ) != 0 )
        ;

    return c;
}

int fgetc( FILE* stream )
{
    uint8_t c = 0;
    while( UartGetChar( &Uart1, &c ) != 0 )
        ;

    /* Echo back the character */
    while( UartPutChar( &Uart1, c ) != 0 )
        ;

    return ( int ) c;
}
#endif

#ifdef USE_FULL_ASSERT
#include <stdio.h>
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /*
     * User can add his own implementation to report
     * the file name and line number,
     * ex: printf("Wrong parameters
     * value: file %s on line %lu\n", file, line)
     */
    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* ) file, line );

    /* Infinite loop */
    while( 1 )
    {
        /* Do nothing */
    }
}
#endif
