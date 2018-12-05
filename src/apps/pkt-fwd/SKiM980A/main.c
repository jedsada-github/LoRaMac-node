/*!
 * \file      main.c
 *
 * \brief     Single Packet Forward implementation
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
 *              (C)2015-2018 EmOne
 *
 * \endcode
 *
 * \author    Anol Paisal ( EmOne )
 *
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "utilities.h"
#include "uart-board.h"

#if defined( REGION_AS923 )

#define RF_FREQUENCY                                923200000 // Hz

#elif defined( REGION_AU915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_CN779 )

#define RF_FREQUENCY                                779000000 // Hz

#elif defined( REGION_EU868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( REGION_KR920 )

#define RF_FREQUENCY                                920000000 // Hz

#elif defined( REGION_IN865 )

#define RF_FREQUENCY                                865000000 // Hz

#elif defined( REGION_US915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( REGION_RU864 )

#define RF_FREQUENCY                                864000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 255 // Define the payload size here

extern Uart_t Uart1;

uint16_t UpBufferSize = BUFFER_SIZE;
uint8_t UpBuffer[BUFFER_SIZE] = { 0 };
uint16_t DnBufferSize = BUFFER_SIZE;
uint8_t DnBuffer[BUFFER_SIZE] = { 0 };

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led3;
extern Gpio_t Led4;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
int PacketValidate(uint8_t * data, uint16_t* len);

/**
 * Main application entry point.
 */
int main( void )
{
    uint16_t i = 0;
    uint16_t nbReadByte = 0;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    UartPutBuffer(&Uart1, (uint8_t *) "Hello LoRa\r\n", 12);
    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx( RX_TIMEOUT_VALUE );

    while( 1 )
    {
        switch( State )
        {
        case RX:
            //TODO: forward payload to serial with statistic
            UartPutBuffer(&Uart1, UpBuffer, UpBufferSize);
            memset(UpBuffer, 0, UpBufferSize);

            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a Downlink
            GpioToggle( &Led3 );
            // TODO: Downlink ack
            UartPutBuffer(&Uart1, (uint8_t *) "ACK\r\n", 5);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
             // TODO: Downlink nack
             UartPutBuffer(&Uart1, (uint8_t *) "NACK\r\n", 6);
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            if(Radio.GetStatus() == RF_IDLE)
                Radio.Rx( RX_TIMEOUT_VALUE );
            // Set low power
            break;
        }

        //TODO: Get JIT Queue downlink from serial
        while (UartGetBuffer(&Uart1, DnBuffer + i, DnBufferSize, &nbReadByte) == 0) {
            i += nbReadByte;
        }
        if(nbReadByte > 0) {
            //TODO: Downlink packet validation
            if(PacketValidate(DnBuffer, &nbReadByte) == 0 ) {
                DelayMs( 1 );
                Radio.Send(DnBuffer, nbReadByte);
            }
            memset(DnBuffer, 0, sizeof DnBuffer);
            i = nbReadByte = 0;
        }

        // BoardLowPowerHandler( );

    }
}

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    UpBufferSize = size + 5;
    memcpy( UpBuffer + 5, payload, UpBufferSize );
    UpBuffer[0] = 0x1;
    UpBuffer[3] = RssiValue = rssi;
    UpBuffer[4] = SnrValue = snr;
    UpBuffer[UpBufferSize++] = '\r';
    UpBuffer[UpBufferSize++] = '\n';
    UpBuffer[1] = (uint8_t) (UpBufferSize >> 8) & 0xff;
    UpBuffer[2] = (uint8_t) (UpBufferSize) & 0xff;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}

int PacketValidate(uint8_t * data, uint16_t* len) {
    return 0;
}