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

#define RF_FREQUENCY 923000000 // Hz

#define TX_OUTPUT_POWER 20 // dBm

#define LORA_BANDWIDTH 0         // [0: 125 kHz, 
                                 //  1: 250 kHz, 
                                 //  2: 500 kHz, 
                                 //  3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 1        // [1: 4/5, 
                                 //  2: 4/6, 
                                 //  3: 4/7, 
                                 //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8   // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT

} States_t;

typedef enum
{
    DOWNLINK,
    FREQ,
    SF,
    PWR,
    PINGPONG,
} Request_t;

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 255 // Define the payload size here

extern Uart_t Uart1;

uint16_t UpBufferSize = 0;
uint8_t UpBuffer[BUFFER_SIZE] = {0};
uint16_t DnBufferSize = 0;
uint8_t DnBuffer[BUFFER_SIZE] = {0};

volatile States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;
uint32_t OnAirValue = 0;
volatile bool REQ_TX = false;
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
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/*!
 * \brief Function executed on validate msg
 */
int PacketValidate(uint8_t *data, uint16_t *len);

/*!
 * \brief Function executed on Uart receive msg irq
 */
void OnUartRx(UartNotifyId_t id);

/*!
 * \brief Function executed on Uart receive msg ping
 */
void OnPing(void);

/**
 * Main application entry point.
 */
int main(void)
{
    // uint16_t i = 0;
    // uint16_t nbReadByte = 0;

    // Target board initialization
    BoardInitMcu();
    BoardInitPeriph();

    Uart1.IrqNotify = OnUartRx;
    UartPutBuffer(&Uart1, (uint8_t *)"Hello LoRa\r\n", 12);

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);
    
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetPublicNetwork(false);

    Radio.Rx(RX_TIMEOUT_VALUE);
    
    while (1)
    {
        switch (State)
        {
        case RX:
            //TODO: forward payload to serial
            UartPutBuffer(&Uart1, UpBuffer, UpBufferSize);
            memset(UpBuffer, 0, UpBufferSize);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a Downlink
            // GpioToggle( &Led3 );
            UartPutBuffer(&Uart1, (uint8_t *) "ACK\r\n", 5);
            State = LOWPOWER;
            break;
            // TODO: Downlink ack response
        case TX_TIMEOUT: // TODO: Downlink nack response
            UartPutBuffer(&Uart1, (uint8_t *) "NACK\r\n", 6);
            State = LOWPOWER;
            break;
        
        case LOWPOWER:
            DelayMs( 1 );
            if(REQ_TX) { // TODO: Downlink nack response
                State = LOWPOWER;
                REQ_TX = false;
                // OnAirValue = Radio.TimeOnAir(MODEM_LORA, DnBufferSize);
                UartPutBuffer(&Uart1, (uint8_t *) "Sending\r\n", 9);
                Radio.Send(DnBuffer, DnBufferSize);
                memset(DnBuffer, 0, sizeof DnBuffer);
                DnBufferSize = 0;
                
            } else 
                Radio.Rx(RX_TIMEOUT_VALUE);
            break;
        default:
            // Set low power
            break;
        }

        // BoardLowPowerHandler( );
    }
}

void OnTxDone(void)
{
    Radio.Sleep();
    
    State = TX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    
    State = TX_TIMEOUT;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();
#if 0
    UpBufferSize = size + 8;
    memcpy( UpBuffer + 8, payload, UpBufferSize );
    UpBuffer[0] = 0x1; //Start of frame
    UpBuffer[1] = RX;
    UpBuffer[4] = RssiValue = rssi;
    UpBuffer[5] = SnrValue = snr;
    UpBuffer[6] = 0; //Reserved
    UpBuffer[7] = 0; //Reserved
    UpBuffer[UpBufferSize++] = '\r';    //Delimiter
    UpBuffer[UpBufferSize++] = '\n';    //Delimiter
    UpBuffer[2] = (uint8_t) (UpBufferSize >> 8) & 0xff;
    UpBuffer[3] = (uint8_t) (UpBufferSize) & 0xff;
    //TODO : CRC neccessary?
#else
    UpBufferSize = size;
    memcpy(UpBuffer, payload, UpBufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    UpBuffer[++UpBufferSize] = '\r'; //Delimiter
    UpBuffer[++UpBufferSize] = '\n'; //Delimiter
#endif
    State = RX;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep();
    State = RX_ERROR;
}

// int PacketValidate(uint8_t * data, uint16_t* len) {
//     uint16_t count;

//     if (data[0] == 0x1) {
//         count = (uint16_t) data[2] << 8 | (uint16_t) data[3];
//         if (count == *len && memcmp(data, "\r\n", *len)) {

//             switch (data[1])
//             {
//                 //TODO: Config request or down link message
//                 case FREQ: break;
//                 case SF: break;
//                 case PWR: break;
//                 case PINGPONG: break;
//                     State = PING;
//                     return 1;
//                 case DOWNLINK:
//                 default:
//                     count = *len - 10;
//                     memcpy(data, data + 8, count);
//                     *len = count;
//                     break;
//             }

//         }
//         else {
//             return 1;
//         }
//     }
//     else {
//         return 1;
//     }

//     return 0;
// }

// static bool SOF = false;

void OnUartRx(UartNotifyId_t id)
{
    uint8_t tmp;
    if (id == UART_NOTIFY_RX)
    {
        if (UartGetChar(&Uart1, &tmp) == 0)
        {
#if 0
            if (SOF == false) {
                if (tmp == 0x1) { //found start of frame
                    SOF = true;
                    DnBuffer[0] = tmp;
                    DnBufferSize = 1;
                }
            } else {
                DnBuffer[DnBufferSize++] = tmp;
                if(memchr(DnBuffer, 0xa, DnBufferSize) != NULL){
                    if(PacketValidate(DnBuffer, &DnBufferSize) == 0) {
                        DelayMs( 1 );
                        OnAirValue = Radio.TimeOnAir(MODEM_LORA, DnBufferSize);
                        Radio.Send(DnBuffer, DnBufferSize); 
                        memset(DnBuffer, 0, sizeof DnBuffer);
                        DnBufferSize = 0;
                        SOF = false;
                    }
                }
            }
#else
            DnBuffer[DnBufferSize++] = tmp;
            if (tmp == '\n')
            {
                REQ_TX = true;
            }

#endif
        }
    }
    else
    {
        /* code */
    }
}

// void OnPing( void )
// {
//     const char *pong = "PONG";
//     Radio.Sleep( );

//     DnBufferSize = sizeof pong + 8;
//     snprintf((char *) DnBuffer + 8, DnBufferSize,  pong);
//     DnBuffer[0] = 0x1; //Start of frame
//     DnBuffer[1] = PING;
//     DnBuffer[4] = 0;
//     DnBuffer[5] = 0;
//     DnBuffer[6] = 0;
//     DnBuffer[7] = 0;
//     DnBuffer[DnBufferSize++] = '\r';    //Delimiter
//     DnBuffer[DnBufferSize++] = '\n';    //Delimiter
//     DnBuffer[2] = (uint8_t) (DnBufferSize >> 8) & 0xff;
//     DnBuffer[3] = (uint8_t) (DnBufferSize) & 0xff;
//     //TODO : CRC neccessary?
//     State = PING;
// }