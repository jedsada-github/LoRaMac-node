	#ifndef __MODBUSPORT__H
#define __MODBUSPORT__H

#include "ModBus.h"
#include "uart.h"

extern unsigned char ReceiveBuffer[RECEIVE_BUFFER_SIZE];
extern unsigned char ReceiveCounter;

extern unsigned char MasterReceiveBuffer[MASTER_RECEIVE_BUFFER_SIZE];
extern unsigned char MasterReceiveCounter;

extern void ModBusSlave_UART_Initialise(void);
extern void ModBusSlave_TIMER_Initialise(void);
extern void ModBusSlave_UART_Putch(unsigned char c);
extern unsigned char ModBusSlave_UART_String(unsigned char *s, unsigned int Length);

extern void ModBusMaster_UART_Initialise(void);
extern void ModBusMaster_TIMER_Initialise(void);
extern void ModBusMaster_UART_Putch(unsigned char c);
extern unsigned char ModBusMaster_UART_String(unsigned char *s, unsigned int Length);

void MODBUS_RX_IRQHandler(UartNotifyId_t id);

typedef struct {
	uint8_t address;
	uint8_t function; 
	uint16_t start_address;
	uint16_t numberofdata; 
	void * data;
} SLAVE_T;

extern SLAVE_T modbus_slave;

#endif
