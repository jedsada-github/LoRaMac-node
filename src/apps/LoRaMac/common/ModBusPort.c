#include "ModBus.h"
#include "ModBusPort.h"
#include "timer.h"

extern Uart_t Uart1;
static TimerEvent_t ModbusPacketTimer;
SLAVE_T modbus_slave;

unsigned char ReceiveBuffer[RECEIVE_BUFFER_SIZE];
unsigned char ReceiveCounter=0;
unsigned char TimerInitFlag =0;

unsigned char MasterReceiveBuffer[MASTER_RECEIVE_BUFFER_SIZE];
unsigned char MasterReceiveCounter=0;

void ModBusSlave_UART_Initialise(void)
{
//    General_USART_Init();
    Uart1.IrqNotify = MODBUS_RX_IRQHandler;
}

void ModBusSlave_TIMER_Initialise(void)
{
    if(TimerInitFlag==0)
    {
        TimerInitFlag =1;
//        General_Timer_Init();
    }
}

void ModBusSlave_UART_Putch(unsigned char c)
{
	//Wait for the uart to finish sending the byte.
	// while(HAL_UART_STATE_BUSY_TX_RX == HAL_UART_GetState(&huart1));
//	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	// HAL_UART_Transmit(&huart1, &c, 1, 500);
//	USART_SendData(USART2,c);
    UartPutChar(&Uart1, c);
}

unsigned char ModBusSlave_UART_String(unsigned char *s, unsigned int Length)
{
  while(Length)
	{
		ModBusSlave_UART_Putch(*s++);
		Length--;
	}
	
	return TRUE;
}

/*******************************For Master*************************************/
void OnReadHoldingRegisters (void* context) {
    TimerStop(&ModbusPacketTimer);
    MasterReadTimerValue += 1000;
    
    if(ModBusMasterRead(1, 1, 40001, 10, 5000) == TRUE) {
        
    }
}

void ModBusMaster_UART_Initialise(void)
{
    //InitUART2();
    Uart1.IrqNotify = MODBUS_RX_IRQHandler;
}

void ModBusMaster_TIMER_Initialise(void)
{
   if(TimerInitFlag == 0)
   {
       TimerInitFlag = 1;
    //    InitTMR1();
        TimerInit( &ModbusPacketTimer, OnReadHoldingRegisters );
        TimerSetValue(&ModbusPacketTimer, TIMEOUTTIMER);
   }
}

void ModBusMaster_UART_Putch(unsigned char c)
{
    //U2TXREG=c;
        //while(U2STAbits.UTXBF);   // Gonderim tamamlandi mi
    UartPutChar(&Uart1, c);
}

unsigned char ModBusMaster_UART_String(unsigned char *s, unsigned int Length)
{
    while(Length)
    {
        ModBusMaster_UART_Putch(*s++);
        Length--;
    }

    return TRUE;
}

/***************************Interrupt For Slave********************************/
void MODBUS_RX_IRQHandler(UartNotifyId_t id)
{
    uint8_t tmp;
    if (id == UART_NOTIFY_RX)
    {
        if (UartGetChar(&Uart1, &tmp) == 0)
        {
            ReceiveBuffer[ReceiveCounter]   = (unsigned char)  tmp;
            ReceiveCounter++;
        }
        if(ReceiveCounter>RECEIVE_BUFFER_SIZE)
            ReceiveCounter=0;
        
        SlaveTimerValue=0;
        MasterReadTimerValue=0;
        MasterWriteTimerValue=0;

    }
//	if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)
//	{
//		//USART_ClearFlag(USART2, USART_IT_RXNE);

	// ReceiveBuffer[ReceiveCounter]   = (unsigned char)  tmp;
    // ReceiveCounter++;

    // if(ReceiveCounter>RECEIVE_BUFFER_SIZE)
    //     ReceiveCounter=0;

    // SlaveTimerValue=0;
//	}
}

/**************************Interrupts For Master********************************/
//void __attribute__((interrupt, , auto_psv)) _U2RXInterrupt( void )
//{
//    IFS1bits.U2RXIF = 0;                                                        // UART alim kesme bayragi temizleniyor

//    MasterReceiveBuffer[MasterReceiveCounter]   =U2RXREG;
//    MasterReceiveCounter++;

//    if(MasterReceiveCounter>MASTER_RECEIVE_BUFFER_SIZE)
//        MasterReceiveCounter=0;

//    MasterReadTimerValue=0;
//    MasterWriteTimerValue=0;
//}

/******************************************************************************/



