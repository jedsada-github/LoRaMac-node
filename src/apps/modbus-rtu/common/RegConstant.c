#include "ModBus.h"
#include "ModBusPort.h"

/***********************Input/Output Coils and Registers***********************/
#if MBFN_READ_COILS_ENABLED > 0
unsigned char ReadCoilOutputs       [NUMBER_COIL_BYTES];
#endif
#if ((MBFN_WRITE_SINGLE_COIL_ENABLED > 0)||(MBFN_WRITE_MULTIPLE_COILS_ENABLED > 0))
unsigned char WriteCoilOutputs      [NUMBER_COIL_BYTES];
#endif
#if MBFN_READ_DISCRETE_INPUTS_ENABLED > 0
unsigned char DiscreteInputs        [NUMBER_OF_INPUTS];
#endif
#if MBFN_READ_INPUT_REGISTERS_ENABLED > 0
RegStructure  RegisterInputs        [NUMBER_OF_INPUT_REGISTERS];
#endif
#if ((MBFN_WRITE_MULTIPLE_REGISTERS_ENABLED > 0)|| (MBFN_WRITE_SINGLE_REGISTER_ENABLED > 0) || (MBFN_READ_HOLDING_REGISTERS_ENABLED > 0))
RegStructure  RegisterOutputs       [NUMBER_OF_OUTPUT_REGISTERS]={
/* ActValue - MaxValue - Default - MinValue -R/W    */
      0,      99,        1,        0,      1,    // 00   - Address Modbus
      0,      4,        1,        0,      1,    // 01   - baud rate RS232 #1 (2-3) baud rate RS232 #2 (4-5)
      0,      4,        0,        0,      1,    // 02   - Config CH1
      0,      4,        0,        0,      1,    // 03   - Config CH2
      0,      4,        0,        0,      1,    // 04   - Config CH3
      0,      4,        0,        0,      1,    // 05   - Config CH4

      0,      65535,        0,        0,      0,    // 06   -
      0,      65535,        0,        0,      0,    // 07   -
      0,      65535,        0,        0,      0,    // 08   -
      0,      65535,        0,        0,      0,    // 09   -
      0,      65535,        0,        0,      0,    // 10   -
      0,      59,        0,        0,      1,    // 11   - RTC.min
      0,      23,        0,        0,      1,    // 12   - RTC.hour
      0,      7,        1,        1,      1,    // 13   - RTC.day
      0,      31,        1,        1,      1,    // 14   - RTC.date
      0,      12,        1,        1,      1,    // 15   - RTC.month
      0,      99,        18,        0,      1,    // 16   - RTC.year
      0,      65535,        0,        0,      0,    // 17   -

      0,      65535,        0,        0,      1,    // 18   - Minimum Value CH1
      0,      65535,        0,        0,      1,    // 19   - Maximum Value CH1
      0,      1999,        1000,        0001,      1,    // 20   - Span CH1
      0,      0,        0,        0,      1,    // 21   - Zero CH1
      0,      65535,        0,        0,      1,    // 22   - Minimum Value CH2
      0,      65535,        0,        0,      1,    // 23   - Maximum Value CH3
      0,      1999,        1000,        0001,      1,    // 24   - Span CH2
      0,      0,        0,        0,      1,    // 25   - Zero CH2
      0,      65535,        0,        0,      1,    // 26   - Minimum Value CH3
      0,      65535,        0,        0,      1,    // 27   - Maximum Value CH3
      0,      1999,        1000,        0001,      1,    // 28   - Span CH3
      0,      0,        0,        0,      1,    // 29   - Zero CH3
      0,      65535,        0,        0,      1,    // 30   - Minimum Value CH4
      0,      65535,        0,        0,      1,    // 31   - Maximum Value CH4
      0,      1999,        1000,        0001,      1,    // 32   - Span CH4
      0,      0,        0,        0,      1,    // 33   - Zero CH4
//      0,      65535,        0,        0,      0,    // 34   - Minimum Value CH4
//      0,      65535,        0,        0,      0,    // 35   - Maximum Value CH4
//      0,      1999,        1000,        0001,      1,    // 36   - Span CH4
//      0,      0,        0,        0,      1,    // 37   - Zero CH4

//      0,      65535,        0,        0,      0,    // 38   -   
//      0,      65535,        0,        0,      0,    // 39   -

//      0,      65535,        0,        0,      0,    // 40   -   
//      0,      65535,        0,        0,      0,    // 41   -  
//      0,      65535,        0,        0,      0,    // 42   -  
//      0,      65535,        0,        0,      0,    // 43   -  
//      0,      65535,        0,        0,      0,    // 44   -  
//      0,      65535,        0,        0,      0,    // 45   -  
//      0,      65535,        0,        0,      0,    // 46   -  
//      0,      65535,        0,        0,      0,    // 47   -  
//      0,      65535,        0,        0,      0,    // 48   -  
//      0,      65535,        0,        0,      0,    // 49   -

//      0,      65535,        0,        0,      0,    // 50   -  
//      0,      65535,        0,        0,      0,    // 51   -
//      0,      65535,        0,        0,      1,    // 52   -
//      0,      65535,        0,        0,      0,    // 53   -  
//      0,      65535,        0,        0,      0,    // 54   -  
//      0,      65535,        0,        0,      0,    // 55   -  
//      0,      65535,        0,        0,      0,    // 56   -  
//      0,      65535,        0,        0,      0,    // 57   -  
//      0,      65535,        0,        0,      0,    // 58   -  
//      0,          1,        0,        0,      1,    // 59   -

//      0,      65535,        0,        0,      0,    // 60   -  
//      0,      65535,        0,        0,      0,    // 61   -  
//      0,      65535,        0,        0,      0,    // 62   -  
//      0,      65535,        0,        0,      0,    // 63   -
//      0,          1,        0,        0,      1,    // 64   -  
//      0,          1,        0,        0,      1,    // 65   -  
//      0,          1,        0,        0,      1     // 66   -  
};
#endif


#if MBFN_MASTER_REGISTERS_ENABLED > 0
RegStructure  MasterRegisterInputs        [NUMBER_MASTER_INPUT_REGISTERS];
#endif
/********************End of Input/Output Coils and Registers*******************/
