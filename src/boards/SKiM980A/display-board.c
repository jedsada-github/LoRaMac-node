/*!
 * \file      display-board.c
 *
 * \brief     Target board display implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *
 * \endcode
 *
 * \author    Anol P. ( EmOne ) <anol.p@emone.co.th>
 */
#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "display-board.h"
#include "string.h"

static Gpio_t OledNrst;
static Gpio_t OledKey1;
static Gpio_t OledKey2;
static Gpio_t OledDC;
static Gpio_t OledCS;
// static Gpio_t OledSck;
// static Gpio_t OledDi;

//extern I2c_t I2c;
extern Spi_t Spi;

// basic lcd display support interface
uint8_t m_startLine = 0;
uint8_t m_column = 0;
uint8_t m_page = 0;
uint8_t m_seg_offset = 0;
#define Imagesize  1024 //(((OLED_WIDTH%8==0)? (OLED_WIDTH/8): (OLED_WIDTH/8+1)) * OLED_HEIGHT)
volatile uint8_t BlackImage[Imagesize];
extern PAINT Paint;
static bool wkup = 0;
/********************************************************************************
function:   
            reverse a byte data
********************************************************************************/
static uint8_t reverse(uint8_t temp)
{
    temp = ((temp & 0x55) << 1) | ((temp & 0xaa) >> 1);
    temp = ((temp & 0x33) << 2) | ((temp & 0xcc) >> 2);
    temp = ((temp & 0x0f) << 4) | ((temp & 0xf0) >> 4);  
    return temp;
}

void DisplayMcuOnKey1Signal( void* context )
{
    // Wake up
    if(wkup == 0) {
        LpmSetStopMode( LPM_DISPLAY_ID , LPM_ENABLE );
        DisplayOff();
        wkup = 1;
        /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
        Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
        // HAL_SuspendTick();

        // LpmEnterStopMode();
    } else {
        LpmSetStopMode( LPM_DISPLAY_ID , LPM_DISABLE );
        wkup = 0;
        /* Resume Tick interrupt if disabled prior to SLEEP mode entry */
        // HAL_ResumeTick();
        DisplayInitReg();
        DisplayOn();
    }
}

void DisplayMcuOnKey2Signal( void* context )
{
    // Reactivation
    LpmSetStopMode( LPM_DISPLAY_ID , LPM_DISABLE );
    NvmDataMgmtFactoryReset( );
    BoardResetMcu();
}

void DisplaySendData_8Bit(uint8_t val)
{
    DisplaySendData(&val, 1);
}

void DisplaySendData_16Bit(uint16_t val)
{
    DisplaySendData((uint8_t *) &val, 2);
}

void DisplayInitReg( void )
{
    DisplayOff(); /*turn off OLED display*/

    DisplaySendCommand(0x00);   //Set Lower Column Address

    DisplaySendCommand(0x10);   //Set Higher Column Address

    DisplaySendCommand(0xB0);  	//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F, SSD1305_CMD)
    
    DisplaySendCommand(0xDC);   //#et display start line 
    DisplaySendCommand(0x00); 

    DisplaySendCommand(0x81);   //contrast control 
    DisplaySendCommand(0x80);   //128
    
    DisplaySendCommand(0x21);   // Set Memory addressing mode (0x20/0x21); //

    DisplaySendCommand(0xA0);    //set segment remap (0xA0 down/ 0xA1 up) rotates

    DisplaySendCommand(0xC0);    //COM0 to COM[N-1] scan direction
    
    DisplaySendCommand(0xA4);   //Set Entire Display On (0xA4 normal/0xA5 entire); 
    
    DisplaySendCommand(0xA6);    //0xA6 normal / 0xA7 reverse // Set SEG Output Current Brightness ��
   
    DisplaySendCommand(0xA8);    //multiplex ratio  //--Set SEG/Column Mapping	
    DisplaySendCommand(0x3F);    //duty = 1/64
	
    DisplaySendCommand(0xD3);	//--set multiplex ratio(1 to 64) #set display offset 
    DisplaySendCommand(0x60);	//--1/64 duty
    
    DisplaySendCommand(0xD5);	//-set display offset	Shift Mapping RAM Counter (0x00~0x3F) 
    DisplaySendCommand(0x41);  	//-not offset
    
    DisplaySendCommand(0xD9);	//--set pre-charge period
    DisplaySendCommand(0x22);	//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock

    DisplaySendCommand(0xDB); 	 /*set vcomh*/
    DisplaySendCommand(0x35);  	//Set VCOM Deselect Level

	DisplaySendCommand(0xAD);    //set charge pump enable 
    DisplaySendCommand(0x8A);    //Set DC-DC enable (a=0:disable; 
}

/*!
 * \brief Initializes the display
 */
void DisplayInit( void )
{
    GpioInit(&OledNrst, OLED_NRST, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&OledDC, OLED_DC, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    
    GpioInit(&OledKey1, OLED_KEY1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioSetInterrupt( &OledKey1, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &DisplayMcuOnKey1Signal );
    GpioInit(&OledKey2, OLED_KEY2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioSetInterrupt( &OledKey2, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &DisplayMcuOnKey2Signal );

    GpioInit(&OledCS, OLED_CS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    // I2cInit(&I2c, I2C_1, OLED_SCL, OLED_SDA);
    SpiInit(&Spi, SPI_2, OLED_DI, NC, OLED_SCK, NC);

    DisplayReset();

    DisplayInitReg();

    RtcDelayMs(200);

    DisplayOn();
    
    Paint_NewImage(BlackImage, OLED_WIDTH, OLED_HEIGHT, 0, BLACK);	
    
    // //1.Select Image
    Paint_SelectImage(BlackImage);
}

/*!
 * \brief Resets the display
 */
void DisplayReset( void )
{
    GpioWrite( &OledNrst, 1 );
    RtcDelayMs(100);
    GpioWrite( &OledNrst, 0 );    // power up the Display
    RtcDelayMs(100);
    GpioWrite( &OledCS, 1 );    
    GpioWrite( &OledDC, 0 );    
    GpioWrite( &OledNrst, 1 );    // power up the Display
    RtcDelayMs(100);
}

/*!
 * \brief Sends a command to the display
 *
 * \param cmd Command to be sent
 */
void DisplaySendCommand( uint8_t cmd )
{
    GpioWrite( &OledDC, 0 );
    GpioWrite( &OledCS, 0 );
    SpiOut(&Spi, (uint16_t) cmd);
    GpioWrite( &OledCS, 1 );
}

/*!
 * \brief Sends a data buffer to the display
 *
 * \param buffer Buffer to be sent
 * \param size   Buffer size to be sent
 */
void DisplaySendData( uint8_t *buffer, uint16_t size )
{
    GpioWrite( &OledDC, 1 );
    GpioWrite( &OledCS, 0 );
    while (size--)
    {
        SpiOut(&Spi, (uint16_t) *(buffer++));
    }
    GpioWrite( &OledCS, 1 );
}

/*!
 * \brief Enables the display
 */
void DisplayOn( void ) 
{
	//Turn on the OLED display
    DisplaySendCommand(0xAF);
}

/*!
 * \brief Disables the display
 */
void DisplayOff( void )
{
	//Turn off the OLED display
    DisplaySendCommand(0xAE);
}

/*!
 * \brief Clears the display
 */
void DisplayClear( void )
{
    memset( Paint.Image, 0x0, Imagesize);
    DisplayUpdate();
}

/*!
 * \brief Inverts colors of the display
 *
 * \param invert [true: invert, false: normal]
 */
void DisplayInvertColors( bool invert )
{

}

/*!
 * \brief Updates the display with MCU RAM copy
 */
void DisplayUpdate( void )
{
    uint32_t Width, Height, column, temp;
    Width = (OLED_WIDTH % 8 == 0)? (OLED_WIDTH / 8 ): (OLED_WIDTH / 8 + 1);
    Height = OLED_HEIGHT;   
    DisplaySendCommand(0xb0); 	//Set the row  start address
    for (uint32_t j = 0; j < Height; j++) {
        column = 63 - j;
        DisplaySendCommand(0x00 + (column & 0x0f));  //Set column low start address
        DisplaySendCommand(0x10 + (column >> 4));  //Set column higt start address
        for (uint32_t i = 0; i < Width; i++) {
            temp =  Paint.Image[i + j * Width];
            // printf("0x%x \r\n",temp);
            temp = reverse(temp);	//reverse the buffer
            DisplaySendData_8Bit(temp);
         }
    }  
}

/*!
 * \brief Sets the cursor at coordinates (x,y)
 *
 * \param x   X coordinate
 * \param y   Y coordinate
 */
void DisplaySetCursor( int16_t x, int16_t y )
{

}

/*!
 * \brief Gets current X coordinate of the cursor
 *
 * \retval x   X coordinate
 */
int16_t DisplayGetCursorX( void )
{
    return 0;
}

/*!
 * \brief Gets current Y coordinate of the cursor
 *
 * \retval y   Y coordinate
 */
int16_t DisplayGetCursorY( void )
{
    return 0;
}

/*!
 * \brief Sets text size
 *
 * \param s New text size
 */
void DisplaySetTextSize( uint8_t s )
{

}

/*!
 * \brief Sets text color
 *
 * \param color New text color
 */
void DisplaySetTextColor( DisplayColor_t color )
{

}

/*!
 * \brief Sets foreground and background color
 *
 * \param fg Foreground color
 * \param bg Background color
 */
void DisplaySetFgAndBg( DisplayColor_t fg, DisplayColor_t bg )
{

}

/*!
 * \brief Enables/Disable text wrapping
 *
 * \param w [true: wrap ON, false: wrap OFF]
 */
void DisplaySetTextWrap( bool w )
{

}

/*!
 * \brief Gets current display rotation
 *
 * \retval rotation   Display rotation (Vertical/Horizontal)
 */
uint8_t DisplayGetRotation( void )
{
    return 0;
}

/*!
 * \brief Sets current display rotation
 *
 * \param x   Display rotation (Vertical/Horizontal)
 */
void DisplaySetRotation( uint8_t x )
{
    uint8_t memoryAccessReg = 0x00;
    if(x)
    {
        memoryAccessReg = 0x70;
    } 
    DisplaySendCommand(0x36); //MX, MY, RGB mode
    DisplaySendData(&memoryAccessReg, 1);   //RGB

}

/*!
 * \brief Draws a pixel of color at coordinates (x,y)
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param color Pixel color
 */
void DisplayDrawPixel( int16_t x, int16_t y, DisplayColor_t color )
{
    Paint_DrawPoint(x, y, color, DOT_PIXEL_1X1, DOT_STYLE_DFT);
}

/*!
 * \brief Draws a line starting at coordinates (x0,y0) ending at
 *        coordinates (x1,y1) of color
 *
 * \param x0     X0 coordinate
 * \param y0     Y0 coordinate
 * \param x1     X1 coordinate
 * \param y1     Y1 coordinate
 * \param color  Line color
 */
void DisplayDrawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, DisplayColor_t color )
{

}

/*!
 * \brief Draws a vertical line starting at coordinates (x,y) with given height
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param h     Line height
 * \param color Line color
 */
void DisplayDrawVerticalLine( int16_t x, int16_t y, int16_t h, DisplayColor_t color )
{

}

/*!
 * \brief Draws an Horizontal line starting at coordinates (x,y) with given width
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param w     Line width
 * \param color Line color
 */
void DisplayDrawHorizontalLine( int16_t x, int16_t y, int16_t w, DisplayColor_t color )
{

}

/*!
 * \brief Draws a rectangle at coordinates (x,y) with given width and height
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param w     Line width
 * \param h     Line height
 * \param color Line color
 */
void DisplayDrawRect( int16_t x, int16_t y, int16_t w, int16_t h, DisplayColor_t color )
{

}

/*!
 * \brief Draws a filled rectangle at coordinates (x,y) with given width and height
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param w     Line width
 * \param h     Line height
 * \param color Fill color
 */
void DisplayFillRect( int16_t x, int16_t y, int16_t w, int16_t h, DisplayColor_t color )
{

}

/*!
 * \brief Fills all display with pixels of color
 *
 * \param color Fill color
 */
void DisplayFillScreen( DisplayColor_t color )
{

}

/*!
 * \brief Draws a triangle by giving the 3 vertices coordinates
 *
 * \param x0    X0 coordinate
 * \param y0    Y0 coordinate
 * \param x1    X1 coordinate
 * \param y1    Y1 coordinate
 * \param x2    X2 coordinate
 * \param y2    Y2 coordinate
 * \param color Line color
 */
void DisplayDrawTriangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, DisplayColor_t color )
{

}

/*!
 * \brief Draws a filled triangle by giving the 3 vertices coordinates
 *
 * \param x0    X0 coordinate
 * \param y0    Y0 coordinate
 * \param x1    X1 coordinate
 * \param y1    Y1 coordinate
 * \param x2    X2 coordinate
 * \param y2    Y2 coordinate
 * \param color Fill color
 */
void DisplayFillTriangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, DisplayColor_t color ) 
{

}

/*!
 * \brief Draws a character at given coordinates
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param c     Character
 * \param color Character color
 * \param bg    Background color
 * \param size  Character size
 */
void DisplayDrawChar( int16_t x, int16_t y, unsigned char c, DisplayColor_t color, DisplayColor_t bg, uint8_t size )
{

}

/*!
 * \brief Display putc function. (Mimics standard C putc function)
 *
 * \param c     Character
 */
void DisplayPutc( uint8_t c )
{

}

/*!
 * \brief Sets cursor at line
 *
 * \param line  Line number
 */
void DisplaySetLine( uint8_t line )
{

}

/*!
 * \brief Display print function. Prints the given string
 */
void DisplayPrint( const char *string )
{

}

/*!
 * \brief Display printf function. (Mimics standard C printf function)
 */
void DisplayPrintf( const char *format, ... )
{

}

/*!
 * \brief Draws bitmap at coordinates (x,y) with given width and height
 *
 * \param x     X coordinate
 * \param y     Y coordinate
 * \param pBmp  Bitmap pointer
 * \param w     Line width
 * \param h     Line height
  */
void DisplayDrawBitmap( int16_t x, int16_t y, uint8_t*pBmp, int16_t w, int16_t h)
{
    uint16_t i, j, byteWidth = (w + 7)/8;
    for(j = 0; j < h; j ++){
        for(i = 0; i < w; i ++ ) {
            if(*(pBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                DisplayDrawPixel(x+i, y+j, 1);
            }else {
                DisplayDrawPixel(x+i, y+j, 0);
            }
        }
    }
    DisplayUpdate();  
}
