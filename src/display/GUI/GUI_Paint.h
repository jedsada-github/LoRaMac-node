#ifndef __GUI_PAINT_H
#define __GUI_PAINT_H

// #include "DEV_Config.h"
#include "../Fonts/fonts.h"

/**
 * Image attributes
**/
typedef struct {
    uint8_t *Image;
    uint32_t Width;
    uint32_t Height;
    uint32_t WidthMemory;
    uint32_t HeightMemory;
    uint32_t Color;
    uint32_t Rotate;
    uint32_t Mirror;
    uint32_t WidthByte;
    uint32_t HeightByte;
    uint32_t Scale;
} PAINT;
extern PAINT Paint;

/**
 * Display rotate
**/
#define ROTATE_0            0
#define ROTATE_90           90
#define ROTATE_180          180
#define ROTATE_270          270

/**
 * Display Flip
**/
typedef enum {
    MIRROR_NONE  = 0x00,
    MIRROR_HORIZONTAL = 0x01,
    MIRROR_VERTICAL = 0x02,
    MIRROR_ORIGIN = 0x03,
} MIRROR_IMAGE;
#define MIRROR_IMAGE_DFT MIRROR_NONE

/**
 * image color
**/
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED           0XFFE0
#define GBLUE          0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0XBC40
#define BRRED          0XFC07
#define GRAY           0X8430

#define IMAGE_BACKGROUND    BLACK
#define FONT_FOREGROUND     WHITE
#define FONT_BACKGROUND     BLACK

/**
 * The size of the point
**/
typedef enum {
    DOT_PIXEL_1X1  = 1,	// 1 x 1
    DOT_PIXEL_2X2  , 		// 2 X 2
    DOT_PIXEL_3X3  ,		// 3 X 3
    DOT_PIXEL_4X4  ,		// 4 X 4
    DOT_PIXEL_5X5  , 		// 5 X 5
    DOT_PIXEL_6X6  , 		// 6 X 6
    DOT_PIXEL_7X7  , 		// 7 X 7
    DOT_PIXEL_8X8  , 		// 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

/**
 * Point size fill style
**/
typedef enum {
    DOT_FILL_AROUND  = 1,		// dot pixel 1 x 1
    DOT_FILL_RIGHTUP  , 		// dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT  DOT_FILL_AROUND  //Default dot pilex

/**
 * Line style, solid or dashed
**/
typedef enum {
    LINE_STYLE_SOLID = 0,
    LINE_STYLE_DOTTED,
} LINE_STYLE;

/**
 * Whether the graphic is filled
**/
typedef enum {
    DRAW_FILL_EMPTY = 0,
    DRAW_FILL_FULL,
} DRAW_FILL;

/**
 * Custom structure of a time attribute
**/
typedef struct {
    uint32_t	Year;  //0000
    uint8_t Month; //1 - 12
    uint8_t Day;   //1 - 30
    uint8_t Hour;  //0 - 23
    uint8_t Min;   //0 - 59
    uint8_t Sec;   //0 - 59
} PAINT_TIME;
extern PAINT_TIME sPaint_time;

/**
 * Custom structure of a time attribute
**/
typedef struct {
    int32_t lat;  //decimal degree
    int32_t lon; //decimal degree
    uint16_t alt;   //m
    bool fix;  //0: not fix, 1: fixed
} PAINT_GPS;
extern PAINT_GPS sPaint_gps;

/**
 * Custom structure of a time attribute
**/
typedef struct {
    int16_t rssi;  //-137
    int8_t lsnr; //1 -20
    uint8_t dr;   //SF5~SF12
    uint8_t len;  //1 - 254
    uint8_t port;  //0 - 254
} PAINT_LoRa;
extern PAINT_LoRa sPaint_lora;

//init and Clear
void Paint_NewImage(uint8_t *image, uint32_t Width, uint32_t Height, uint32_t Rotate, uint32_t Color);
void Paint_SelectImage(uint8_t *image);
void Paint_SetRotate(uint32_t Rotate);
void Paint_SetMirroring(uint8_t mirror);
void Paint_SetPixel(uint32_t Xpoint, uint32_t Ypoint, uint32_t Color);
void Paint_SetScale(uint8_t scale);

void Paint_Clear(uint32_t Color);
void Paint_ClearWindows(uint32_t Xstart, uint32_t Ystart, uint32_t Xend, uint32_t Yend, uint32_t Color);

//Drawing
void Paint_DrawPoint(uint32_t Xpoint, uint32_t Ypoint, uint32_t Color, DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_FillWay);
void Paint_DrawLine(uint32_t Xstart, uint32_t Ystart, uint32_t Xend, uint32_t Yend, uint32_t Color, DOT_PIXEL Line_width, LINE_STYLE Line_Style);
void Paint_DrawRectangle(uint32_t Xstart, uint32_t Ystart, uint32_t Xend, uint32_t Yend, uint32_t Color, DOT_PIXEL Line_width, DRAW_FILL Draw_Fill);
void Paint_DrawCircle(uint32_t X_Center, uint32_t Y_Center, uint32_t Radius, uint32_t Color, DOT_PIXEL Line_width, DRAW_FILL Draw_Fill);

//Display string
void Paint_DrawChar(uint32_t Xstart, uint32_t Ystart, const char Acsii_Char, sFONT* Font, uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawString_EN(uint32_t Xstart, uint32_t Ystart, const char * pString, sFONT* Font, uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawString_CN(uint32_t Xstart, uint32_t Ystart, const char * pString, cFONT* font, uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawNum(uint32_t Xpoint, uint32_t Ypoint, double Nummber, sFONT* Font, uint32_t Digit,uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawTime(uint32_t Xstart, uint32_t Ystart, PAINT_TIME *pTime, sFONT* Font, uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawGps(uint32_t Xstart, uint32_t Ystart, PAINT_GPS *pGps, sFONT* Font,
                    uint32_t Color_Foreground, uint32_t Color_Background);
void Paint_DrawLoRa(uint32_t Xstart, uint32_t Ystart, PAINT_LoRa *pLora, sFONT* Font,
                    uint32_t Color_Foreground, uint32_t Color_Background);
//pic
void Paint_DrawBitMap(const unsigned char* image_buffer);
void Paint_DrawBitMap_Block(const unsigned char* image_buffer, uint8_t Region);

void Paint_DrawImage(const unsigned char *image, uint32_t xStart, uint32_t yStart, uint32_t W_Image, uint32_t H_Image) ;
void Paint_DrawImage1(const unsigned char *image, uint32_t xStart, uint32_t yStart, uint32_t W_Image, uint32_t H_Image);
void Paint_BmpWindows(unsigned char x,unsigned char y,const unsigned char *pBmp,\
					unsigned char chWidth,unsigned char chHeight);


#endif





