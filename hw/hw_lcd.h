#ifndef __HW_LCD_H
#define __HW_LCD_H

#include "ti_msp_dl_config.h"

#define LCD_W 320
#define LCD_H 170

#define LCD_RES_Clr() DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_PIN_RES_PIN)
#define LCD_RES_Set() DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_PIN_RES_PIN)

#define LCD_DC_Clr() DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_PIN_DC_PIN)
#define LCD_DC_Set() DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_PIN_DC_PIN)

#define LCD_CS_Clr() DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_PIN_CS_PIN)
#define LCD_CS_Set() DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_PIN_CS_PIN)

#define LCD_BLK_Clr() DL_GPIO_clearPins(GPIO_LCD_PORT, GPIO_LCD_PIN_BLK_PIN)
#define LCD_BLK_Set() DL_GPIO_setPins(GPIO_LCD_PORT, GPIO_LCD_PIN_BLK_PIN)

void LCD_WR_DATA(unsigned int dat);
void LCD_WR_DATA8(unsigned char dat);
void lcd_init(void);
void LCD_Address_Set(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2);
void LCD_Fill(unsigned int xsta, unsigned int ysta, unsigned int xend,
    unsigned int yend, unsigned int color);
void LCD_DrawPoint(unsigned int x, unsigned int y, unsigned int color);
void LCD_DrawLine(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2, unsigned int color);
void LCD_DrawRectangle(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2, unsigned int color);
void Draw_Circle(unsigned int x0, unsigned int y0, unsigned char r,
    unsigned int color);
void LCD_ShowChinese(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowChinese12x12(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowChinese16x16(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowChinese24x24(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowChinese32x32(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowChar(unsigned int x, unsigned int y, unsigned char num,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
void LCD_ShowString(unsigned int x, unsigned int y, const unsigned char *p,
    unsigned int fc, unsigned int bc, unsigned char sizey,
    unsigned char mode);
unsigned int mypow(unsigned char m, unsigned char n);
void LCD_ShowIntNum(unsigned int x, unsigned int y, unsigned int num,
    unsigned char len, unsigned int fc, unsigned int bc,
    unsigned char sizey);
void LCD_ShowFloatNum1(unsigned int x, unsigned int y, float num,
    unsigned char len, unsigned int fc, unsigned int bc,
    unsigned char sizey);
void LCD_ShowPicture(unsigned int x, unsigned int y, unsigned int length,
    unsigned int width, const unsigned char pic[]);
void LCD_ArcRect(unsigned int xsta, unsigned int ysta, unsigned int xend,
    unsigned int yend, unsigned int color);
void LCD_DrawVerrticalLine(int x, int y1, int y2, unsigned int color);

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430
#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458
#define LIGHTGREEN 0X841F
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12

#endif
