#ifndef __SSD1306_H__
#define __SSD1306_H__
/* { */

#include <stdint.h>
#include "stm32f10x_conf.h"

/* GPIOx config */
#define OLED_SCL  GPIO_Pin_6             //PB6
#define OLED_SDA  GPIO_Pin_7             //PB7

#define BAT_N_SEG 6

/**************************************
 * Display macro
 **************************************/

#define SSD1306_I2C_ADDR       0x78
 
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

#define SSD1306_SetContrast                 0x81
#define SSD1306_AllPixRAM                   0xA4
#define SSD1306_AllPixOn                    0xA5
#define SSD1306_SetInverseOff               0xA6
#define SSD1306_SetInverseOn                0xA7
#define SSD1306_Sleep                       0xAE
#define SSD1306_Wake                        0xAF
#define SSD1306_DeactivateScroll            0x2E
#define SSD1306_SetMemAdressingMode         0x20    
#define SSD1306_SetColumnAddr               0x21
#define SSD1306_SetPageAddr                 0x22
#define SSD1306_PageAddrMode_SetPage        0xB0
#define SSD1306_PageAddrMode_StartColumnLo  0x00
#define SSD1306_PageAddrMode_StartColumnHi  0x10
#define SSD1306_SetDisplayStartLine         0x40
#define SSD1306_SetSegmentRemap             0xA0
#define SSD1306_SetMultiplexRatio           0xA8
#define SSD1306_SetCOMoutScanDirection      0xC0 
#define SSD1306_SetDisplayOffset            0xD3
#define SSD1306_SetCOMPinsConfig            0xDA
#define SSD1306_SetDisplayClockDivider      0xD5
#define SSD1306_SetPrechargePeriod          0xD9
#define SSD1306_SetVCOMHDeselectLevel       0xDB
#define SSD1306_ChargePumpSetting           0x8D


I2C_TypeDef* lcdI2c;

/**************************************
 * Type definitions
 **************************************/

/* Command or data */
typedef enum
{
	COMMAND = 0,
	DATA = 1
} lcd_cd_t;

/* Font descriptor type */
typedef struct
{
	uint16_t char_width;
	uint16_t char_offset;
} font_descriptor_t;

/* Font info type */
typedef struct
{
	uint16_t font_height;
	unsigned char start_char;
	unsigned char end_char;
	const font_descriptor_t* descr_array;
	const unsigned char* font_bitmap_array;
} font_info_t;

extern void sendCommand(uint8_t cmd, uint8_t* pBuffer, uint16_t NumByteToWrite);
extern void sendCommandByte(uint8_t cmd, uint8_t value);
extern void sendJustCommand(uint8_t cmd);
extern void sendData(uint8_t* pBuffer, uint16_t NumByteToWrite);

extern void sleepMode(uint8_t enabled);

extern void I2C_WriteData(uint8_t address, uint8_t* pBuffer, uint16_t NumByteToWrite);
extern void I2C_WriteDataConst(uint8_t address, uint8_t reg, uint8_t constData, uint16_t NumByteToWrite);
extern void I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
extern void lcd_init (int mode);
extern void lcd_hw_reset (void);
extern void lcd_write (lcd_cd_t cd, uint8_t byte);
extern void lcd_clear (void);
extern void lcd_gotoxy (uint8_t x ,uint8_t y);
extern void lcd_putchar (const char c);
extern void lcd_putstr (const char *str,int fill);
extern void lcd_putnum (int x, int y,char *str);
extern void lcd_printBat(int x, int y, int percent);
void lcd_setcontrast(uint8_t c);

extern char mask;

#endif
