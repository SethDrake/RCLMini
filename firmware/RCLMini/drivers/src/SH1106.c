#include "SH1106.h"
#include "lcd_generic_font.h"

#include "stm32f10x.h"
#include "stm32f10x_conf.h"


unsigned char TimeX = 2;

char mask;
int xpos;

int vmirror = 0;

void __attribute__((noinline)) I2C_WriteData(uint8_t address, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	I2C_AcknowledgeConfig(lcdI2c, ENABLE);
	/* Send START condition */
	I2C_GenerateSTART(lcdI2c, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_MODE_SELECT)) ;
	/* Send I2C address for write */
	I2C_Send7bitAddress(lcdI2c, address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) ;
	while (NumByteToWrite)
	{
		I2C_SendData(lcdI2c, *pBuffer);
		/* Test on EV8 and clear it */
		while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
		if (NumByteToWrite > 0) {
			/* Point to the next location where the byte write will be saved */
			pBuffer++;
			/* Decrement the read bytes counter */
			NumByteToWrite--;
		}
	}
}

void __attribute__((noinline)) I2C_WriteDataConst(uint8_t address, uint8_t reg, uint8_t constData, uint16_t NumByteToWrite)
{
	I2C_AcknowledgeConfig(lcdI2c, ENABLE);
	/* Send START condition */
	I2C_GenerateSTART(lcdI2c, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_MODE_SELECT)) ;
	/* Send I2C address for write */
	I2C_Send7bitAddress(lcdI2c, address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) ;

	I2C_SendData(lcdI2c, reg);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;

	while (NumByteToWrite)
	{
		I2C_SendData(lcdI2c, constData);
		/* Test on EV8 and clear it */
		while (!I2C_CheckEvent(lcdI2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) ;
		if (NumByteToWrite > 0) {
			/* Decrement the read bytes counter */
			NumByteToWrite--;
		}
	}
}

void sendCommand(uint8_t cmd, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	uint8_t buf[4];
	buf[0] = 0x00;
	buf[1] = cmd;
	for (uint8_t i = 0; i < NumByteToWrite; i++) 
	{
		buf[i + 2] = pBuffer[i];		
	}
	I2C_WriteData(SSD1306_I2C_ADDR, buf, NumByteToWrite + 2);
}

void sendCommandByte(uint8_t cmd, uint8_t value)
{
	sendCommand(cmd, &value, 1);
}

void sendJustCommand(uint8_t cmd)
{
	sendCommand(cmd, 0, 0);
}

void sendData(uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	pBuffer[0] = 0x40;
	I2C_WriteData(SSD1306_I2C_ADDR, pBuffer, NumByteToWrite);
}

void sleepMode(uint8_t enabled)
{
	sendCommand(enabled ? 0xAE : 0xAF, 0, 0);
}

void I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	xpos++;

	uint8_t buf[2] = {reg, data};
	I2C_WriteData(address, buf, 2);
}



/***************** LOW LEVEL ************************************************/
void delayus(int us)
{
	us *= 4;
	for(;us>0;us--) __NOP();
}

void __attribute__ ((noinline))  OLED_PortInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_InitStructure.GPIO_Pin    = OLED_SCL | OLED_SDA;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef  I2C_InitStructure;

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);


	lcdI2c = I2C1;

	delayus(20);

}


void __attribute__ ((noinline))  lcd_init (int mode)
{

	OLED_PortInit();	

	sleepMode(1); 													// Display Off
	sendCommandByte(SSD1306_SetDisplayClockDivider, 0x80);  				// Set Display Clk Div 0x80
	sendCommandByte(SSD1306_SetMultiplexRatio, SSD1306_HEIGHT - 1);    		// Set Multiplex
	sendCommandByte(SSD1306_SetDisplayOffset, 0x00);   					// Set Display Offset
	sendJustCommand(SSD1306_SetDisplayStartLine | 0x00);   				// Set Display Start Line
	sendCommandByte(SSD1306_ChargePumpSetting, 0x14);   					// Charge Pump

	sendCommandByte(SSD1306_SetMemAdressingMode, 0x00);  					// Set Memory Addressing Mode - Horizontal

	sendJustCommand(SSD1306_SetSegmentRemap | 0x01);   					// Set Segment Remap
	sendJustCommand(SSD1306_SetCOMoutScanDirection | 0x08);   				// Scan direction
	sendCommandByte(SSD1306_SetCOMPinsConfig, 0x12);     					// COM Pins 0x12
	sendCommandByte(SSD1306_SetContrast, 0xCF);
	sendCommandByte(SSD1306_SetPrechargePeriod, 0x1F);   					// Precharge period
	sendCommandByte(SSD1306_SetVCOMHDeselectLevel, 0x40);   				// VCOM Detect

	sendJustCommand(SSD1306_AllPixRAM);  									// Draw from RAM - normal mode
	sendJustCommand(SSD1306_SetInverseOff);   								// Disable inversion
	sendJustCommand(SSD1306_DeactivateScroll); 							// Disable scroll	


    sendJustCommand(SSD1306_PageAddrMode_StartColumnLo | 0x02);		    /*set lower column address*/
	sendJustCommand(SSD1306_PageAddrMode_StartColumnHi | 0x00);			/*set higher column address*/
	
	sendJustCommand(SSD1306_PageAddrMode_SetPage | 0x00);				/*set page address*/
	
	sleepMode(0);


	xpos = 0;

	lcd_clear();
	TimeX = 5;
}

 //
 //Clear LCD screen
 //
void __attribute__ ((noinline))  lcd_clear (void)
{	
	uint8_t buf[2] = { 0, SSD1306_WIDTH - 1 };
	sendCommand(SSD1306_SetColumnAddr, buf, 2);
	buf[1] = (SSD1306_HEIGHT / 8) - 1;
	sendCommand(SSD1306_SetPageAddr, buf, 2);
	I2C_WriteDataConst(SSD1306_I2C_ADDR, 0x40, 0x00, (SSD1306_WIDTH * SSD1306_HEIGHT));
}

 //
 //Set current position
 //
void __attribute__ ((noinline))  lcd_gotoxy (uint8_t x ,uint8_t y){
	xpos = x;
	x = x + 2;

	sendJustCommand(SSD1306_PageAddrMode_SetPage + y);
	sendJustCommand(SSD1306_PageAddrMode_StartColumnHi | (x >> 4));
	sendJustCommand(SSD1306_PageAddrMode_StartColumnLo | (x & 0x0f));
}



void  lcd_putnum (int x, int y,char *str){
	
	char c;
	int i,j;

	char* str2 = str;

	uint8_t buf[SSD1306_WIDTH * 2];
	for(i =0;i<3;i++)
	{
		lcd_gotoxy(x,y+2-i);

		str2 = str;

		uint8_t cnt = 1;
		buf[0] = 0x40;
		
		while( (c = (*str2++)) )
		{
			if( (c>=0x30  &&  c<= 0x39)|| (c<11) )
			{
				int n = c - 0x30;
				if(n < 0) n = c+10;

				buf[cnt++] = mask;

				for(j=numbers_idx[n]+i; j< numbers_idx[n]+13*3 ; j+=3)
				{
					int dd = 0;
					if (j< numbers_idx[n+1]) dd = numbers[j];

					if ( (*str2 == '.') && (i==0)&&(j>( numbers_idx[n]+13*3 - 9))) dd |= 0x06;

					if (xpos <= SSD1306_WIDTH) {
						buf[cnt++] = dd ^ mask;
					}
				}
			}
		}

		buf[cnt++] = 0x00;

		I2C_WriteData(SSD1306_I2C_ADDR, buf, cnt);
	}
}

/*
 * Put character to current position
 */
void __attribute__ ((noinline))  lcd_putchar (const char c){
	char cc = c;
	if(cc < 32) cc = 0; else cc -= 32;

	uint8_t buf[7];

	buf[0] = 0x40;
	uint8_t cnt = 1; 
	for (uint8_t i = 0; i < 5; i++) {
		if (xpos < SSD1306_WIDTH)
		{
			buf[cnt++] = (lcd_font[((cc & 0x7f) * 5) + i]) ^ mask;
			xpos++;
		}
	}
	if (xpos <= SSD1306_WIDTH) {
		buf[cnt++] =  0x00 ^ mask;
		xpos++;
	}

	I2C_WriteData(SSD1306_I2C_ADDR, buf, cnt);
	
}

/*
 * Put string from RAM
 */
void __attribute__ ((noinline))  lcd_putstr (const char *str,int fill ){
	char c;
	while( (c = (*str++))  ) lcd_putchar(c);
	uint8_t buf[SSD1306_WIDTH+1];

	if (fill)
	{
		buf[0] = 0x40;
		uint8_t cnt = 1;
		while (xpos <= SSD1306_WIDTH) {
			buf[cnt++] = mask;
			xpos++;
		}
		I2C_WriteData(SSD1306_I2C_ADDR, buf, cnt);
	}
}

void lcd_setcontrast(uint8_t c)
{
	sendCommandByte(SSD1306_SetContrast, c * 6 + 47);
}


void lcd_printBat(int x, int y, int percent)
{
	uint8_t buf[BAT_N_SEG + 3];
	buf[0] = 0x40;
	uint8_t cnt = 1;
	
	lcd_gotoxy(x, y);

	if (percent < 0) {
		percent = 0;
	}
	const int active = 1 + (BAT_N_SEG* percent) / 100;

	for (int i = 0; i < BAT_N_SEG; i++)
	{
		if (i < active)
		{
			buf[cnt++] = 0x7F;
			buf[cnt++] = 0x7F;
			buf[cnt++] = 0x00;
		}
		else
		{
			buf[cnt++] = 0x41;
			buf[cnt++] = 0x41;
			buf[cnt++] = 0x41;
		}
	}
	buf[cnt++] = 0x7F;    // tail
	buf[cnt++] = 0x1C;

	I2C_WriteData(SSD1306_I2C_ADDR, buf, cnt);
}
