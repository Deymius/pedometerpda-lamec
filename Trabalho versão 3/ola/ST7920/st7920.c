/*
 *	st7920.c
 *
 *	Created: 15-03-2016 16:29:39
 *	Author: Álvaro Matos 1131416@isep.ipp.pt
 *	Revision: António Pereira 1120934@isep.ipp.pt
 *
 *	Executado no ano lectivo 2015/17 no
 *	anmbito do projeto da cadeira de SIINS
 *	no qual fui coautor
 */ 

#include "st7920.h"
#define F_CPU 20000000
#include <util/delay.h>

#include	"font.h"

void lcd_write_byte(uint8_t B)	{
	uint8_t aux,i;
  for(i=8;i>=1;i--)	{
		C_BIT(LCD_PORT,LCD_CLK);
		aux=((B>>(i-1))&1);
		if(aux==1) S_BIT(LCD_PORT,LCD_DATA);
		else C_BIT(LCD_PORT,LCD_DATA);
		//_delay_us(1);
		S_BIT(LCD_PORT,LCD_CLK);
		//_delay_us(1);
	}
}

void lcd_instruction(int8_t ins)	{
	lcd_write_byte(START_BYTE); // 5 1 bits, RS = 0, RW = 0
	lcd_write_byte(ins & 0xf0);
	lcd_write_byte(ins << 4);
	_delay_us(72);
}

void lcd_data(int8_t data)	{
	lcd_write_byte(START_BYTE+WRITE_DATA); // 5 1 bits, RS = 1, RW = 0
	lcd_write_byte(data & 0xf0);
	lcd_write_byte(data << 4);
	_delay_us(72);
}

void lcd_set_cursor(int8_t line, int8_t col)	{
	// Contrary to the data sheet, the starting addresses for lines
	// 0, 1, 2 and 3 are 80, 90, 88 and 98
	lcd_instruction(FUNCTION_SET);//modo char 16
	switch(line)	{
		case 0:lcd_instruction(LINHA1+col);
			break;
		case 1:lcd_instruction(LINHA2+col);
			break;
		case 2:lcd_instruction(LINHA3+col);
			break;
		case 3:lcd_instruction(LINHA4+col);
			break;
	} 
}

void lcd_clear()	{
	lcd_instruction(FUNCTION_SET);//modo char 16
	lcd_instruction(CLR_INST); // clear
	_delay_ms(2); // Needs 1.62ms delay
}
 
void lcd_clear_all()	{
	lcd_clear();
	lcd_clear_graph();
}
 
void lcd_clear_graph()	{
	int ygroup,x,y,i; 
	for(ygroup=0;ygroup<64;ygroup++)	{
		if(ygroup<32)	{
			x=GRAPH_RAM_INIT;				//posição do inicio da primeira linha na memoria e nao no display
			y=ygroup+GRAPH_RAM_INIT;			//distancia de cima para baixo na memoria, e nao no display
		}	else	{
			x=GRAPH_RAM_INIT+0x08;	 //posição do inicio da segunda linha na memoria e nao no display
			y=ygroup-32+GRAPH_RAM_INIT;		//distancia de cima para baixo na memoria, e nao no display
		}
		lcd_instruction(FUNCTION_SET+EXT_INS);		//EXTENDED INSTRUCTION
		lcd_instruction(y);												//ENDEREÇO VERTICAL
		lcd_instruction(x);												//ENDEREÇO HORIZONTAL
		//lcd_instruction(0x30);										//BASIC INSTRUCTION
		for(i=0;i<16;i++)	{
			lcd_data(0);
		}
	}
	lcd_instruction(FUNCTION_SET+EXT_INS+GRAPH_ON);
	lcd_clear();
}

void lcd_init()	{
//	S_BIT(LCD_DDR,LCD_RST);			//Set bit on DDR register to be a output
	S_BIT(LCD_DDR,LCD_DATA);		//Set bit on DDR register to be a output
	S_BIT(LCD_DDR,LCD_CLK);			//Set bit on DDR register to be a output
	S_BIT(LCD_DDR,LCD_PWR);			//Set bit on DDR register to be a output
//	S_BIT(LCD_PORT,LCD_RST);		//Put Reset bit = 1
	S_BIT(LCD_PORT,LCD_PWR);		//Put Power bit = 1
	_delay_ms(10);							// Wait for LCD to start
	lcd_instruction(FUNCTION_SET); // 8 bit data, basic instructions
	lcd_instruction(DISPLAY_CONTROL+DISPLAY_ON); // display on
	lcd_clear();
	lcd_instruction(ENTRY_MODE+ENTRY_MODE_INC); // increment, no shift
}

void lcd_send_str(char *str)	{
	uint8_t i=0;
	lcd_instruction(FUNCTION_SET);//modo char 16
	while(str[i]!='\0')	{
		lcd_data(str[i]);
		i++;
	}
}

void lcd_draw(uint8_t *p)	{
	int ygroup,x,y,i,temp,tmp;
	for( ygroup = 0; ygroup < 64; ygroup++)	{                           
		if(ygroup<32)	{
			x=GRAPH_RAM_INIT;			//posição do inicio da primeira linha na memoria e nao no display
			y=ygroup+GRAPH_RAM_INIT;		//distancia de cima para baixo na memoria, e nao no display
		}	else	{
			x=GRAPH_RAM_INIT+0x08; //posição do inicio da segunda linha na memoria e nao no display
			y=ygroup-32+GRAPH_RAM_INIT; //distancia de cima para baixo na memoria, e nao no display
		}
		lcd_instruction(FUNCTION_SET+EXT_INS);        //EXTENDED INSTRUCTION
		lcd_instruction(y);														//ENDEREÇO VERTICAL
		lcd_instruction(x);														//ENDEREÇO HORIZONTAL
		tmp = ygroup*16;
		for( i = 0; i < 16 ; i++)	{
			temp = p[tmp++];
//imagens criadas pelo gimp ficam com os bytes de traz para a frente entao uso o seguinte for para os trocar
			for( x = 0, y = 0; x <= 7; x++)	{
				y|= (((temp >> x) & 0x01) << ( 7 - x));			 
			}
			lcd_data(y);
		}
	}
	lcd_instruction(FUNCTION_SET+EXT_INS+GRAPH_ON);
}
 