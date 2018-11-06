/*
 * st7920.h
 *
 *	Created: 15-03-2016 16:29:39
 *	Author: Álvaro Matos 1131416@isep.ipp.pt
 *	Revision: António Pereira 1120934@isep.ipp.pt
 *
 *	Executado no ano lectivo 2015/17 no
 *	anmbito do projeto da cadeira de SIINS
 *	no qual fui cooautor
 */

#ifndef ST7920_H_
#define ST7920_H_

#include <avr/io.h>
#define S_BIT(PORTO,BIT) (PORTO|=(1<<BIT))		//Set bit
#define C_BIT(PORTO,BIT) (PORTO&=~(1<<BIT))		//Clear bit
 #define	LCD_DDR						DDRB							//Direction register
 #define	LCD_PORT					PORTB							//Port register
 #define	LCD_PWR						0									//LCD Power control pin
 #define	LCD_CLK						1									//LCD Clock pin
 #define	LCD_DATA					2									//LCD Data pin
// função não utilizada
// #define	LCD_RST						3									//LCD Reset pin
 #define	START_BYTE				0xf8
 #define	WRITE_DATA				0x02
 #define	ENTRY_MODE				0X04
 #define	ENTRY_MODE_INC		0x02
 #define	ENTRY_MODE_SHIFT	0X01	
 #define	DISPLAY_CONTROL		0x08
 #define	DISPLAY_ON				(1<<2)
 #define	CURSOR_ON					(1<<1)
 #define	CHAR_BLINK_ON			(1<<0)
 #define	CLR_INST					0x01
 #define	CHAR_TERM					0X03							// não pode ser utilizado como um character
 #define	FUNCTION_SET			0x30
 #define	EXT_INS						0x04
 #define	GRAPH_ON					0x02
 
 #define	GRAPH_RAM_INIT		0x80
 #define	LINHA1						0x80
 #define	LINHA2						0x90
 #define	LINHA3		LINHA1+	0x08
 #define	LINHA4		LINHA2+	0x08

/* Functions */
void lcd_init(void);
void lcd_write_byte(uint8_t B);
void lcd_instruction(int8_t ins);
void lcd_data(int8_t data);
void lcd_set_cursor(int8_t line, int8_t col);
void lcd_clear();
void lcd_clear_graph();
void lcd_clear_all();
void lcd_reset();
void lcd_send_str(char *str);
void lcd_draw(uint8_t *p);
void lcd_draw_str8(uint8_t linha,uint8_t coluna, char string[16]);

#endif /* ST7920_H_ */