/*
 * v1.0-atmega168p.c
 *
 * Created: 11/20/2018 9:08:10 PM
 * Author : tastruck
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


#define F_CPU 8000000

volatile int cont=0;
volatile char usart0_data;									// Vari?vel com o caracter recebido na USART0

volatile struct{ 			// Flags
	uint8_t rx0_on:			1;			// Indica que houve uma recep??o na USART0
	uint8_t rx1_on:			1;			// Indica que houve uma recep??o na USART1
	uint8_t blink_ok:		1;			// Indica que pode piscar
	uint8_t lcd_on:			1;			// Indica que LCD est? on e pode actualizar o PWM
	uint8_t tmr_on:			1;			// Indica que tem tempo a descontar
	uint8_t deb:			1;			// Tempo de debounce
	uint8_t D_val:			1;			// Dados v?lidos
	uint8_t all:			1;			//
}myflags;

void start(){
	DDRB = 0b00000001;
	PORTB = 0b00000000;
		
	//USART0
	UBRR0H	=	0x01;										// Setup USART0
	UBRR0L	=	0x9f;										// BaudRate 9600
	UCSR0A	=	0b00000010;									// U2Xn = 1,(UBRRnX=(F_CPU/(2*B_Rate))-1) ==> 416 ==> 19fh
	UCSR0B	=	0b10011000;									// 8N1
	UCSR0C	=	0b00000110;									//
	
	//TIMER0
	OCR0A		=	240;									// Valor para compara??o no timer0
	TCCR0A	=	0b10000010;									// Configura??es timer0 = 2.5ms
	TCCR0B	=	0b00000101;									// CTC mode 1024 presacaler
	TIMSK0 |=	0b00000010;									// enable T0
	
	sei();
}

ISR(TIMER0_COMPA_vect){										//contador de 500ms
	cont++;													// incrementa contador
	if (cont==15)	{										// 200*2.5=500ms
		cont=0;												// limpar variavel de tempo
		PORTB ^= (1<<PORTB0);
	}
}

ISR(USART0_RX_vect){
	myflags.rx0_on=	1;										// turn on flag on (data in buffer)
	usart0_data		=	UDR0;								// save data on here
}

int main(void)
{
    /* Replace with your application code */
	start();
	
    while (1) 
    {
    }
}

