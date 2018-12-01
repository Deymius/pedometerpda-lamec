/*
 * v1.0-atmega168p.c
 *
 * Created: 11/20/2018 9:08:10 PM
 * Author : tastruck
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "RTC/rtc.h"
//#include <RTC/twi.h>


#define F_CPU 8000000


volatile int cont=0;
volatile int i=0;
volatile char usart0_data;									// Vari?vel com o caracter recebido na USART0
	 	 char PC_buffer[30];									// Vetor com os dados a transmitir para o PCvola
		  
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

void send_PC(){
	unsigned char i=0;										// Vari?vel local, apontador de bytes
	while(PC_buffer[i]!='\0'){								// enquanto houver bytes para envio
		while((UCSR0A & 1<<UDRE0)==0);						// esperar pelo envio de 1 byte
		UDR0=PC_buffer[i];									// pr?ximo byte
		i++;												// incrementa apontador
	}
}

/*void identificar()	{
	sprintf(PC_buffer, "\r\n PEDOMETRO \r\n");		//rs232
	send_PC();
	sprintf(PC_buffer, "1140205@isep.ipp.pt - Andre Marques\r\n\n");			//rs232
	send_PC();
	sprintf(PC_buffer, "1141286@isep.ipp.pt - Thor Jose Struck\r\n\n");			//rs232
	send_PC();
}*/

void rtc_send_time(){
	sprintf(PC_buffer, "DS RTC Library Test\n");
	send_PC();
		
}

void start(){
	DDRB = 0b00000001;
	PORTB = 0b00000000;
		
	//USART0
	UBRR0H	=	0x00;										// Setup USART0
	UBRR0L	=	0x67;										// BaudRate 9600
	UCSR0A	=	0b00000010;									// U2Xn = 1,(UBRRnX=(F_CPU/(2*B_Rate))-1) ==> 103.2 ==> 67h
	UCSR0B	=	0b00011000;									// 8N1
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
		//identificar();
		i = 1;
	}
}

void Init_RTC(){
	start();
	
	rtc_init();
	//rtc_set_time_s(12, 59, 50);
	sprintf(PC_buffer, "After Init\n");
	send_PC();
	sprintf(PC_buffer, "DS3231\n");
	send_PC();
	sprintf(PC_buffer,"--------");
	send_PC();
}

void send_Email(){
	
}

/*ISR(USART0_RX_vect){
	myflags.rx0_on=	1;										// turn on flag on (data in buffer)
	usart0_data		=	UDR0;								// save data on here
}*/

int main(void)
{
    /* Replace with your application code */
	struct tm* t = NULL;
	uint8_t midFlag = 0;
	//uint8_t hour, min, sec;
	
	Init_RTC();
	
    while (1) 
    {
		t = rtc_get_time();
		
		if(midFlag == 1){
			send_Email();
			midFlag = 0;
		}
		
		if((t->hour == 23) && (t->min == 59) && (t->sec == 59) && (midFlag == 0){
				//Send report to e-mail
				//Pode ter uma função que envia via a interface e espera por um acknoledge
				//Com este acknoledge faz reset a flag de envio ja que o envio foi feito com sucesso
				midFlag = 1;
		}
    }
}



volatile int stepFlag = 0;
volatile int initFlag = 1;

uint16_t maximum(uint16_t *auxVec, uint8_t size){
	uint16_t maxi = auxVec[0];
	for(int n = 1, n<=(size-1), n++){
		if(maxi <= auxVec[n]){
			maxi = auxVec[n];
		}
	}
	return maxi;
}

uint16_t minimum(uint16_t *auxVec, uint8_t size){
	uint16_t mini = auxVec[0];
	for(int n = 1, n<=(size-1), n++){
		if(mini <= auxVec[n]){
			mini = auxVec[n];
		}
	}
	return mini;
}

void tmpo(){
	uint16_t A;
	uint8_t Ax=0, Ay=0;
	uint16_t auxVec[50];
	uint16_t thsMax = 0;
	uint16_t thsMin = 20;
	uint16_t THS = 0;
	if(t2sFlag==1){
		if ((auxVec[49] != 0) && A>=THS){
			stepFlag=1;
		}
		if(initFlag == 1){
			for(int n=0, n<=49, n++){
				auxVec[n]=0;
			}
			initFlag=0;
		}	
	}
	//read acel
	A = sqrt(Ax^2+Ay^2);
	for(int n = 48, n>=0, n--){
		 auxVec[n+1] = auxVec[n];
	}
	auxVec[0] = A;
	thsMax = maximum(auxVec,50);
	thsMin = minimum(auxVec,50);
	THS = (thsMax + thsMin) / 2;
}

