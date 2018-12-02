/*
 * ola.c
 *
 * Created: 08/11/2018 20:28:37
 * Author : PC
 */ 

#include <avr/io.h>
# define F_CPU 20000000
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "MPU9250\mpu9250.h"
#include "ST7920\st7920.h"
#include "RFID\rfid.h"


volatile struct{ 			// Flags
	uint8_t rx0_on:			1;			// Indica que houve uma recepção na USART0
	uint8_t rx1_on:			1;			// Indica que houve uma recepção na USART1
	uint8_t blink_ok:		1;			// Indica que pode piscar
	uint8_t blink25_ok:	1;			// Indica que LCD está on e pode actualizar o PWM
	uint8_t tmr_on:			1;			// Indica que tem tempo a descontar
	uint8_t deb:			1;			// Tempo de debounce
	uint8_t D_val:			1;			// Dados válidos
	uint8_t all:			1;			//
}myflags;

	char		PC_buffer[30];									// Vetor com os dados a transmitir para o PC
	char		Bluetooth_buffer[30];
	volatile char usart0_data;									// Variável com o caracter recebido na USART0
	volatile char usart1_data;									// Variável com o caracter recebido na USART1
	volatile unsigned char cont		= 0;
	volatile unsigned char cont25	= 0;
	
	volatile unsigned char ret0;								// I2C usage
	float ax, ay, az;
	float gx, gy, gz;
	char tudo [5];
	char tudo1[5];
	char tudo2[5];
	char tudo3[7];
	char tudo4[7];
	char tudo5[7];
	char tudo6[7];
	char tudo7[7];
	float inclinacao;
	float rotacao;
	float valorteste = 57.30;					//180/PI;	
	
	uint8_t bytemain;
	uint8_t str[MAX_LEN];
	
/* Confgurações iniciais do sistema */
void start(){
	DDRB |= (1<<PORTB0)|(1<<PORTB1)|(1<<PORTB2); // PB0
	
	//USART0
	UBRR0H	=	0x00;										// Setup USART0
	UBRR0L	=	0x2A;										// BaudRate 57600
	UCSR0A	=	0b00000010;									// U2Xn = 1,((F_CPU/(8*B_Rate_1))-1)
	UCSR0B	=	0b10011000;									// 8N1
	UCSR0C	=	0b00000110;									//
	
	//USART1
	UBRR1H	=	0x00;										// Setup USART1 BaudRate 115200
	UBRR1L	=	0x81;										// U2Xn = 0, ((F_CPU/(16*B_Rate_1))-1)
	UCSR1A	=	0b00000000;									// 8N1
	UCSR1B	=	0b10011000;									//
	UCSR1C	=	0b00000110;									//
	
	//TIMER0
	OCR0A		=	0xC2;									// Valor para comparação no timer0
	TCCR0A	=	0b10000010;									// Configurações timer0 = 2.5ms
	TCCR0B	=	0b00000100;									// CTC mode 256 presacaler
	TIMSK0 |=	0b00000010;									// enable T0
	
	TWSR = 0;												// no prescaler
	TWBR = ((20000000/400000)-16)/2;						// must be > 10 for stable operation ((F_CPU/SCL_CLOCK)-16)/2
	_gres=0.0076;											// 250.0/32767.0; MPU9250_GFS_250 (Existe ainda 500, 1000, 2000)
	_ares=0.000061;											//2.0/32767.0; MPU9250_AFS_2G (Existe ainda 4G, 8G, 16G)

		
	SREG   |=	0b10000000;									// Enable Global Interrupt
}

/* Interrupt Handler Timer 0 */
ISR(TIMER0_COMPA_vect){										//contador de 500ms
	cont++;	
	cont25++;												// incrementa contador
	if (cont==200)	{										// 200*2.5=500ms
		cont=0;												// limpar variavel de tempo
		myflags.blink_ok=1;
	}
	if (cont25==25)	{										// 200*2.5=500ms
		cont25=0;												// limpar variavel de tempo
		myflags.blink25_ok=1;
	}
	
}

/* Interrupt Handler da USART 0 */
ISR(USART0_RX_vect){
	myflags.rx0_on=	1;										// turn on flag on (data in buffer)
	usart0_data		=	UDR0;								// save data on here
}

/* INTERRUPT  HANDLER DA USART 1*/
ISR(USART1_RX_vect){
	myflags.rx1_on=1;
	usart1_data		=	UDR1;
}

void send_PC(){
	unsigned char i=0;										// Variável local, apontador de bytes
	while(PC_buffer[i]!='\0'){								// enquanto houver bytes para envio
		while((UCSR0A & 1<<UDRE0)==0);						// esperar pelo envio de 1 byte
		UDR0=PC_buffer[i];									// próximo byte
		i++;												// incrementa apontador
	}
}

void send_Bluetooth(){
	unsigned char i=0;										// Variável local, apontador de bytes
	while(Bluetooth_buffer[i]!='\0'){								// enquanto houver bytes para envio
		while((UCSR1A & 1<<UDRE1)==0);						// esperar pelo envio de 1 byte
		UDR1=Bluetooth_buffer[i];									// próximo byte
		i++;												// incrementa apontador
	}
}

/* Identificação do trabalho*/
void identificar()	{
	sprintf(PC_buffer, "\r\n PEDOMETRO \r\n");		//rs232
	send_PC();
	sprintf(PC_buffer, "1140205@isep.ipp.pt - Andre Marques\r\n\n");			//rs232
	send_PC();
	sprintf(PC_buffer, "1141286@isep.ipp.pt - Thor Jose Struck\r\n\n");			//rs232
	send_PC();
	lcd_clear_all();
	lcd_set_cursor(0,0);
	lcd_send_str("  Andre Marques");
	lcd_set_cursor(1,0);
	lcd_send_str("  N': ");
	lcd_send_str("1140205");
	lcd_set_cursor(2,0);
	lcd_send_str("  Thor Struck");
	lcd_set_cursor(3,0);
	lcd_send_str("  N': ");
	lcd_send_str("1141286");
	_delay_ms(6000);
	lcd_clear_all();
}

int main(void){
    lcd_init();											//	Configurar o LCD
	start();
	identificar();

	//I2C
	ret0 = i2c_rep_start(MPU9250_ADDR+I2C_WRITE);			// set device address and write mode
	if ( ret0 ){											// failed to issue start condition, possibly no device found
		i2c_stop();
		sprintf(Bluetooth_buffer, "Falhou o endereco %d\r\n\n", MPU9250_ADDR);
		send_Bluetooth();
	}
	else{
		sprintf(Bluetooth_buffer, "ENTROU %d\r\n\n", MPU9250_ADDR);
		send_Bluetooth();
		i2c_write(0x6B);									// write address = 5
		i2c_write(0);										// write value 0x75 to EEPROM
		i2c_stop();											// set stop conditon = release bus
	}
	////////////////////////////////////////////////////////////////

	while (1) 
    {
		readAccelxyz(&ax, &ay, &az);
		
		if (myflags.blink_ok==1){
			myflags.blink_ok=0;
			PORTB ^= (1<<PORTB0);	// pull up activa
		
		
			/* ************************** ACELERÓMETRO ********************** */
			dtostrf(ax, 4, 2, tudo);					// converter em string
			dtostrf(ay, 4, 2, tudo1);					// converter em string
			dtostrf(az, 3, 2, tudo2);					// converter em string
			
			sprintf(Bluetooth_buffer, "ax =%s\r\n", tudo);
			send_Bluetooth();
			sprintf(Bluetooth_buffer, "ay =%s\r\n", tudo1);
			send_Bluetooth();
			sprintf(Bluetooth_buffer, "az = %s\r\n", tudo2);
			send_Bluetooth();
		
		}
				/* ********************* rotação ************************** */
		/*		float raiz1;
				raiz1=sqrt(az*az + ax*ax);
				rotacao=atan2(ay,raiz1)*valorteste;
				dtostrf(rotacao, 4,2,tudo7);
				sprintf(PC_buffer, "rotacao = %s\r\n", tudo7);
				send_PC();
		}*/
		
	}
}


