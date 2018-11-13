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
	uint8_t lcd_on:			1;			// Indica que LCD está on e pode actualizar o PWM
	uint8_t tmr_on:			1;			// Indica que tem tempo a descontar
	uint8_t deb:			1;			// Tempo de debounce
	uint8_t D_val:			1;			// Dados válidos
	uint8_t all:			1;			//
}myflags;

	char		PC_buffer[30];									// Vetor com os dados a transmitir para o PC
	volatile char usart0_data;									// Variável com o caracter recebido na USART0
	volatile unsigned char cont		= 0;
	
	volatile unsigned char ret0;								// I2C usage
	float ax, ay, az;
	float gx, gy, gz;
	char tudo [5];
	char tudo1[5];
	char tudo2[5];
	char tudo3[7];
	char tudo4[7];
	char tudo5[7];
	
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
	cont++;													// incrementa contador
	if (cont==200)	{										// 200*2.5=500ms
		cont=0;												// limpar variavel de tempo
		myflags.blink_ok=1;
	}
}

/* Interrupt Handler da USART 0 */
ISR(USART0_RX_vect){
	myflags.rx0_on=	1;										// turn on flag on (data in buffer)
	usart0_data		=	UDR0;								// save data on here
}


void send_PC(){
	unsigned char i=0;										// Variável local, apontador de bytes
	while(PC_buffer[i]!='\0'){								// enquanto houver bytes para envio
		while((UCSR0A & 1<<UDRE0)==0);						// esperar pelo envio de 1 byte
		UDR0=PC_buffer[i];									// próximo byte
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

void printAcelData(){
	/* ************************** ACELERÓMETRO ********************** */
	readAccelxyz(&ax, &ay, &az);
	dtostrf(ax, 4, 2, tudo);					// converter em string
	dtostrf(ay, 4, 2, tudo1);					// converter em string
	dtostrf(az, 3, 2, tudo2);					// converter em string
	
	lcd_set_cursor(0,0);
	lcd_send_str("ax= ");
	lcd_send_str(tudo);
	lcd_set_cursor(1,0);
	lcd_send_str("ay= ");
	lcd_send_str(tudo1);
	lcd_set_cursor(2,0);
	lcd_send_str("az= ");
	lcd_send_str(tudo2);
}

int main(void){
    lcd_init();											//	Configurar o LCD
	start();
	identificar();

	//I2C
	ret0 = i2c_rep_start(MPU9250_ADDR+I2C_WRITE);			// set device address and write mode
	if ( ret0 ){											// failed to issue start condition, possibly no device found
		i2c_stop();
		sprintf(PC_buffer, "Falhou o endereco %d\r\n\n", MPU9250_ADDR);
		send_PC();
	}
	else{
		sprintf(PC_buffer, "ENTROU %d\r\n\n", MPU9250_ADDR);
		send_PC();
		i2c_write(0x6B);									// write address = 5
		i2c_write(0);										// write value 0x75 to EEPROM
		i2c_stop();											// set stop conditon = release bus
	}
	////////////////////////////////////////////////////////////////
	
	//RFID
/*	lcd_clear_all();
	lcd_set_cursor(0,0);
	lcd_send_str("   Passe a tag");
	lcd_set_cursor(1,1);
	lcd_send_str(" Pelo leitor ");
	lcd_set_cursor(2,0);
	lcd_send_str("   Por favor");
	_delay_ms(2500);
	*/
	spi_init();
	mfrc522_init();
	bytemain = mfrc522_read(VersionReg);
	sprintf(PC_buffer, "UID tag = %d \r\n", bytemain);
	send_PC();
	bytemain = mfrc522_read(ComIEnReg);
	mfrc522_write(ComIEnReg, bytemain | 0x20);
	bytemain = mfrc522_read(DivIEnReg);
	mfrc522_write(DivIEnReg, bytemain | 0x80);
	
	while (1) 
    {
				bytemain = mfrc522_request(PICC_REQALL, str);
				//		sprintf(PC_buffer, "%d\r\n", bytemain);
				//		send_PC();
				
				if (bytemain == 1){
					bytemain = mfrc522_get_card_serial(str);
				//	sprintf(PC_buffer, "TAG = ");
				//	send_PC();
					for (uint8_t t = 0; t<5; t++){
				//		sprintf(PC_buffer, "%X ", str[t]);
				//		send_PC();
					}
				//	sprintf(PC_buffer, "\r\n");
				//	send_PC();
				}
				
				switch (str[0]){
					case 0xE6:
					lcd_clear_all();
					ledverde();
					lcd_set_cursor(0,0);
					lcd_send_str("    Acesso ");
					lcd_set_cursor(1,1);
					lcd_send_str(" Autorizado ");
					lcd_set_cursor(3,0);
					lcd_send_str(" Andre Marques");
					_delay_ms(2500);
					
					//				sprintf(PC_buffer, "Acesso Autorizado - Andre Marques\r\n");
					//				send_PC();
					
					break;
					case 0xBB:
					lcd_clear_all();
					ledvermelho();
					lcd_set_cursor(1,0);
					lcd_send_str(" Acesso Negado");
					_delay_ms(2500);
					// 				sprintf(PC_buffer, "Acesso Negado\r\n");
					// 				send_PC();
					break;
					default:
					break;
				}
		
		
		
		if (myflags.blink_ok==1){
			myflags.blink_ok=0;
			PORTB ^= (1<<PORTB0);	// pull up activa
			PORTB ^= (1<<PORTB1);	// pull up activa
			PORTB ^= (1<<PORTB2);	// pull up activa	
		
		
	/*		readGyroxyz(&gx, &gy, &gz);
			dtostrf(gx, 4, 2, tudo3);					// converter em string
			dtostrf(gy, 4, 2, tudo4);					// converter em string
			dtostrf(gz, 4, 2, tudo5);					// converter em string
			
			sprintf(PC_buffer, "gx = %s\r\n", tudo3);
			send_PC();
			sprintf(PC_buffer, "gy = %s\r\n", tudo4);
			send_PC();
			sprintf(PC_buffer, "gz = %s\r\n", tudo5);
			send_PC();
			
			
			
			sprintf(PC_buffer, "ax =%s\r\n", tudo);
			send_PC();
			sprintf(PC_buffer, "ay =%s\r\n", tudo1);
			send_PC();
			sprintf(PC_buffer, "az = %s\r\n", tudo2);
			send_PC();
			*/
	
			/* ********************* inclinação *********************** */
/*			float raiz;
			float raiz1;
			
			raiz=sqrt(ay*ay + az*az);
			inclinacao=atan2(ax,raiz)*valorteste;
			dtostrf(inclinacao, 4,2,tudo6);
			
			if(inclinacao >= 5){
				sprintf(PC_buffer, "inclinacao = %s - subir \r\n", tudo6);
				send_PC();
			}
			if((inclinacao > -5) & (inclinacao < 5)){
				sprintf(PC_buffer, "inclinacao=%s - plano \r\n", tudo6);
				send_PC();
			}
			if(inclinacao <= -5){
				sprintf(PC_buffer, "inclinacao=%s - descer \r\n", tudo6);
				send_PC();
			}*/
			
			
			/*ACELEROMETRO*/
			
			printAcelData();
			
			/* ********************* rotação ************************** */
	/*		raiz1=sqrt(az*az + ax*ax);
			rotacao=atan2(ay,raiz1)*valorteste;
			dtostrf(rotacao, 4,2,tudo7);
			sprintf(PC_buffer, "rotacao = %s\r\n", tudo7);
			send_PC();
			
			if (rotacao <= -5){
				sprintf(PC_buffer, "rotacao = %s - esquerda\r\n", tudo7);
				send_PC();
				
			}
			if (rotacao > 5){
				sprintf(PC_buffer, "rotacao = %s - direita\r\n", tudo7);
				send_PC();
			}
			if((rotacao > -5) & (rotacao < 5)){
				sprintf(PC_buffer, "rotacao = %s - frente\r\n", tudo7);
				send_PC();
			}
			
				
		*/}
		
	}
}


