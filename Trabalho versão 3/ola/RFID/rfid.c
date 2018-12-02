
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
# define F_CPU 20000000
#include <util/delay.h>
#include "rfid.h"

void spi_init(){
	SPI_DDR |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<SPI_SS); //set mosi sck ss output, and others input
	SPCR0 |= (1<<SPE0) | (1<<MSTR0) | (1<<SPR00);
}

uint8_t spi_transmit(uint8_t data){
	SPDR0 = data;
	
	while (! (SPSR0 & (1<<SPIF0) ) );
	return SPDR0;
}

void mfrc522_write(uint8_t reg, uint8_t data){
	ENABLE_CHIP();
	spi_transmit((reg<<1) & 0x7E);
	spi_transmit(data);
	DISABLE_CHIP();
}

void mfrc522_reset(){
	mfrc522_write(CommandReg, SoftReset_CMD);
}

uint8_t mfrc522_read(uint8_t reg){
	uint8_t data;
	
	ENABLE_CHIP();
	spi_transmit(((reg<<1) & 0x7E) | 0x80);
	data = spi_transmit(0x00);
	DISABLE_CHIP();
	return data;
}

void mfrc522_init(){
	uint8_t byte;
	mfrc522_reset();
	mfrc522_write(TModeReg, 0x8D);
	mfrc522_write(TPrescalerReg, 0x3E);
	mfrc522_write(TReloadReg_1, 30);
	mfrc522_write(TReloadReg_2, 0);
	mfrc522_write(TxASKReg, 0x40);
	mfrc522_write(ModeReg, 0x3D);
		
	byte = mfrc522_read(TxControlReg);
	if(! (byte & 0x03) ){
		mfrc522_write(TxControlReg, byte | 0x03);
	}
}

uint8_t mfrc522_to_card(uint8_t cmd, uint8_t *send_data, uint8_t send_data_len, uint8_t *back_data, uint32_t *back_data_len){
	uint8_t status = ERROR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint8_t tmp;
	uint32_t i;
	
	switch (cmd){
		case MFAuthent_CMD:
		{	irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case Transceive_CMD:
		{	irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}
	
	n=mfrc522_read(ComIrqReg);
	mfrc522_write(ComIrqReg, n&(~0x80) );
	n=mfrc522_read(FIFOLevelReg);
	mfrc522_write(FIFOLevelReg, n|0x80);
	
	mfrc522_write(CommandReg, Idle_CMD);
	
	for(i=0; i<send_data_len; i++){
		mfrc522_write(FIFODataReg, send_data[i]);
	}
	
	mfrc522_write(CommandReg, cmd);
	if (cmd == Transceive_CMD){
		n=mfrc522_read(BitFramingReg);
		mfrc522_write(BitFramingReg, n|0x80);
	}
	
	i=2000;
	do{
		n=mfrc522_read(ComIrqReg);
		i--;
	}
	
	while((i!=0) && !(n & 0x01) && !(n&waitIRq));
	
	tmp=mfrc522_read(BitFramingReg);
	mfrc522_write(BitFramingReg, tmp&(~0x80));
	
	if(i!= 0){
		if (! (mfrc522_read(ErrorReg) & 0x1B)){
			status = CARD_FOUND;
			if(n & irqEn & 0x01){
				status = CARD_NOT_FOUND;
			}
			if (cmd == Transceive_CMD){
				n = mfrc522_read(FIFOLevelReg);
				lastBits = mfrc522_read(ControlReg) & 0x07;
				if(lastBits){
					*back_data_len = (n-1)*8 + lastBits;
				}
				else{
					*back_data_len = n*8;
				}
				if (n==0){
					n=1;
				}
				if (n>MAX_LEN){
					n=MAX_LEN;
				}
				for (i=0; i<n; i++){
					back_data[i] = mfrc522_read(FIFODataReg);
				}
			}
		}
		else{
			status = ERROR;
		}
	}
	return status;
}

uint8_t mfrc522_request(uint8_t req_mode, uint8_t *tag_type){
	uint8_t status;
	uint32_t backBits;
	
	mfrc522_write(BitFramingReg, 0x07);
	
	tag_type[0] = req_mode;
	status = mfrc522_to_card(Transceive_CMD, tag_type, 1, tag_type, &backBits);
	
	if ((status != CARD_FOUND) || (backBits != 0x10)){
		status = ERROR;
	}
	return status;
}

uint8_t mfrc522_get_card_serial(uint8_t * serial_out){
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint32_t unLen;
	
	mfrc522_write(BitFramingReg, 0x00);
	
	serial_out[0] = PICC_ANTICOLL;
	serial_out[1] = 0x20;
	status = mfrc522_to_card(Transceive_CMD, serial_out,2, serial_out, &unLen);
	
	if (status == CARD_FOUND){
		for(i=0; i<4; i++){
			serNumCheck ^= serial_out[i];
		}
		if (serNumCheck != serial_out[i]){
			status = ERROR;
		}
	}
	return status;
}

void ledverde(){
	DDRC = (1<<PORTC6);
	PORTC = (1<<PORTC6);
	_delay_ms(3000);
	PORTC = (0<<PORTC6);
}

void ledvermelho(){
	DDRC = (1<<PORTC7);
	PORTC = (1<<PORTC7);
	_delay_ms(500);
	PORTC = (0<<PORTC7);
	_delay_ms(500);
	PORTC = (1<<PORTC7);
	_delay_ms(500);
	PORTC = (0<<PORTC7);
	_delay_ms(500);
	PORTC = (1<<PORTC7);
	_delay_ms(500);
	PORTC = (0<<PORTC7);
}