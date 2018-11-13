/*
 * rfid.h
 *
 * Created: 28/10/2017 12:36:07
 *  Author: André Marques
 */ 


#ifndef RFID_H_
#define RFID_H_

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SPI_PIN PINB
#define SPI_MISO PB6
#define SPI_MOSI PB5
#define SPI_SCK	 PB7
#define SPI_SS   PB4

#define ENABLE_CHIP() (SPI_PORT &= (~ (1<<SPI_SS) ) )
#define DISABLE_CHIP() (SPI_PORT |= (1<<SPI_SS) )
#define CommandReg			0x01
#define SoftReset_CMD		0x0F
#define TModeReg			0x2A
#define TPrescalerReg		0x2B
#define TReloadReg_1		0x2C
#define TReloadReg_2		0x2D
#define TxASKReg			0x15
#define	ModeReg				0x11
#define TxControlReg		0x14
#define VersionReg			0x37
#define ComIEnReg			0x02
#define DivIEnReg			0x03
#define BitFramingReg		0x0D
#define MFAuthent_CMD		0x0E
#define Transceive_CMD		0x0C
#define ComIrqReg			0x04
#define FIFOLevelReg		0x0A
#define Idle_CMD			0x00
#define FIFODataReg			0x09
#define ControlReg			0x0C
#define ErrorReg			0x06
#define PICC_REQALL			0x52
#define PICC_ANTICOLL		0X93

#define NR_OF_CARDS  2
#define CARDS_SIZES  10
#define MAX_LEN 16
#define ERROR	3
#define CARD_NOT_FOUND 2
#define CARD_FOUND 1


/**********************FUNÇOES**********************/
void spi_init();
uint8_t spi_transmit(uint8_t data);
void mfrc522_write(uint8_t reg, uint8_t data);
void mfrc522_reset();
uint8_t mfrc522_read(uint8_t reg);
void mfrc522_init();
uint8_t mfrc522_to_card(uint8_t cmd, uint8_t *send_data, uint8_t send_data_len, uint8_t *back_data, uint32_t *back_data_len);
uint8_t mfrc522_request(uint8_t req_mode, uint8_t *tag_type);
uint8_t mfrc522_get_card_serial(uint8_t * serial_out);
void ledverde();
void ledvermelho();


#endif /* RFID_H_ */