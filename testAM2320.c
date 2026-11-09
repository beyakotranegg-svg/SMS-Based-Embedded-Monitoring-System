#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define AM2320_ADDR 0x5C

// =================== I2C ===================

void I2C_Init(void) {
    TWSR = 0x00; // prescaler = 1
    TWBR = 72;   // SCL = 100kHz pour F_CPU = 16MHz
}

void I2C_Start(void) {
    TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void I2C_Stop(void) {
    TWCR = (1<<TWSTO)|(1<<TWEN)|(1<<TWINT);
    _delay_us(10);
}

void I2C_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN)|(1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

uint8_t I2C_Read(uint8_t ack) {
    if (ack)
        TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA);
    else
        TWCR = (1<<TWEN)|(1<<TWINT);

    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

// =================== CRC ===================
uint16_t CRC16(uint8_t *ptr, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for(uint8_t j=0; j<length; j++) {
        crc ^= ptr[j];
        for(uint8_t i=0; i<8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// =================== AM2320 ===================
uint8_t buffer[8];

void AM2320_Read(void) {
    // Réveil du capteur
    I2C_Start();
    I2C_Write((AM2320_ADDR<<1)|0);
    I2C_Stop();
    _delay_ms(2);

    // Commande lecture registre 0x00, longueur 4
    I2C_Start();
    I2C_Write((AM2320_ADDR<<1)|0);
    I2C_Write(0x03); // fonction
    I2C_Write(0x00); // registre départ
    I2C_Write(0x04); // longueur
    I2C_Stop();
    _delay_ms(2);

    // Lecture des données (7 octets + CRC)
    I2C_Start();
    I2C_Write((AM2320_ADDR<<1)|1);
    for(uint8_t i=0; i<7; i++)
        buffer[i] = I2C_Read(1); // ACK
    buffer[7] = I2C_Read(0);     // NACK
    I2C_Stop();
}

// =================== UART ===================
void UART_Init(unsigned long baud) {
    uint16_t ubrr = F_CPU/16/baud-1;
    UBRR0H = (ubrr>>8);
    UBRR0L = ubrr;
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void UART_SendChar(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

void UART_SendString(const char* s) {
    while(*s) UART_SendChar(*s++);
}

void UART_SendNumber(int16_t num) {
    char buffer[6];
    sprintf(buffer, "%d", num);
    UART_SendString(buffer);
}

// =================== MAIN ===================
int main(void) {
    uint16_t hum, temp;
    uint16_t crc_calc, crc_read;

    I2C_Init();
    UART_Init(9600);

    while(1) {
        AM2320_Read();

        crc_calc = CRC16(buffer,6);
        crc_read = buffer[6] | (buffer[7]<<8);

        if(crc_calc == crc_read) {
            hum = (buffer[2]<<8) | buffer[3];
            temp = (buffer[4]<<8) | buffer[5];

            UART_SendString("Hum: ");
            UART_SendNumber(hum/10);
            UART_SendChar('.');
            UART_SendNumber(hum%10);
            UART_SendString(" %  ");

            UART_SendString("Temp: ");
            UART_SendNumber(temp/10);
            UART_SendChar('.');
            UART_SendNumber(temp%10);
            UART_SendString(" C\r\n");
        } else {
            UART_SendString("Erreur CRC\r\n");
        }

        _delay_ms(2000);
    }
}
