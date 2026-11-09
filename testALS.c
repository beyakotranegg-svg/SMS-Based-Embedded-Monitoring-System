/* 
 * File:   testALS.c
 * Author: beyak
 *
 * Created on November 25, 2025, 4:08 PM
 */
#include <stdio.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// Chip Select pin (Arduino digital pin 10 = PB2)
#define CS PB2

void SPI_MasterInit(void)
{
    // Set MOSI (PB3), SCK (PB5) and CS (PB2) as OUTPUT
    DDRB |= (1 << PB3) | (1 << PB5) | (1 << CS);

    // Enable SPI, set as Master
    // SPI mode 0, clock = F_CPU/16 = 1MHz
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t SPI_Transfer(uint8_t data)
{
    SPDR = data;               // Send data
    while (!(SPSR & (1 << SPIF)));  // Wait for transfer complete
    return SPDR;               // Return received byte
}

int main(void)
{
    uint8_t intensity;
    uint8_t high, low;

    SPI_MasterInit();

    // Initialize UART for display (9600 baud)
    // UBRR = 103 for 16 MHz ? 9600 bps
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B |= (1 << TXEN0);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

    while (1)
    {
        // Pull CS low to start communication
        PORTB &= ~(1 << CS);

        // Same as Arduino : shift and merge bits
        high = SPI_Transfer(0) << 3;
        low  = SPI_Transfer(0) >> 4;

        intensity = high | low;

        // CS HIGH
        PORTB |= (1 << CS);

        // Send intensity over UART
        char buffer[20];
        sprintf(buffer, "Light=%u\r\n", intensity);

        for (int i = 0; buffer[i]; i++)
        {
            while (!(UCSR0A & (1 << UDRE0))); // Wait UART ready
            UDR0 = buffer[i];
        }

        _delay_ms(1000);
    }
}


