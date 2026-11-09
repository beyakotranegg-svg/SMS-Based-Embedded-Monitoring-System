#define F_CPU 16000000UL  // fréquence CPU 16 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Définition des pins
#define CS PB2        // Chip select sur PB2 (Digital 10 Arduino)
#define BUTTON PD2    // Bouton sur PD2 (Digital 2 Arduino)

// Fonction d'initialisation SPI en mode maître
void SPI_MasterInit(void)
{
    DDRB |= (1 << PB5) | (1 << PB3) | (1 << CS); // MOSI, SCK et CS en sortie
    DDRB &= ~(1 << PB4); // MISO en entrée

    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // SPI enable, master, f_osc/16
}

// Fonction pour transférer un octet via SPI
uint8_t SPI_Transmit(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF))); // attendre la fin de transmission
    return SPDR;
}

// Initialisation UART pour debug
void UART_Init(uint16_t ubrr)
{
    UBRR0H = (uint8_t)(ubrr>>8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << TXEN0);   // TX enable
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits data
}

// Fonction pour envoyer un caractère via UART
void UART_Transmit(char data)
{
    while (!(UCSR0A & (1 << UDRE0))); // attendre buffer libre
    UDR0 = data;
}

// Fonction pour envoyer une chaîne via UART
void UART_Print(const char *str)
{
    while(*str)
        UART_Transmit(*str++);
}

// Fonction pour envoyer un entier (0-255) via UART
void UART_PrintInt(uint8_t val)
{
    char buffer[4];
    sprintf(buffer, "%d", val);
    UART_Print(buffer);
}

int main(void)
{
    // Initialisations
    DDRD &= ~(1 << BUTTON); // bouton en entrée
    PORTD |= (1 << BUTTON); // pull-up interne activée

    SPI_MasterInit();
    UART_Init(103); // 9600 bps pour F_CPU=16MHz

    // Mettre CS à HIGH par défaut
    PORTB |= (1 << CS);

    while (1)
    {
        // Si le bouton est appuyé (PD2 = 0)
        if (!(PIND & (1 << BUTTON)))
        {
            uint8_t intensity = 0;

            // Activation CS
            PORTB &= ~(1 << CS);

            // Lecture SPI
            intensity = SPI_Transmit(0) << 3;     // premiers 5 bits
            intensity |= (SPI_Transmit(0) >> 4);  // derniers 3 bits

            // Désactivation CS
            PORTB |= (1 << CS);

            // Affichage sur UART
            UART_Print("Light intensity = ");
            UART_PrintInt(intensity);
            UART_Print("\r\n");

            _delay_ms(200); // anti-rebond et délai d'envoi
        }
    }
}
