#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define DHT_PORT PORTD
#define DHT_DDR  DDRD
#define DHT_PIN  PIND
#define DHT_BIT  PD2

void UART_init() {
    UBRR0H = 0;
    UBRR0L = 103;     // 9600 baud @ 16MHz
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8N1
}

void UART_send(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

void UART_print(char *s) {
    while (*s) UART_send(*s++);
}

void DHT_start() {
    DHT_DDR |= (1<<DHT_BIT);
    DHT_PORT &= ~(1<<DHT_BIT);
    _delay_ms(18);

    DHT_PORT |= (1<<DHT_BIT);
    _delay_us(30);

    DHT_DDR &= ~(1<<DHT_BIT);
}

uint8_t DHT_check() {
    _delay_us(40);
    if (!(DHT_PIN & (1<<DHT_BIT))) {
        _delay_us(80);
        if (DHT_PIN & (1<<DHT_BIT)) {
            _delay_us(80);
            return 1;
        }
    }
    return 0;
}

uint8_t DHT_readByte() {
    uint8_t data = 0;
    for (uint8_t i=0; i<8; i++) {
        while (!(DHT_PIN & (1<<DHT_BIT)));
        _delay_us(40);

        if (DHT_PIN & (1<<DHT_BIT))
            data |= (1 << (7 - i));

        while (DHT_PIN & (1<<DHT_BIT));
    }
    return data;
}

int main() {
    UART_init();
    char txt[50];

    uint8_t H_int, H_dec, T_int, T_dec, chk;

    while (1) {
        DHT_start();
        if (DHT_check()) {

            H_int = DHT_readByte();
            H_dec = DHT_readByte();
            T_int = DHT_readByte();
            T_dec = DHT_readByte();
            chk   = DHT_readByte();

            if ((H_int + H_dec + T_int + T_dec) == chk) {

                sprintf(txt, "Temp=%d.%dC  Hum=%d.%d%%\r\n",
                        T_int, T_dec, H_int, H_dec);
                UART_print(txt);

            } else {
                UART_print("Erreur checksum\r\n");
            }
        } else {
            UART_print("Pas de reponse du DHT11\r\n");
        }

        _delay_ms(1000);
    }
}
