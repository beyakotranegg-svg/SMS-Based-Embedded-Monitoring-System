#define F_CPU 16000000UL
#define __AVR_ATmega328P__
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

/* ---------- UART ---------- */

void UART_init(unsigned long baud) {
    unsigned int ubrr = F_CPU/16/baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);  // Activer TX et RX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits, 1 stop bit, pas de parité
}

void UART_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));  // Attendre buffer vide
    UDR0 = c;
}

char UART_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));   // Attendre données reçues
    return UDR0;
}

void UART_print(const char *s) {
    while (*s) {
        UART_send(*s++);
    }
}

/* ---------- GSM ---------- */

int GSM_waitForResponse(const char *expected, int timeout_ms) {
    char response[64] = {0};
    int index = 0;
    unsigned long start_time = 0;  // Vous devrez implémenter un timer
    
    while (1) {
        if (UCSR0A & (1 << RXC0)) {
            char c = UDR0;
            if (index < 63) {
                response[index++] = c;
            }
            
            // Vérifier si réponse contient "OK" ou "ERROR"
            if (strstr(response, expected) != NULL) {
                return 1;  // Succès
            }
            if (strstr(response, "ERROR") != NULL) {
                return 0;  // Erreur
            }
        }
        _delay_ms(1);
        // Implémenter timeout ici
    }
}

void GSM_sendCommand(const char *cmd) {
    UART_print(cmd);
    UART_print("\r\n");
    _delay_ms(500);
    
    // Attendre réponse
    GSM_waitForResponse("OK", 2000);
}

void GSM_sendSMS(const char *number, const char *message) {
    // Initialisation
    GSM_sendCommand("AT");
    _delay_ms(1000);
    
    // Mode texte
    GSM_sendCommand("AT+CMGF=1");
    _delay_ms(1000);
    
    // Numéro avec délai
    UART_print("AT+CMGS=\"");
    UART_print(number);
    UART_print("\"\r\n");
    _delay_ms(2000);  // IMPORTANT: Délai avant message
    
    // Message
    UART_print(message);
    
    // CTRL+Z pour envoyer (0x1A en hexa)
    UART_send(0x1A);
    
    // Long délai pour envoi SMS
    _delay_ms(10000);
}

/* ---------- MAIN ---------- */

int main(void) {
    // Initialisation UART
    UART_init(9600);
    
    // LONG délai pour démarrage GSM
    _delay_ms(15000);  // 15 secondes
    
    // Configurer une LED pour debug (PB5 = Arduino D13)
    DDRB |= (1 << PB5);
    PORTB |= (1 << PB5);  // LED allumée
    
    // Envoyer SMS
    GSM_sendSMS("+21697667215", "Test SMS");
    
    // Clignoter LED pour indiquer fin
    while (1) {
        PORTB ^= (1 << PB5);
        _delay_ms(500);
    }
    
    return 0;
}