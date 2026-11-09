/*
 * Projet: Système de surveillance multi-capteurs avec RFID et GSM
 * MCU: ATmega328P
 * Compilateur: AVR-GCC (MPLAB X)
 * 
 * Fonctionnalités:
 * - Lecture RFID (MFRC522)
 * - Communication GSM (SIM900A)
 * - Capteur température/humidité (DHT11)
 * - Capteur CO2 (analogique)
 * - Capteur luminosité (ALS via SPI)
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

// ===== DÉFINITIONS DES PINS =====
// GSM (UART Software sur D4/D5)
#define GSM_RX_PORT PORTD
#define GSM_RX_DDR  DDRD
#define GSM_RX_PIN  PIND
#define GSM_RX_BIT  PD4

#define GSM_TX_PORT PORTD
#define GSM_TX_DDR  DDRD
#define GSM_TX_BIT  PD5

// SPI Hardware
#define SPI_DDR     DDRB
#define SPI_PORT    PORTB
#define MOSI        PB3
#define MISO        PB4
#define SCK         PB5

// Chip Selects
#define RFID_SS_DDR   DDRB
#define RFID_SS_PORT  PORTB
#define RFID_SS_BIT   PB2    // D10

#define RFID_RST_DDR  DDRD
#define RFID_RST_PORT PORTD
#define RFID_RST_BIT  PD7    // D7

#define ALS_CS_DDR    DDRB
#define ALS_CS_PORT   PORTB
#define ALS_CS_BIT    PB1    // D9

// DHT11
#define DHT_DDR     DDRD
#define DHT_PORT    PORTD
#define DHT_PIN     PIND
#define DHT_BIT     PD2      // D2

// CO2 Analog
#define CO2_CHANNEL 0        // A0 (ADC0)

// ===== COMMANDES MFRC522 =====
#define MFRC522_COMMAND_REG     0x01
#define MFRC522_COML_EN_REG     0x02
#define MFRC522_DIVL_EN_REG     0x03
#define MFRC522_COMM_IRQ_REG    0x04
#define MFRC522_DIV_IRQ_REG     0x05
#define MFRC522_ERROR_REG       0x06
#define MFRC522_STATUS1_REG     0x07
#define MFRC522_STATUS2_REG     0x08
#define MFRC522_FIFO_DATA_REG   0x09
#define MFRC522_FIFO_LEVEL_REG  0x0A
#define MFRC522_CONTROL_REG     0x0C
#define MFRC522_BIT_FRAMING_REG 0x0D
#define MFRC522_MODE_REG        0x11
#define MFRC522_TX_CONTROL_REG  0x14
#define MFRC522_TX_ASK_REG      0x15
#define MFRC522_CRC_RESULT_H    0x21
#define MFRC522_CRC_RESULT_L    0x22
#define MFRC522_TIMER_MODE_REG  0x2A
#define MFRC522_TIMER_PRESCALER 0x2B
#define MFRC522_TIMER_RELOAD_H  0x2C
#define MFRC522_TIMER_RELOAD_L  0x2D
#define MFRC522_VERSION_REG     0x37

// Commandes
#define MFRC522_IDLE            0x00
#define MFRC522_TRANSCEIVE      0x0C
#define MFRC522_SOFTRESET       0x0F

// PICC Commands
#define PICC_REQIDL             0x26
#define PICC_ANTICOLL           0x93

// ===== VARIABLES GLOBALES =====
volatile uint8_t rfidSmsEnvoye = 0;
char uartBuffer[200];
uint8_t uartIndex = 0;

// ===== PROTOTYPES =====
// UART Hardware (pour Serial Monitor)
void UART_Init(uint16_t baud);
void UART_Transmit(uint8_t data);
void UART_TransmitString(const char* str);
uint8_t UART_Receive(void);
uint8_t UART_Available(void);

// UART Software (pour GSM)
void SoftUART_Init(void);
void SoftUART_Transmit(uint8_t data);
void SoftUART_TransmitString(const char* str);

// SPI
void SPI_Init(void);
uint8_t SPI_Transfer(uint8_t data);

// MFRC522
void MFRC522_WriteRegister(uint8_t reg, uint8_t value);
uint8_t MFRC522_ReadRegister(uint8_t reg);
void MFRC522_Init(void);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* tagType);
uint8_t MFRC522_Anticoll(uint8_t* uid);
void MFRC522_Halt(void);

// DHT11
uint8_t DHT11_Start(void);
uint8_t DHT11_ReadByte(void);
uint8_t DHT11_Read(uint8_t* humidity, uint8_t* temperature);

// ADC (CO2)
void ADC_Init(void);
uint16_t ADC_Read(uint8_t channel);

// ALS (SPI)
uint8_t ALS_Read(void);

// GSM
void GSM_SendSMS(const char* numero, const char* message);
void SendSensorMessage(void);
void SendRFIDMessage(uint8_t* uid);

// Utilitaires
void delay_ms(uint16_t ms);
void uint8_to_hex(uint8_t val, char* buffer);

// ===== IMPLÉMENTATION UART HARDWARE =====
void UART_Init(uint16_t baud) {
    uint16_t ubrr = F_CPU/16/baud - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_Transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_TransmitString(const char* str) {
    while (*str) {
        UART_Transmit(*str++);
    }
}

uint8_t UART_Receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

uint8_t UART_Available(void) {
    return (UCSR0A & (1 << RXC0));
}

// Réception UART par interruption
ISR(USART_RX_vect) {
    char c = UDR0;
    if (c == '\n' || c == '\r') {
        uartBuffer[uartIndex] = '\0';
        // Ne pas réinitialiser l'index ici, on le fera dans main()
    } else if (uartIndex < sizeof(uartBuffer) - 1) {
        uartBuffer[uartIndex++] = c;
    }
}

// ===== UART SOFTWARE (GSM) =====
void SoftUART_Init(void) {
    GSM_TX_DDR |= (1 << GSM_TX_BIT);
    GSM_TX_PORT |= (1 << GSM_TX_BIT);
    GSM_RX_DDR &= ~(1 << GSM_RX_BIT);
    GSM_RX_PORT |= (1 << GSM_RX_BIT);
}

void SoftUART_Transmit(uint8_t data) {
    uint16_t bitDelay = F_CPU / 9600 / 4;
    
    // Start bit
    GSM_TX_PORT &= ~(1 << GSM_TX_BIT);
    _delay_loop_2(bitDelay);
    
    // Data bits
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x01) {
            GSM_TX_PORT |= (1 << GSM_TX_BIT);
        } else {
            GSM_TX_PORT &= ~(1 << GSM_TX_BIT);
        }
        data >>= 1;
        _delay_loop_2(bitDelay);
    }
    
    // Stop bit
    GSM_TX_PORT |= (1 << GSM_TX_BIT);
    _delay_loop_2(bitDelay);
}

void SoftUART_TransmitString(const char* str) {
    while (*str) {
        SoftUART_Transmit(*str++);
    }
}

// ===== SPI =====
void SPI_Init(void) {
    SPI_DDR |= (1 << MOSI) | (1 << SCK);
    SPI_DDR &= ~(1 << MISO);
    
    RFID_SS_DDR |= (1 << RFID_SS_BIT);
    ALS_CS_DDR |= (1 << ALS_CS_BIT);
    
    RFID_SS_PORT |= (1 << RFID_SS_BIT);
    ALS_CS_PORT |= (1 << ALS_CS_BIT);
    
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
}

uint8_t SPI_Transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

// ===== MFRC522 =====
void MFRC522_WriteRegister(uint8_t reg, uint8_t value) {
    RFID_SS_PORT &= ~(1 << RFID_SS_BIT);
    SPI_Transfer((reg << 1) & 0x7E);
    SPI_Transfer(value);
    RFID_SS_PORT |= (1 << RFID_SS_BIT);
}

uint8_t MFRC522_ReadRegister(uint8_t reg) {
    RFID_SS_PORT &= ~(1 << RFID_SS_BIT);
    SPI_Transfer(((reg << 1) & 0x7E) | 0x80);
    uint8_t value = SPI_Transfer(0x00);
    RFID_SS_PORT |= (1 << RFID_SS_BIT);
    return value;
}

void MFRC522_Init(void) {
    RFID_RST_DDR |= (1 << RFID_RST_BIT);
    RFID_RST_PORT &= ~(1 << RFID_RST_BIT);
    _delay_ms(10);
    RFID_RST_PORT |= (1 << RFID_RST_BIT);
    _delay_ms(50);
    
    MFRC522_WriteRegister(MFRC522_COMMAND_REG, MFRC522_SOFTRESET);
    _delay_ms(50);
    
    MFRC522_WriteRegister(MFRC522_TIMER_MODE_REG, 0x8D);
    MFRC522_WriteRegister(MFRC522_TIMER_PRESCALER, 0x3E);
    MFRC522_WriteRegister(MFRC522_TIMER_RELOAD_L, 30);
    MFRC522_WriteRegister(MFRC522_TIMER_RELOAD_H, 0);
    MFRC522_WriteRegister(MFRC522_TX_ASK_REG, 0x40);
    MFRC522_WriteRegister(MFRC522_MODE_REG, 0x3D);
    MFRC522_WriteRegister(MFRC522_TX_CONTROL_REG, 0x83);
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* tagType) {
    MFRC522_WriteRegister(MFRC522_BIT_FRAMING_REG, 0x07);
    tagType[0] = reqMode;
    
    MFRC522_WriteRegister(MFRC522_FIFO_LEVEL_REG, 0x80);
    MFRC522_WriteRegister(MFRC522_FIFO_DATA_REG, reqMode);
    MFRC522_WriteRegister(MFRC522_COMMAND_REG, MFRC522_TRANSCEIVE);
    MFRC522_WriteRegister(MFRC522_BIT_FRAMING_REG, 0x87);
    
    _delay_ms(10);
    
    uint8_t n = MFRC522_ReadRegister(MFRC522_FIFO_LEVEL_REG);
    if (n == 0) return 0;
    
    for (uint8_t i = 0; i < n; i++) {
        tagType[i] = MFRC522_ReadRegister(MFRC522_FIFO_DATA_REG);
    }
    
    return 1;
}

uint8_t MFRC522_Anticoll(uint8_t* uid) {
    MFRC522_WriteRegister(MFRC522_BIT_FRAMING_REG, 0x00);
    
    MFRC522_WriteRegister(MFRC522_FIFO_LEVEL_REG, 0x80);
    MFRC522_WriteRegister(MFRC522_FIFO_DATA_REG, PICC_ANTICOLL);
    MFRC522_WriteRegister(MFRC522_FIFO_DATA_REG, 0x20);
    MFRC522_WriteRegister(MFRC522_COMMAND_REG, MFRC522_TRANSCEIVE);
    MFRC522_WriteRegister(MFRC522_BIT_FRAMING_REG, 0x80);
    
    _delay_ms(10);
    
    uint8_t n = MFRC522_ReadRegister(MFRC522_FIFO_LEVEL_REG);
    if (n < 5) return 0;
    
    for (uint8_t i = 0; i < 4; i++) {
        uid[i] = MFRC522_ReadRegister(MFRC522_FIFO_DATA_REG);
    }
    
    return 1;
}

void MFRC522_Halt(void) {
    MFRC522_WriteRegister(MFRC522_COMMAND_REG, MFRC522_IDLE);
}

// ===== DHT11 =====
uint8_t DHT11_Start(void) {
    DHT_DDR |= (1 << DHT_BIT);
    DHT_PORT &= ~(1 << DHT_BIT);
    _delay_ms(18);
    
    DHT_PORT |= (1 << DHT_BIT);
    DHT_DDR &= ~(1 << DHT_BIT);
    _delay_us(40);
    
    if (DHT_PIN & (1 << DHT_BIT)) return 0;
    _delay_us(80);
    if (!(DHT_PIN & (1 << DHT_BIT))) return 0;
    _delay_us(80);
    
    return 1;
}

uint8_t DHT11_ReadByte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        while (!(DHT_PIN & (1 << DHT_BIT)));
        _delay_us(30);
        if (DHT_PIN & (1 << DHT_BIT)) {
            byte |= (1 << (7 - i));
        }
        while (DHT_PIN & (1 << DHT_BIT));
    }
    return byte;
}

uint8_t DHT11_Read(uint8_t* humidity, uint8_t* temperature) {
    if (!DHT11_Start()) return 0;
    
    uint8_t hum_int = DHT11_ReadByte();
    uint8_t hum_dec = DHT11_ReadByte();
    uint8_t temp_int = DHT11_ReadByte();
    uint8_t temp_dec = DHT11_ReadByte();
    uint8_t checksum = DHT11_ReadByte();
    
    DHT_DDR |= (1 << DHT_BIT);
    DHT_PORT |= (1 << DHT_BIT);
    
    if (checksum != (hum_int + hum_dec + temp_int + temp_dec)) {
        return 0;
    }
    
    *humidity = hum_int;
    *temperature = temp_int;
    return 1;
}

// ===== ADC =====
void ADC_Init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// ===== ALS =====
uint8_t ALS_Read(void) {
    ALS_CS_PORT &= ~(1 << ALS_CS_BIT);
    uint8_t high = SPI_Transfer(0x00);
    uint8_t low = SPI_Transfer(0x00);
    ALS_CS_PORT |= (1 << ALS_CS_BIT);
    
    uint8_t light = (high << 3) | (low >> 4);
    return light;
}

// ===== GSM =====
void GSM_SendSMS(const char* numero, const char* message) {
    SoftUART_TransmitString("AT+CMGF=1\r");
    _delay_ms(1000);
    
    SoftUART_TransmitString("AT+CMGS=\"");
    SoftUART_TransmitString(numero);
    SoftUART_TransmitString("\"\r");
    _delay_ms(1000);
    
    SoftUART_TransmitString(message);
    _delay_ms(500);
    SoftUART_Transmit(26); // Ctrl+Z
    _delay_ms(3000);
}

void SendSensorMessage(void) {
    char buffer[150];
    char temp_str[10];
    
    uint8_t light = ALS_Read();
    
    uint8_t hum, temp;
    DHT11_Read(&hum, &temp);
    
    uint16_t co2 = ADC_Read(CO2_CHANNEL);
    
    strcpy(buffer, "DONNEES CAPTEURS\n");
    strcat(buffer, "Temp: ");
    itoa(temp, temp_str, 10);
    strcat(buffer, temp_str);
    strcat(buffer, " C\nHum: ");
    itoa(hum, temp_str, 10);
    strcat(buffer, temp_str);
    strcat(buffer, " %\nLumiere: ");
    itoa(light, temp_str, 10);
    strcat(buffer, temp_str);
    strcat(buffer, "\nCO2: ");
    itoa(co2, temp_str, 10);
    strcat(buffer, temp_str);
    
    GSM_SendSMS("+21693832266", buffer);
    UART_TransmitString("SMS capteurs envoye\r\n");
}

void SendRFIDMessage(uint8_t* uid) {
    char buffer[100];
    char hex[3];
    
    strcpy(buffer, "ALERTE RFID\nCarte detectee\nUID: ");
    
    for (uint8_t i = 0; i < 4; i++) {
        uint8_to_hex(uid[i], hex);
        strcat(buffer, hex);
        strcat(buffer, " ");
    }
    
    GSM_SendSMS("+21693832266", buffer);
    UART_TransmitString("SMS RFID envoye\r\n");
}

// ===== UTILITAIRES =====
void delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

void uint8_to_hex(uint8_t val, char* buffer) {
    const char hex_chars[] = "0123456789ABCDEF";
    buffer[0] = hex_chars[val >> 4];
    buffer[1] = hex_chars[val & 0x0F];
    buffer[2] = '\0';
}

// ===== MAIN =====
int main(void) {
    // Initialisation
    UART_Init(9600);
    SoftUART_Init();
    SPI_Init();
    MFRC522_Init();
    ADC_Init();
    
    sei(); // Enable interrupts
    
    UART_TransmitString("Systeme pret\r\n");
    UART_TransmitString("Tapez 'send' pour envoyer les donnees capteurs\r\n");
    
    uint8_t tagType[2];
    uint8_t uid[4];
    
    while (1) {
        // Mode manuel - commande "send"
        if (uartBuffer[0] != '\0') {  // Si buffer contient quelque chose
            if (strstr(uartBuffer, "send")) {
                UART_TransmitString("Envoi des donnees capteurs...\r\n");
                SendSensorMessage();
            }
            // IMPORTANT: Réinitialiser le buffer APRÈS traitement
            memset(uartBuffer, 0, sizeof(uartBuffer));
            uartIndex = 0;
        }
        
        // Mode automatique RFID
        ALS_CS_PORT |= (1 << ALS_CS_BIT);
        RFID_SS_PORT &= ~(1 << RFID_SS_BIT);
        
        if (MFRC522_Request(PICC_REQIDL, tagType)) {
            if (MFRC522_Anticoll(uid)) {
                if (!rfidSmsEnvoye) {
                    UART_TransmitString("Carte detectee UID: ");
                    for (uint8_t i = 0; i < 4; i++) {
                        char hex[3];
                        uint8_to_hex(uid[i], hex);
                        UART_TransmitString(hex);
                        UART_TransmitString(" ");
                    }
                    UART_TransmitString("\r\n");
                    
                    SendRFIDMessage(uid);
                    rfidSmsEnvoye = 1;
                }
                MFRC522_Halt();
            }
        } else {
            rfidSmsEnvoye = 0;
        }
        
        RFID_SS_PORT |= (1 << RFID_SS_BIT);
        
        _delay_ms(100);
    }
    
    return 0;
}