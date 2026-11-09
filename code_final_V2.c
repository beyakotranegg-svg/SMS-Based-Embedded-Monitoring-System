// =============================================================
// uno_gsm_rfid_hw_uart.c
// ATmega328P  16 MHz  (Arduino Pro Mini)
//
// WIRING CHANGE vs previous versions:
//   GSM TX  ?  Arduino D0  (PD0 = hardware UART RX)
//   GSM RX  ?  Arduino D1  (PD1 = hardware UART TX)
//   (disconnect D0/D1 from USB-serial adapter while flashing)
//
// No software UART anymore. The hardware UART handles GSM
// at 9600 baud with zero timing jitter and a proper 64-byte
// hardware FIFO backed by our own 256-byte ring buffer.
//
// All other pins unchanged:
//   RFID SS=D10(PB2)  RST=D7(PD7)
//   ALS  CS=D9 (PB1)
//   DHT11   D2 (PD2)
//   CO2     A0
//   Buzzer  D6 (PD6)
//   LCD     I2C SDA=A4 SCL=A5
// =============================================================

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ----------------------------------------------------------------
// SPI / RFID / ALS
// ----------------------------------------------------------------
#define MOSI          PB3
#define MISO          PB4
#define SCK           PB5
#define SPI_DDR       DDRB

#define RFID_SS_DDR   DDRB
#define RFID_SS_PORT  PORTB
#define RFID_SS_BIT   PB2

#define ALS_CS_DDR    DDRB
#define ALS_CS_PORT   PORTB
#define ALS_CS_BIT    PB1

#define RFID_RST_DDR  DDRD
#define RFID_RST_PORT PORTD
#define RFID_RST_BIT  PD7

// ----------------------------------------------------------------
// DHT11  D2
// ----------------------------------------------------------------
#define DHT_DDR  DDRD
#define DHT_PORT PORTD
#define DHT_PIN  PIND
#define DHT_BIT  PD2

// ----------------------------------------------------------------
// ADC / CO2  A0
// ----------------------------------------------------------------
#define CO2_CHANNEL 0

// ----------------------------------------------------------------
// Buzzer  D6
// ----------------------------------------------------------------
#define BUZZER_DDR  DDRD
#define BUZZER_PORT PORTD
#define BUZZER_BIT  PD6

// ----------------------------------------------------------------
// Authorised card UID
// ----------------------------------------------------------------
static const uint8_t AUTH_UID[4] = {0xD5, 0x54, 0xD4, 0x1F};

// ----------------------------------------------------------------
// Phone numbers
// ----------------------------------------------------------------
static const char MY_NUMBER[]    = "+21693832266";
static const char ALERT_NUMBER[] = "+21693832266";

// ================================================================
// LCD  I2C PCF8574 @ 0x27
// ================================================================
#define LCD_ADDR  0x27
#define LCD_RS   (1U << 0)
#define LCD_EN   (1U << 2)
#define LCD_BL   (1U << 3)

static void twi_init(void) {
    DDRC  &= (uint8_t)~((1U<<PC4)|(1U<<PC5));
    PORTC |= (1U<<PC4)|(1U<<PC5);
    TWSR   = 0x00;
    TWBR   = 72;   // 100 kHz @ 16 MHz
    TWCR   = (1U<<TWEN);
}

static void twi_start(uint8_t addr7, uint8_t rw) {
    TWCR = (1U<<TWINT)|(1U<<TWSTA)|(1U<<TWEN);
    while (!(TWCR & (1U<<TWINT))) {}
    TWDR = (uint8_t)((addr7<<1)|rw);
    TWCR = (1U<<TWINT)|(1U<<TWEN);
    while (!(TWCR & (1U<<TWINT))) {}
}

static void twi_write(uint8_t d) {
    TWDR = d;
    TWCR = (1U<<TWINT)|(1U<<TWEN);
    while (!(TWCR & (1U<<TWINT))) {}
}

static void twi_stop(void) {
    TWCR = (1U<<TWINT)|(1U<<TWSTO)|(1U<<TWEN);
}

static void lcd_exp(uint8_t d) {
    twi_start(LCD_ADDR, 0); twi_write(d); twi_stop();
}

static void lcd_pulse(uint8_t d) {
    lcd_exp((uint8_t)(d | LCD_EN));  _delay_us(1);
    lcd_exp((uint8_t)(d & ~LCD_EN)); _delay_us(50);
}

static void lcd_nibble(uint8_t n, uint8_t rs) {
    uint8_t o = (uint8_t)((n<<4)&0xF0) | LCD_BL;
    if (rs) o |= LCD_RS;
    lcd_pulse(o);
}

static void lcd_byte(uint8_t v, uint8_t rs) {
    lcd_nibble(v>>4, rs);
    lcd_nibble(v&0x0F, rs);
}

static void lcd_cmd(uint8_t c) {
    lcd_byte(c, 0);
    if (c==0x01||c==0x02) _delay_ms(2);
}

static void lcd_ch(uint8_t c)   { lcd_byte(c, 1); }
static void lcd_clear(void)     { lcd_cmd(0x01); _delay_ms(2); }

static void lcd_xy(uint8_t col, uint8_t row) {
    lcd_cmd((uint8_t)(0x80 | col | (row ? 0x40u : 0u)));
}

static void lcd_str(const char *s) { while (*s) lcd_ch((uint8_t)*s++); }
static void lcd_int(int v) { char b[12]; itoa(v,b,10); lcd_str(b); }

static void lcd_init(void) {
    _delay_ms(50);
    lcd_nibble(0x03,0); _delay_ms(5);
    lcd_nibble(0x03,0); _delay_ms(5);
    lcd_nibble(0x03,0); _delay_ms(1);
    lcd_nibble(0x02,0); _delay_ms(1);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06);
    lcd_clear();
}

static const char* lux_str(uint8_t v) {
    uint16_t p = (uint16_t)v*100u/255u;
    if (p<2 ) return "Obscur";
    if (p<20) return "Faible";
    if (p<70) return "Moyen";
    return "Fort";
}

// ================================================================
// Timer0 ? millis()
// ================================================================
static volatile uint32_t g_ms = 0;

static void timer0_init(void) {
    TCCR0A = 0;
    TCCR0B = (1<<WGM02)|(1<<CS01)|(1<<CS00);
    OCR0A  = 249;
    TIMSK0 = (1<<OCIE0A);
}

ISR(TIMER0_COMPA_vect) { g_ms++; }

static uint32_t millis(void) {
    uint32_t m; uint8_t s=SREG; cli(); m=g_ms; SREG=s; return m;
}

// ================================================================
// Timer2 ? buzzer 1 kHz
// ================================================================
static volatile uint8_t g_buz = 0;

static void timer2_init(void) {
    TCCR2A = 0;
    TCCR2B = (1<<WGM22)|(1<<CS22);
    OCR2A  = 62;   // toggle @ ~2 kHz ? 1 kHz tone
    TIMSK2 = (1<<OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
    if (g_buz) BUZZER_PORT ^= (1<<BUZZER_BIT);
}

static void buzzer(uint8_t on) {
    g_buz = on;
    if (!on) BUZZER_PORT &= ~(1<<BUZZER_BIT);
}

// ================================================================
// Hardware UART ? used for GSM @ 9600 baud
// PD0 = RX  (connect to GSM TX)
// PD1 = TX  (connect to GSM RX)
//
// 256-byte ring buffer ? ISR stores every received byte.
// ================================================================
#define GSM_BUF_SIZE 256
static volatile uint8_t gsm_buf[GSM_BUF_SIZE];
static volatile uint8_t gsm_head = 0;   // written by ISR
static volatile uint8_t gsm_tail = 0;   // read  by main loop

static void uart_init(void) {
    // 9600 baud @ 16 MHz: UBRR = 103
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0A = 0;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

ISR(USART_RX_vect) {
    uint8_t byte = UDR0;
    uint8_t next = (uint8_t)((gsm_head + 1u) & (GSM_BUF_SIZE - 1u));
    if (next != gsm_tail) {
        gsm_buf[gsm_head] = byte;
        gsm_head = next;
    }
    // overflow: byte silently dropped (better than corrupting the buffer)
}

// Read one byte from the ring buffer. Returns 1 on success, 0 if empty.
static uint8_t gsm_getc(uint8_t *c) {
    if (gsm_head == gsm_tail) return 0;
    *c = gsm_buf[gsm_tail];
    gsm_tail = (uint8_t)((gsm_tail + 1u) & (GSM_BUF_SIZE - 1u));
    return 1;
}

// Transmit one byte ? blocking, but hardware UART does not need cli()
static void gsm_putc(uint8_t d) {
    while (!(UCSR0A & (1<<UDRE0))) {}
    UDR0 = d;
}

static void gsm_puts(const char *s) { while (*s) gsm_putc((uint8_t)*s++); }

// Drain and discard everything arriving for wait_ms milliseconds.
// Used after AT commands to swallow the "OK\r\n" echo.
static void gsm_flush(uint16_t wait_ms) {
    uint32_t t = millis();
    while (millis() - t < wait_ms) {
        uint8_t c; while (gsm_getc(&c)) {}
    }
}

// ================================================================
// SPI
// ================================================================
static void spi_init(void) {
    SPI_DDR    |=  (1<<MOSI)|(1<<SCK);
    SPI_DDR    &= ~(1<<MISO);
    RFID_SS_DDR|=  (1<<RFID_SS_BIT);
    ALS_CS_DDR |=  (1<<ALS_CS_BIT);
    RFID_SS_PORT|= (1<<RFID_SS_BIT);
    ALS_CS_PORT |= (1<<ALS_CS_BIT);
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
}

static uint8_t spi_xfer(uint8_t d) {
    SPDR = d;
    while (!(SPSR & (1<<SPIF))) {}
    return SPDR;
}

// ================================================================
// MFRC522
// ================================================================
#define REG_COMMAND    0x01
#define REG_TMMODE     0x2A
#define REG_TPRESCALER 0x2B
#define REG_TRELOADL   0x2C
#define REG_TRELOADH   0x2D
#define REG_BITFRAME   0x0D
#define REG_FIFOLEVEL  0x0A
#define REG_FIFODATA   0x09
#define REG_MODE       0x11
#define REG_TXCONTROL  0x14
#define REG_TXASK      0x15
#define CMD_SOFTRESET  0x0F
#define CMD_IDLE       0x00
#define CMD_TRANSCEIVE 0x0C
#define PICC_REQIDL    0x26
#define PICC_ANTICOLL  0x93

static void rfid_wr(uint8_t r, uint8_t v) {
    RFID_SS_PORT &= ~(1<<RFID_SS_BIT);
    spi_xfer((r<<1)&0x7E); spi_xfer(v);
    RFID_SS_PORT |=  (1<<RFID_SS_BIT);
}

static uint8_t rfid_rd(uint8_t r) {
    RFID_SS_PORT &= ~(1<<RFID_SS_BIT);
    spi_xfer(((r<<1)&0x7E)|0x80);
    uint8_t v = spi_xfer(0);
    RFID_SS_PORT |=  (1<<RFID_SS_BIT);
    return v;
}

static void rfid_init(void) {
    RFID_RST_DDR  |=  (1<<RFID_RST_BIT);
    RFID_RST_PORT &= ~(1<<RFID_RST_BIT); _delay_ms(10);
    RFID_RST_PORT |=  (1<<RFID_RST_BIT); _delay_ms(50);
    rfid_wr(REG_COMMAND,    CMD_SOFTRESET); _delay_ms(50);
    rfid_wr(REG_TMMODE,     0x8D);
    rfid_wr(REG_TPRESCALER, 0x3E);
    rfid_wr(REG_TRELOADL,   30);
    rfid_wr(REG_TRELOADH,   0);
    rfid_wr(REG_TXASK,      0x40);
    rfid_wr(REG_MODE,       0x3D);
    rfid_wr(REG_TXCONTROL,  0x83);
}

static uint8_t rfid_request(uint8_t *tag) {
    rfid_wr(REG_BITFRAME,  0x07);
    rfid_wr(REG_FIFOLEVEL, 0x80);
    rfid_wr(REG_FIFODATA,  PICC_REQIDL);
    rfid_wr(REG_COMMAND,   CMD_TRANSCEIVE);
    rfid_wr(REG_BITFRAME,  0x87);
    _delay_ms(10);
    uint8_t n = rfid_rd(REG_FIFOLEVEL);
    if (!n) return 0;
    for (uint8_t i=0;i<n;i++) tag[i] = rfid_rd(REG_FIFODATA);
    return 1;
}

static uint8_t rfid_anticoll(uint8_t *uid) {
    rfid_wr(REG_BITFRAME,  0x00);
    rfid_wr(REG_FIFOLEVEL, 0x80);
    rfid_wr(REG_FIFODATA,  PICC_ANTICOLL);
    rfid_wr(REG_FIFODATA,  0x20);
    rfid_wr(REG_COMMAND,   CMD_TRANSCEIVE);
    rfid_wr(REG_BITFRAME,  0x80);
    _delay_ms(10);
    uint8_t n = rfid_rd(REG_FIFOLEVEL);
    if (n<5) return 0;
    for (uint8_t i=0;i<4;i++) uid[i] = rfid_rd(REG_FIFODATA);
    return 1;
}

static void rfid_halt(void) { rfid_wr(REG_COMMAND, CMD_IDLE); }

// ================================================================
// DHT11
// ================================================================
static uint8_t dht_start(void) {
    DHT_DDR  |=  (1<<DHT_BIT);
    DHT_PORT &= ~(1<<DHT_BIT); _delay_ms(18);
    DHT_PORT |=  (1<<DHT_BIT);
    DHT_DDR  &= ~(1<<DHT_BIT); _delay_us(40);
    if ( DHT_PIN & (1<<DHT_BIT)) return 0;
    _delay_us(80);
    if (!(DHT_PIN & (1<<DHT_BIT))) return 0;
    _delay_us(80);
    return 1;
}

static uint8_t dht_byte(void) {
    uint8_t b=0;
    for (uint8_t i=0;i<8;i++) {
        while (!(DHT_PIN & (1<<DHT_BIT))) {}
        _delay_us(30);
        if (DHT_PIN & (1<<DHT_BIT)) b |= (1<<(7-i));
        while (DHT_PIN & (1<<DHT_BIT)) {}
    }
    return b;
}

static uint8_t dht_read(uint8_t *h, uint8_t *t) {
    if (!dht_start()) return 0;
    uint8_t hi=dht_byte(), hd=dht_byte(), ti=dht_byte(), td=dht_byte(), cs=dht_byte();
    DHT_DDR|=(1<<DHT_BIT); DHT_PORT|=(1<<DHT_BIT);
    if ((uint8_t)(hi+hd+ti+td)!=cs) return 0;
    *h=hi; *t=ti; return 1;
}

// ================================================================
// ADC + ALS
// ================================================================
static void adc_init(void) {
    ADMUX  = (1<<REFS0)|(CO2_CHANNEL&0x0F);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

static uint16_t adc_read(uint8_t ch) {
    ADMUX = (uint8_t)((ADMUX&0xF0)|(ch&0x0F));
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC)) {}
    return ADC;
}

static uint8_t als_read(void) {
    ALS_CS_PORT &= ~(1<<ALS_CS_BIT);
    uint8_t hi=spi_xfer(0), lo=spi_xfer(0);
    ALS_CS_PORT |=  (1<<ALS_CS_BIT);
    return (uint8_t)((hi<<3)|(lo>>4));
}
static uint16_t co2_to_ppm(uint16_t raw) {
    return (uint16_t)(400UL + ((uint32_t)raw * 4600UL) / 1023UL);
}
// ================================================================
// GSM high-level
// ================================================================
static void gsm_init(void) {
    gsm_puts("AT+CMGF=1\r\n");
    gsm_flush(500);
    gsm_puts("AT+CNMI=2,2,0,0,0\r\n");
    gsm_flush(500);
}

static void gsm_send_sms(const char *num, const char *msg) {
    gsm_puts("AT+CMGF=1\r\n");     gsm_flush(500);
    gsm_puts("AT+CMGS=\"");
    gsm_puts(num);
    gsm_puts("\"\r\n");             gsm_flush(500);
    gsm_puts(msg);
    gsm_putc(26);                   // CTRL+Z
    gsm_flush(8000);                // wait up to 8 s for +CMGS:/OK
}

static void send_sensor_sms(const char *phone) {
    uint8_t h=0, t=0;
    dht_read(&h, &t);
    uint8_t  lux = als_read();
    uint16_t co2 = co2_to_ppm(adc_read(CO2_CHANNEL));   // <-- converti ici

    char msg[160];
    snprintf(msg, sizeof(msg),
             "DONNEES:\nTemp:%u C\nHum:%u%%\nCO2:%u ppm\nLum:%s",  // <-- ppm
             t, h, (unsigned)co2, lux_str(lux));

    lcd_clear(); lcd_xy(0,0); lcd_str("Envoi du SMS...");
    lcd_xy(0,1); lcd_str("Patientez...");

    gsm_send_sms(phone, msg);

    lcd_clear(); lcd_xy(0,0); lcd_str("SMS envoye !");
    _delay_ms(2000);
}

static void send_alert_sms(const char *phone, const char *msg) {
    gsm_send_sms(phone, msg);
}

static uint8_t uid_ok(const uint8_t u[4]) {
    return u[0]==AUTH_UID[0] && u[1]==AUTH_UID[1] &&
           u[2]==AUTH_UID[2] && u[3]==AUTH_UID[3];
}

static void alarm_check(uint8_t temp, uint16_t co2) {
    if (temp>50 || co2>800) buzzer(1); else buzzer(0);
}

// ================================================================
// Main
// ================================================================
int main(void) {

    DHT_DDR    &= ~(1<<DHT_BIT);
    BUZZER_DDR |=  (1<<BUZZER_BIT);
    buzzer(0);
    RFID_RST_DDR  |= (1<<RFID_RST_BIT);
    RFID_RST_PORT |= (1<<RFID_RST_BIT);

    uart_init();
    spi_init();
    rfid_init();
    adc_init();
    twi_init();
    timer0_init();
    timer2_init();
    sei();

    lcd_init();
    lcd_clear(); lcd_xy(0,0); lcd_str("Systeme pret");
    lcd_xy(0,1);  lcd_str("Swiper carte");
    _delay_ms(2000);
    lcd_clear();

    gsm_init();

    // ---- SMS line parser state ----
    static char    line[200];
    static uint8_t lpos      = 0;
    static char    sender[24]= {0};
    static uint8_t want_body = 0;
    static uint8_t sms_ready = 0;

    // ---- RFID state ----
    static uint8_t  rfid_done   = 0;
    static uint8_t  auth_active = 0;
    static uint32_t auth_until  = 0;

    uint8_t tag[2], uid[4];

    while (1) {

        // ==== 1) GSM ring-buffer drain ====
        //
        // Hardware UART ISR fills gsm_buf[] independently of anything
        // the main loop does ? no timing constraints here at all.
        // We only set sms_ready inside this loop; the actual SMS send
        // happens below, after the loop, to keep the loop tight.
        //
        uint8_t c;
        while (gsm_getc(&c)) {

            if (c == '\r') continue;

            if (c == '\n') {
                line[lpos] = '\0';
                lpos = 0;

                if (line[0] == '\0') continue;   // blank line, keep want_body

                if (want_body) {
                    want_body = 0;

                    // trim leading spaces/tabs
                    char *body = line;
                    while (*body==' '||*body=='\t') body++;

                    // match "s" or "S" with any trailing whitespace/CR
                    if ((body[0]=='s'||body[0]=='S') &&
                        (body[1]=='\0'||body[1]==' '||body[1]=='\t'||body[1]=='\r')) {
                        sms_ready = 1;
                    }

                } else if (strncmp(line, "+CMT:", 5) == 0) {
                    // +CMT: "+21600000000","","yy/mm/dd,hh:mm:ss+zz"
                    char *p1 = strchr(line, '"');
                    if (p1) {
                        p1++;
                        char *p2 = strchr(p1, '"');
                        if (p2) {
                            uint8_t len = (uint8_t)(p2-p1);
                            if (len >= sizeof(sender)) len = sizeof(sender)-1;
                            memcpy(sender, p1, len);
                            sender[len] = '\0';
                        }
                    }
                    want_body = 1;
                }

            } else {
                if (lpos < (uint8_t)(sizeof(line)-1))
                    line[lpos++] = (char)c;
            }
        }

        // Act on received SMS command (outside the drain loop)
        if (sms_ready) {
            sms_ready = 0;
            send_sensor_sms(sender);
        }

        // ==== 2) RFID ====
        ALS_CS_PORT  |=  (1<<ALS_CS_BIT);
        RFID_SS_PORT &= ~(1<<RFID_SS_BIT);

        uint8_t card = rfid_request(tag) && rfid_anticoll(uid);

        if (card) {
            if (!rfid_done) {
                if (uid_ok(uid)) {
                    auth_active = 1;
                    auth_until  = millis() + 10000UL;
                    lcd_clear(); lcd_xy(0,0); lcd_str("Acces autorise");
                    lcd_xy(0,1); lcd_str("Bienvenue");
                    buzzer(1); _delay_ms(100); buzzer(0);
                } else {
                    lcd_clear(); lcd_xy(0,0); lcd_str("Acces refuse");
                    lcd_xy(0,1); lcd_str("Carte inconnue");
                    for (uint8_t i=0;i<3;i++) {
                        buzzer(1); _delay_ms(100);
                        buzzer(0); _delay_ms(100);
                    }
                    send_alert_sms(ALERT_NUMBER, "CARTE INCONNUE DETECTEE !");
                }
                rfid_done = 1;
                rfid_halt();
                _delay_ms(2000);
            }
        } else {
            rfid_done = 0;
        }

        RFID_SS_PORT |= (1<<RFID_SS_BIT);

        // ==== 3) Sensor display / idle ====
        if (auth_active) {
            if (millis() < auth_until) {
                static uint32_t last_s = 0;
                if (millis()-last_s >= 2000UL) {
                    uint8_t h=0, t=0;
                    dht_read(&h, &t);
                    uint8_t  lux = als_read();
                    uint16_t co2 = adc_read(CO2_CHANNEL);
                    alarm_check(t, co2);

                    lcd_clear(); lcd_xy(0,0);
                    lcd_str("T:"); lcd_int((int)t);
                    lcd_str("C H:"); lcd_int((int)h); lcd_ch('%');
                    lcd_xy(0,1);
                    lcd_str("CO2:"); lcd_int((int)co2);lcd_str("ppm");
                    lcd_str(" L:"); lcd_str(lux_str(lux));

                    last_s = millis();
                }
            } else {
                auth_active = 0; buzzer(0); lcd_clear();
            }
        } else {
            static uint32_t last_idle = 0;
            if (millis()-last_idle >= 1000UL) {
                last_idle = millis();
                lcd_clear(); lcd_xy(0,0); lcd_str("Swiper la carte");
                lcd_xy(0,1); lcd_str("pour continuer");
            }
        }

        _delay_ms(10);
    }
}