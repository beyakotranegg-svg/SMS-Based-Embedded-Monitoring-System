#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* ================= CONFIG ================= */
#define SS_PIN   PB2
#define RST_PIN  PD7

#define MI_OK       0
#define MI_ERR      1
#define MI_NOTAGERR 2

/* MFRC522 Registers */
#define CommandReg     0x01
#define CommIEnReg     0x02
#define CommIrqReg     0x04
#define ErrorReg       0x06
#define Status2Reg     0x08
#define FIFODataReg    0x09
#define FIFOLevelReg   0x0A
#define ControlReg     0x0C
#define BitFramingReg  0x0D
#define ModeReg        0x11
#define TxControlReg   0x14
#define TxASKReg       0x15
#define TModeReg       0x2A
#define TPrescalerReg  0x2B
#define TReloadRegH   0x2C
#define TReloadRegL   0x2D

/* MFRC522 Commands */
#define PCD_IDLE       0x00
#define PCD_TRANSCEIVE 0x0C

/* PICC Commands */
#define PICC_REQIDL    0x26
#define PICC_ANTICOLL  0x93

/* ================= UART ================= */
void UART_init(uint16_t baud)
{
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void UART_sendString(const char *s)
{
    while (*s) UART_sendChar(*s++);
}

void UART_sendHex(uint8_t val)
{
    char hex[] = "0123456789ABCDEF";
    UART_sendChar(hex[val >> 4]);
    UART_sendChar(hex[val & 0x0F]);
}

/* ================= SPI ================= */
void SPI_init(void)
{
    DDRB |= (1 << PB3) | (1 << PB5) | (1 << SS_PIN);
    DDRB &= ~(1 << PB4);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    PORTB |= (1 << SS_PIN);
}

uint8_t SPI_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

/* ================= MFRC522 LOW LEVEL ================= */
void MFRC522_Write(uint8_t reg, uint8_t val)
{
    PORTB &= ~(1 << SS_PIN);
    SPI_transfer((reg << 1) & 0x7E);
    SPI_transfer(val);
    PORTB |= (1 << SS_PIN);
}

uint8_t MFRC522_Read(uint8_t reg)
{
    uint8_t val;
    PORTB &= ~(1 << SS_PIN);
    SPI_transfer(((reg << 1) & 0x7E) | 0x80);
    val = SPI_transfer(0x00);
    PORTB |= (1 << SS_PIN);
    return val;
}

void MFRC522_SetBit(uint8_t reg, uint8_t mask)
{
    MFRC522_Write(reg, MFRC522_Read(reg) | mask);
}

void MFRC522_ClearBit(uint8_t reg, uint8_t mask)
{
    MFRC522_Write(reg, MFRC522_Read(reg) & (~mask));
}

/* ================= MFRC522 CORE ================= */
void MFRC522_AntennaOn(void)
{
    uint8_t temp = MFRC522_Read(TxControlReg);
    if (!(temp & 0x03))
        MFRC522_SetBit(TxControlReg, 0x03);
}

void MFRC522_Init(void)
{
    DDRD |= (1 << RST_PIN);
    PORTD |= (1 << RST_PIN);

    MFRC522_Write(CommandReg, PCD_IDLE);
    MFRC522_Write(TModeReg, 0x8D);
    MFRC522_Write(TPrescalerReg, 0x3E);
    MFRC522_Write(TReloadRegL, 30);
    MFRC522_Write(TReloadRegH, 0);
    MFRC522_Write(TxASKReg, 0x40);
    MFRC522_Write(ModeReg, 0x3D);

    MFRC522_AntennaOn();
}

/* ================= COMMUNICATION ================= */
uint8_t MFRC522_ToCard(uint8_t *send, uint8_t sendLen,
                       uint8_t *back, uint8_t *backLen)
{
    uint8_t irqWait = 0x30;
    uint8_t n, i;

    MFRC522_Write(CommIrqReg, 0x7F);
    MFRC522_Write(FIFOLevelReg, 0x80);

    for (i = 0; i < sendLen; i++)
        MFRC522_Write(FIFODataReg, send[i]);

    MFRC522_Write(CommandReg, PCD_TRANSCEIVE);
    MFRC522_SetBit(BitFramingReg, 0x80);

    i = 200;
    do {
        n = MFRC522_Read(CommIrqReg);
        i--;
    } while (i && !(n & irqWait));

    MFRC522_ClearBit(BitFramingReg, 0x80);

    if (i == 0) return MI_ERR;
    if (MFRC522_Read(ErrorReg) & 0x1B) return MI_ERR;

    n = MFRC522_Read(FIFOLevelReg);
    *backLen = n;
    for (i = 0; i < n; i++)
        back[i] = MFRC522_Read(FIFODataReg);

    return MI_OK;
}

/* ================= RFID FUNCTIONS ================= */
uint8_t MFRC522_Request(uint8_t *type)
{
    uint8_t backLen;
    MFRC522_Write(BitFramingReg, 0x07);
    type[0] = PICC_REQIDL;
    return MFRC522_ToCard(type, 1, type, &backLen);
}

uint8_t MFRC522_Anticoll(uint8_t *uid)
{
    uint8_t backLen;
    uint8_t cmd[2] = {PICC_ANTICOLL, 0x20};
    MFRC522_Write(BitFramingReg, 0x00);
    return MFRC522_ToCard(cmd, 2, uid, &backLen);
}

/* ================= MAIN ================= */
int main(void)
{
    uint8_t status;
    uint8_t uid[5];
    uint8_t type[2];

    UART_init(9600);
    SPI_init();
    MFRC522_Init();

    UART_sendString("RFID pret\r\n");

    while (1)
    {
        status = MFRC522_Request(type);
        if (status == MI_OK)
        {
            status = MFRC522_Anticoll(uid);
            if (status == MI_OK)
            {
                UART_sendString("Carte detectee : ");
                for (uint8_t i = 0; i < 4; i++)
                {
                    UART_sendHex(uid[i]);
                }
                UART_sendString("\r\n");

                while (MFRC522_Request(type) == MI_OK);
                _delay_ms(200);
            }
        }
    }
}
