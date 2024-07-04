
/* 
 * File:   main.c
 * Author: Hennie Kotze firmware@autonomech.co.za
 *
 * Created on 31 March 2019, 1:38 AM
 */

/******************************************************************************/
/*                             PROJECT PINOUT                                 */
/******************************************************************************/
/*                             ATMega328P
 *                                ______
 *            RESET/PCINT14/PC6 =|01* 28|= PC5/PCINT13/SCL/ADC5
 *               RX/PCINT16/PD0 =|02  27|= PC4/PCINT12/SDA/ADC4
 *               TX/PCINT17/PD1 =|03  26|= PC3/PCINT11/ADC3
 *             INT0/PCINT18/PD2 =|04  25|= PC2/PCINT10/ADC2
 *                  PCINT19/PD3 =|05  24|= PC1/PCINT9/ADC1
 *                  PCINT20/PD4 =|06  23|= PC0/PCINT8/ADC0
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *             XTAL1/PCINT6/PB6 =|09  20|= AVcc
 *             XTAL2/PCINT7/PB7 =|10  19|= PB5/PCINT5/SCK
 *             OC0B/PCINT21/PD5 =|11  18|= PB4/PCINT4/MISO
 *        OC0A/AIN0/PCINT22/PD6 =|12  17|= PB3/PCINT3/MOSI/OC2A/OC2
 *             AIN1/PCINT23/PD7 =|13  16|= PB2/PCINT2/SS/OC1B
 *                   PCINT0/PB0 =|14  15|= PB1/PCINT1/OC1A
 *                                ------
 * 
 *                                ______
 *                              =|01* 28|= ***(Free)
 *                     UART___/ =|02  27|= PIN_RST (PC4)
 *                            \ =|03  26|= PIN_CS (PC3)
 *                 (PD2) PIN_02 =|04  25|= PIN_RS (PC2)
 *                 (PD3) PIN_03 =|05  24|= PIN_WR (PC1)
 *                 (PD4) PIN_04 =|06  23|= PIN_RD (PC0)
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *                      XTAL__/ =|09  20|= AVcc
 *                            \ =|10  19|= \
 *                 (PD5) PIN_05 =|11  18|= |--SPI
 *                 (PD6) PIN_06 =|12  17|= /
 *                 (PD7) PIN_07 =|13  16|= ***(Free) DIAGNOSTIC OUTPUT / POWER PIN
 *                 (PB0) PIN_00 =|14  15|= PIN_01 (PB1) 
 *                                ------
 * 
 * 
 */



/******************************************************************************/
/*                                   DEFS                                     */
/******************************************************************************/

#define F_CPU           16000000
//#define BAUD            9600
#define BAUD            38400
//#define BAUD            19200
#define MEMSIZE         2048
#define TRUE            1
#define FALSE           0

//---MACROS
#define CMD_EQ(A)           strcmp(A, rxbuf) == 0
#define CMD_EQN(A,N)        strncmp(A, rxbuf, N) == 0
#define ITOA(A)             itoa(A, txbuf, 10)
#define ITOA2(A)            itoa(A, txbuf, 2)
#define ITOA16(A)           itoa(A, txbuf, 16)

#define _readpin(a,b)       (a & b)
#define _setpin(a,b)        a |= b
#define _clearpin(a,b)      a &= ~(b)
#define _togglepin(a,b)     a ^= ba

//diagnostix
//#define PIN_DIAG    0x04
//#define PORT_DIAG   PORTB
#define PIN_POWER    0x04
#define PORT_POWER   PORTB

//
#define CHAR_COLS               0x08
#define CHAR_ROWS               0x06
#define CHAR_BIN_WITDH          0x28//40 pixels
#define CHAR_BIN_HEIGHT         0x50//80 pixels
#define CHAR_BBOX_WITDH         0x28//40 pixels
#define CHAR_BBOX_HEIGHT        0x50//80 pixels



/******************************************************************************/
/*                                 INCLUDES                                   */
/******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/setbaud.h>
//#include <util/twi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "ili9481.h"

/******************************************************************************/
/*                                VARIABLES                                   */

/******************************************************************************/

uint32_t timer_counter;
uint8_t status_byte = 0xff;


uint16_t white, black, red, green, blue;

bool flipflop = true;

const uint8_t RX_TX_BUF_LEN = 16;

typedef struct RXTXBUF
{
    volatile uint8_t buffer[RX_TX_BUF_LEN];
    volatile uint8_t idx_marker : 7;
    volatile uint8_t is_complete : 1;
} RXTXBUF;

RXTXBUF rxbuf; // = {.idx_marker = 0, .is_complete = 0};
RXTXBUF txbuf; // = {.idx_marker = 0, .is_complete = 0};
/******************************************************************************/
/*                             STRUCTS AND ENUMS                              */

/******************************************************************************/


/******************************************************************************/
/*                            FUNCTION DECLARATIONS                           */
/******************************************************************************/

int main(void);
static void init(void);
void mainloop(void);
//usart
void usart_tx(uint8_t c);
void usart_rx(void);

void usart_send_chars(char* str);
void usart_clear_buf(RXTXBUF * pBuf);
//void usart_clear_rxbuf(void);
//i2c
//ADC
//eeprom
//timer
ISR(TIMER0_OVF_vect);
//util
uint8_t reverse_bits(uint8_t);
void delay_n_us(uint16_t);
void delay_n_ms(uint16_t);
int available_sram(void);

/******************************************************************************
 *                                FUNCTIONS                                   *
 ******************************************************************************/

int main(void)
{

    rxbuf.idx_marker = 0;
    rxbuf.is_complete = 0;

    white = rgb_565(31, 63, 31);
    black = rgb_565(0, 0, 0);
    red = rgb_565(31, 0, 0);
    green = rgb_565(0, 63, 0);
    blue = rgb_565(0, 0, 31);

    init();
    _clearpin(PORT_POWER, PIN_POWER);
    _delay_ms(250);
    _setpin(PORT_POWER, PIN_POWER);
    _delay_ms(250);
    init_tftlcd();
    initFS(2, //size
           5, //font char px w
           7, //font char px h
           1, //font char pad h
           2, //font char pad v
           0, //xpos
           0, //ypos
           white);
    mainloop();
    return 0;
}

/*******************************************************************************
 *                                                                        INIT*/

static void init(void)
{

    //=======================================================================I/O
    //Direction registers port c 1=output 0=input
    /*If DDxn is written logic one, Pxn is configured as an output pin. 
     * If DDxn is written logic zero, Pxn is configured as an input pin.
     * If PORTxn is written logic one AND the pin is configured as an input pin (0), 
     * the pull-up resistor is activated.
     */
    DDRB |= PIN_TFT_00 | PIN_TFT_01 | PIN_POWER | _BV(PB5);
    PORTB |= 0xff;

    DDRC |= PIN_TFT_RST | PIN_TFT_CS | PIN_TFT_CMD | PIN_TFT_WR | PIN_TFT_RD;
    PORTC |= 0xff;

    DDRD |= PIN_TFT_02 | PIN_TFT_03 | PIN_TFT_04 | PIN_TFT_05 | PIN_TFT_06 | PIN_TFT_07;
    PORTD |= 0xff;

    //=======================================================================ADC

    //======================================================================UART
    UBRR0H = UBRRH_VALUE; //(_UBRR)>>8;
    UBRR0L = UBRRL_VALUE; //_UBRR;
    //enable rx and tx and rx interrupt
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
    //frame format 8 data 1 stop
    UCSR0C |= _BV(UCSZ00) | _BV(UCSZ01);

    //====================================================================TIMER0
    TCNT0 = 0x00;
    TCCR0B |= _BV(CS00); // | (1 << CS02);//prescaler
    TIMSK0 |= _BV(TOV0); //timer 0 overflow interrupt enable

    //=======================================================================I2C

    //================================================================INTERRUPTS
    sei();
}

/******************************************************************************
 *                                                                    MAINLOOP*/
void mainloop(void)
{
    print_str("Test\n\0");


    while (TRUE)
    {
        usart_send_chars("AT+BANK=02\n\0");
        while (!rxbuf.is_complete);

        rxbuf.is_complete = FALSE;
        print_str((const char *) &rxbuf.buffer);
        usart_clear_buf(&rxbuf);
        _delay_ms(100);
        usart_send_chars("AT+CHU\n\0");
        while (!rxbuf.is_complete);
        //{
        //rxbuf.is_complete = FALSE;
        pFS->font_size++;
        /**/if (rxbuf.buffer[4] == '2') print_str("KFM\n\0");
        /**/if (rxbuf.buffer[4] == '3') print_str("Zibonele FM\n\0");
        else if (rxbuf.buffer[4] == '4') print_str("Good Hope FM\n\0");
        else if (rxbuf.buffer[4] == '6') print_str("Radio 2000\n\0");
        else if (rxbuf.buffer[4] == '7') print_str("Bok Radio\n\0");
        else if (rxbuf.buffer[4] == '8') print_str("RSG\n\0");
        else
        {

            //pFS->font_size--;
            pFS->font_size--;
            print_str((const char *) rxbuf.buffer);
            pFS->font_size++;
        }
        pFS->font_size--;
        //printfi_str("%d \0", (uint16_t) rxbuf[4]);
        usart_clear_buf(&rxbuf);
        //}
        _delay_ms(5000);
    }
    return;
}


/*----------------------------------------------------------------------------*/

/*                                                                       USART*/

void usart_tx(uint8_t c)
{
    //count++;
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void usart_rx(void)
{
    while (!(UCSR0A & (1 << RXC0)));
    volatile uint8_t c = UDR0;
    if (rxbuf.idx_marker == (RX_TX_BUF_LEN - 1) /*|| c == 0x0D*/ || c == 0x0A || c == 0x00)
    {
        rxbuf.buffer[rxbuf.idx_marker++] = c;
        rxbuf.is_complete = TRUE;
        return;
    }
    else
    {
        rxbuf.buffer[rxbuf.idx_marker++] = c;
    }
    return;
}

void usart_send_chars(char str[])
{
    pFS->fg_colour = green;
    print_str(str);
    pFS->fg_colour = white;
    for (uint8_t idx = 0; str[idx] != '\0';)
    {
        usart_tx(str[idx++]);
    }
    usart_clear_buf(&txbuf);
}

void usart_clear_buf(RXTXBUF * pBuf)
{
    uint8_t idx;
    for (idx = 0; pBuf->buffer[idx] != '\0';)
    {
        pBuf->buffer[idx++] = 0x00;
    }
    pBuf->idx_marker = 0;
    pBuf->is_complete = 0;
    return;
}


ISR(USART_RX_vect)
{
    usart_rx();
}



/*****************************************************************************
 *                                                                     I2C LCD*/

/******************************************************************************
 *                                                                        ADC*/

/******************************************************************************
 *                                                                      TIMER0*/
ISR(TIMER0_OVF_vect)
{
    timer_counter++;
    if (timer_counter > 0x6FFF)
    {
        timer_counter = 0;
        if (status_byte > 1)
        {
            flipflop = !flipflop;
            if (flipflop)
            {
                fill_rect(310, 470, 5, 5, black);
            }
            else
            {

                fill_rect(310, 470, 5, 5, 0x1C3F);
            }
        }
    }
}

/*****************************************************************************
 *                                                                      EEPROM*/

/*****************************************************************************
 *                                                                        UTIL*/

uint8_t reverse_bits(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void delay_n_us(uint16_t n)
{
    while (n--)
    {
        _delay_us(1);
    }
    return;
}

void delay_n_ms(uint16_t n)
{
    while (n--)
    {
        _delay_ms(1);
    }
    return;
}

int available_sram(void)
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}