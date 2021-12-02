/*
 * File:   main.c
 * Author: anton
 *
 * Created on December 1, 2021, 5:31 PM
 */
#pragma config DEBUG = 1
#pragma config LVP = 0
#pragma config FCMEN = 0
#pragma config IESO = 0
#pragma config BOREN = 00
#pragma config CPD = 0
#pragma config CP = 0
#pragma config MCLRE = 0
#pragma config PWRTE = 1
#pragma config WDTE = 0
#pragma config FOSC = 101

#include "config.h"
#include "lcd.h"

#define LCD_BLANK "              "

#define ADC_TEMP_CHN 0
#define ADC_HUMI_CHN 1

uint16_t adc_read_value;
uint16_t temperature;
char temp_str[5] = {'0', '0', '.', '0', '0'};
uint16_t humidity;
char humid_str[5] = {'0', '0', '.', '0', '0'};

#define EEPROM_ST 0x00
#define ST_ON 'Y'
#define ST_OFF 'N'

#define EEPROM_SRL 0x01
#define SRL_HUMID 'H'
#define SRL_TEMP 'T'

unsigned char serial;
unsigned char state;

#define SEL_BTN 3
#define UP_BTN 2
#define DW_BTN 1
#define is_pushed(x, val) __delay_ms(32); if (x != 0) { return 0; } while (x == 0); return val;
#define is_odd(x) ((x & 1) == 1)
#define MENU_SIZE 2
#define MENU_OPTION 10

uint8_t menu_position = 0;
uint8_t menu_buffer = 0;
uint8_t pushed_button;

void set_adc_channel(uint8_t  channel);
void read_temp(void);
void read_humid(void);
void uart_write(char *string);
void show_menu_arrows(void);
void select_menu(void);
void update_menu_char(char value);
void write_menu_line(char *string, char value, bool top, bool selected);
uint8_t read_btn(void);

void main(void) {
    OSCCON = 0x71; // Run at 8M Hz
    
    // Setup PORTB3:1 as inputs for buttons. Integrated pull-ups are used
    ANSELH = 0x00;
    PORTB = 0x00;
    TRISB = 0b00001110;
    OPTION_REGbits.nRBPU = 0;
    WPUB = TRISB;
    
    // Setup analog ports for reading
    ANSEL = 0xFF;
    TRISA = 0xFF;
    ADCON0 = 0x81;
    ADCON1 = 0x00;
    
    // Setup USART
    TXSTAbits.TXEN = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    RCSTAbits.SPEN = 1;
    SPBRG = 51;
    PORTC = 0x00;
    TRISC = 0x80; // Input for TX
    
    // Interrupt every 200ms to update ADC readings
    T1CON = 0x30;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.TMR1IE = 1;
    
    // Read settings
    state = eeprom_read(EEPROM_ST);
    if ((state != ST_ON) && (state != ST_OFF)) {
        state = ST_ON;
        eeprom_write(state, state);
    }
    if (state == ST_ON) {
        PORTCbits.RC5 = 1;
    }
    else {
        PORTCbits.RC5 = 0;
    }
    
    serial = eeprom_read(EEPROM_SRL);
    if ((serial != SRL_HUMID) && (serial != SRL_TEMP)) {
        serial = SRL_TEMP;
        eeprom_write(EEPROM_SRL, serial);
    }
    
    lcd_init(true, false, false); // Display on. Cursor and blinking off.
    show_menu_arrows(); // Show up and down menu arrows
    write_menu_line("On: ", state, true, true);
    write_menu_line("Serial: ", serial, false, false);
    
    T1CONbits.TMR1ON = 1;
    while (1) {
        MENU_START:
        pushed_button = read_btn();
        if (pushed_button == SEL_BTN) {
            if (is_odd(menu_position)) {
                lcd_move_cursor(0x40 + MENU_OPTION);
            }
            else {
                lcd_move_cursor(0x00 + MENU_OPTION);
            }
            lcd_display(true, false, true);
            switch (menu_position) {
                case 0: // Status 
                    menu_buffer = state;
                    while (1) {
                        pushed_button = read_btn();
                        switch (pushed_button) {
                            case SEL_BTN:
                                if (menu_buffer != state) {
                                    state = menu_buffer;
                                    eeprom_write(EEPROM_ST, state);
                                    update_menu_char(state);
                                    lcd_display(true, false, false);
                                    goto MENU_START;
                                }
                                break;
                            case UP_BTN:
                            case DW_BTN:
                                if (menu_buffer == ST_ON) {
                                    menu_buffer = ST_OFF;
                                }
                                else {
                                    menu_buffer = ST_ON;
                                }
                                update_menu_char(state);
                                break;
                        }
                        
                    }
                    break;
            }
        }
        else if (pushed_button == UP_BTN) {
            if (menu_position > 0) {
                if (is_odd(menu_position)) {
                    menu_position--;
                    select_menu();
                }
                else {
                    //TODO -- Need to update menu
                }
            }
        }
        else if (pushed_button == DW_BTN) {
            if (menu_position < (MENU_SIZE - 1)) {
                if (is_odd(menu_position)) {
                    // TODO -- Need to update menu
                }
                else {
                    menu_position++;
                    select_menu();
                }
            }
        }
    }
}

void __interrupt() handle_interrupt() {
    if (PIR1bits.TMR1IF == 1) {
        T1CONbits.TMR1ON = 0;
        read_temp();
        read_humid();
        if (serial == SRL_TEMP) {
            uart_write(temp_str);
        }
        PIR1bits.TMR1IF = 0;
        TMR1H = 0x80;
        T1CONbits.TMR1ON = 1;
    }
}

void set_adc_channel(uint8_t channel) {
    ADCON0 &= 0xC3;
    ADCON0 |= ((channel << 2) & 0x3C);
}

void read_adc(void) {
    adc_read_value = 0;
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE == 1);
    NOP();
    adc_read_value |= ADRESH;
    adc_read_value = adc_read_value << 2;
    adc_read_value |= (ADRESL >> 6);
}

void read_temp(void) {
    set_adc_channel(ADC_TEMP_CHN);
    read_adc();
    temperature = (uint16_t) (adc_read_value * 48.87585533);
    temp_str[0] = 48 + ((uint8_t)(temperature / 1000));
    temp_str[1] = 48 + ((uint8_t)((temperature / 100) % 10));
    temp_str[3] = 48 + ((uint8_t)((temperature / 10) % 10));
    temp_str[4] = 48 + ((uint8_t)(temperature % 10));
}

void read_humid(void) {
    set_adc_channel(ADC_HUMI_CHN);
    read_adc();
    temperature = (uint16_t) (adc_read_value * 48.87585533);
}

void uart_write(char *string) {
    for(uint8_t count = 0; count != 5; count++) {
        while (PIR1bits.TXIF == 0);
        TXREG = *string;
        string++;
    }
    while (PIR1bits.TXIF == 0);
    TXREG = '\r';
    while (PIR1bits.TXIF == 0);
    TXREG = '\n';
}

void select_menu(void) {
    if ((menu_position & 0x01) == 0) {
        lcd_move_cursor(0x40);
        lcd_write_char(' ');
        lcd_move_cursor(0x00);
        lcd_write_char(SELECT_ARROW);
    }
    else {
        lcd_move_cursor(0x00);
        lcd_write_char(' ');
        lcd_move_cursor(0x40);
        lcd_write_char(SELECT_ARROW);
    }
}

void update_menu_char(char value) {
    if (is_odd(menu_position)) {
        lcd_move_cursor(0x40 + MENU_OPTION);
    }
    else {
        lcd_move_cursor(0x00 + MENU_OPTION);
    }
    lcd_write_char(value);
}

void write_menu_line(char *string, char value, bool top, bool selected) {
    uint8_t cursor = 0x00;
    uint8_t select = ' ';
    if (top == false) {
        cursor = 0x40;
    }
    if (selected == true) {
        select = SELECT_ARROW;
    }
    lcd_move_cursor(cursor);
    lcd_write_string(LCD_BLANK);
    lcd_move_cursor(cursor);
    lcd_write_char(select);
    lcd_write_string(string);
    lcd_move_cursor(cursor + MENU_OPTION);
    lcd_write_char(value);
}

uint8_t read_btn(void) {
    if (PORTBbits.RB3 == 0) {
        is_pushed(PORTBbits.RB3, 3);
    }
    else if (PORTBbits.RB2 == 0) {
        is_pushed(PORTBbits.RB2, 2);
    }
    else if (PORTBbits.RB1 == 0) {
        is_pushed(PORTBbits.RB1, 1);
    }
    return 0;
}

void show_menu_arrows(void) {
    lcd_move_cursor(0x0F);
    if (menu_position > 1) {
        lcd_write_char(UP_ARROW);
    }
    else {
        lcd_write_char(' ');
    }
    if (menu_position < (MENU_SIZE -1)) {
        lcd_move_cursor(0x4F);
        lcd_write_char(DOWN_ARROW);
    }
    else {
        lcd_write_char(' ');
    }
}