/*
 * File:   main.c
 * Author: anton
 *
 * Created on December 1, 2021, 5:31 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include "config.h"
#include "lcd.h"

#define LCD_BLANK "              "

#define ADC_TEMP_CHN 0
#define ADC_HUMI_CHN 1
float HUMIDITY_WET = 1.281;
float HUMIDITY_DIFF = 0.930;
float ADC_RES = 0.00488758;
uint16_t adc_read_value;
uint16_t temperature;
char temp_str[6] = {'0', '0', '.', '0', '0', '\0'};
uint16_t humidity;
float humidity_voltage;
float humidity_level;
char humid_str[6] = {'0', '0', '0', '.', '0', '\0'};

#define EEPROM_ST 0x00
#define ST_ON 'Y'
#define ST_OFF 'N'
#define ST_LEGEND "On: "

#define EEPROM_SRL 0x01
#define SRL_HUMID 'H'
#define SRL_TEMP 'T'
#define SRL_LEGEND "Serial: "

#define EEPROM_TRIG 0x02
#define TRIG_LEGEND "Trigger: "

#define DATA_LEGEND "Show data"

unsigned char serial;
unsigned char state;
uint8_t trigger;

#define SEL_BTN 3
#define UP_BTN 2
#define DW_BTN 1
#define is_pushed(x, val) __delay_ms(32); if (x != 0) { return 0; } while (x == 0); return val;
#define is_odd(x) ((x & 1) == 1)
#define MENU_SIZE 4
#define MENU_OPTION 10

uint8_t menu_position = 1;
uint8_t displayed_menus = 12; // First digit represents top menu. Second one bottom menu.
uint8_t menu_buffer = 0;
uint8_t pushed_button;
uint8_t cursor;

#define WATER_VALVE PORTEbits.RE2

void menu_loop_bichar(unsigned char *menu, uint8_t eeprom_addr, char opt1, char opt2);
void update_state(void);
void set_adc_channel(uint8_t  channel);
void read_temp(void);
void read_humid(void);
void uart_write(char *string);
void show_menu_arrows(void);
void select_menu(void);
void update_menu_char(char value);
void write_menu_line(char *string, char value, bool top, bool selected, bool is_int);
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
    ANSEL = 0x0F;
    TRISA = 0xFF;
    ADCON0 = 0x81;
    ADCON1 = 0x00;
    
    // Port that will control water valve
    PORTE = 0x00;
    TRISE = 0x00;
    
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
    update_state();
    
    serial = eeprom_read(EEPROM_SRL);
    if ((serial != SRL_HUMID) && (serial != SRL_TEMP)) {
        serial = SRL_TEMP;
        eeprom_write(EEPROM_SRL, serial);
    }
    
    trigger = eeprom_read(EEPROM_TRIG);
    if (trigger > 100) {
        trigger = 10;
    }
    
    lcd_init(true, false, false); // Display on. Cursor and blinking off
    write_menu_line(ST_LEGEND, state, true, true, false);
    write_menu_line(SRL_LEGEND, serial, false, false, false);
    show_menu_arrows(); // Show up and down menu arrows
    
    T1CONbits.TMR1ON = 1;
    while (1) {
        MENU_START:
        pushed_button = read_btn();
        if (pushed_button == SEL_BTN) {
            switch (menu_position) {
                case 1: // Status 
                    menu_loop_bichar(&state, EEPROM_ST, ST_ON, ST_OFF);
                    update_state();
                    break;
                case 2: // Serial
                    menu_loop_bichar(&serial, EEPROM_SRL, SRL_TEMP, SRL_HUMID);
                    break;
                case 3: // Trigger
                    pushed_button = 0;
                    cursor = 0x00;
                    if (displayed_menus == 23) {
                        cursor = 0x40;
                    }
                    lcd_move_cursor(cursor);
                    lcd_display(true, false, true);
                    menu_buffer = trigger;
                    while (pushed_button != SEL_BTN) {
                        pushed_button = read_btn();
                        switch (pushed_button) {
                            case UP_BTN:
                                menu_buffer++;
                                lcd_move_cursor(cursor + MENU_OPTION);
                                lcd_write_uint8(menu_buffer);
                                lcd_move_cursor(cursor);
                                break;
                            case DW_BTN:
                                menu_buffer--;
                                lcd_move_cursor(cursor + MENU_OPTION);
                                lcd_write_uint8(menu_buffer);
                                lcd_move_cursor(cursor);
                                break;
                        }
                        
                    }
                    if (menu_buffer != trigger) {
                        eeprom_write(EEPROM_TRIG, menu_buffer);
                        trigger = menu_buffer;
                    }
                    lcd_display(true, false, false);
                    break;
                case 4: // Show data
                    lcd_clear_display();
                    lcd_display(true, false, false);
                    lcd_write_string("Temp:         C");
                    lcd_move_cursor(0x40);
                    lcd_write_string("Humedad:      %");
                    pushed_button = read_btn();
                    while (pushed_button != SEL_BTN) {
                        pushed_button = read_btn();
                        lcd_move_cursor(0x09);
                        lcd_write_string(temp_str);
                        lcd_move_cursor(0x49);
                        lcd_write_string(humid_str);
                    }
                    lcd_clear_display();
                    displayed_menus = 34;
                    menu_position = 4;
                    write_menu_line(TRIG_LEGEND, trigger, true, false, true);
                    write_menu_line(DATA_LEGEND, ' ', false, true, false);
                    show_menu_arrows(); // Show up and down menu arrows
                    __delay_ms(200);
                    break;
            }
        }
        else if (pushed_button == UP_BTN) {
            if (menu_position > 1) {
                switch (displayed_menus) {
                    case 23:
                        if (menu_position == 2) {
                            write_menu_line(ST_LEGEND, state, true, true, false);
                            write_menu_line(SRL_LEGEND, serial, false, false, false);
                            displayed_menus = 12;
                        }
                        break;
                    case 34:
                        if (menu_position == 3) {
                            write_menu_line(SRL_LEGEND, serial, true, true, false);
                            write_menu_line(TRIG_LEGEND, trigger, false, false, true);
                            displayed_menus = 23;
                        }
                        break;
                }
                menu_position--;
                select_menu();
                show_menu_arrows();
            }
        }
        else if (pushed_button == DW_BTN) {
            if (menu_position < MENU_SIZE) {
                switch (displayed_menus) {
                    case 12:
                        if (menu_position == 2) {
                            write_menu_line(SRL_LEGEND, serial, true, false, false);
                            write_menu_line(TRIG_LEGEND, trigger, false, true, true);
                            displayed_menus = 23;
                        }
                        break;
                    case 23:
                        if (menu_position == 3) {
                            write_menu_line(TRIG_LEGEND, trigger, true, false, true);
                            write_menu_line(DATA_LEGEND, ' ', false, true, false);
                            displayed_menus = 34;
                        }
                        break;
                }
                menu_position++;
                select_menu();
                show_menu_arrows();
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
        else {
            uart_write(humid_str);
        }
        if (state == ST_ON) {
            if ((humidity) >= (trigger * 10)) {
                WATER_VALVE = 0;
            }
            else {
                WATER_VALVE = 1;
            }
        }
        PIR1bits.TMR1IF = 0;
        TMR1H = 0x80;
        T1CONbits.TMR1ON = 1;
    }
}

void update_state(void) {
    if (state == ST_ON) {
        PORTCbits.RC5 = 1;
    }
    else {
        PORTCbits.RC5 = 0;
        WATER_VALVE = 0;
    }
}

void set_adc_channel(uint8_t channel) {
    ADCON0 &= 0xC3;
    ADCON0 |= ((channel << 2) & 0x3C);
    __delay_us(20);
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
    humidity_voltage = (adc_read_value * ADC_RES);
    humidity_level = humidity_voltage - HUMIDITY_WET;
    if (humidity_level <= 0) {
        humidity = 1000;
    }
    else {
        humidity = (uint16_t) ((HUMIDITY_DIFF - humidity_level) * 1000);
    }
    humid_str[0] = 48 + ((uint8_t)(humidity / 1000));
    humid_str[1] = 48 + ((uint8_t)((humidity / 100) % 10));
    humid_str[2] = 48 + ((uint8_t)((humidity / 10) % 10));
    humid_str[4] = 48 + ((uint8_t)(humidity % 10));
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
    switch (displayed_menus) {
        case 12:
            if (is_odd(menu_position)) {
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
            break;
        case 23:
            if (is_odd(menu_position)) {
                lcd_move_cursor(0x00);
                lcd_write_char(' ');
                lcd_move_cursor(0x40);
                lcd_write_char(SELECT_ARROW);
            }
            else {
                lcd_move_cursor(0x40);
                lcd_write_char(' ');
                lcd_move_cursor(0x00);
                lcd_write_char(SELECT_ARROW);
            }
            break;
        case 34:
            if (is_odd(menu_position)) {
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
            break;
    }
}

void menu_loop_bichar(unsigned char *menu, uint8_t eeprom_addr, char opt1, char opt2) {
    menu_buffer = *menu;
    cursor = 0x40;
    switch (displayed_menus) {
        case 12:
            if (is_odd(menu_position)) {
                cursor = 0x00;
            }
            break;
        case 23:
            if (!is_odd(menu_position)) {
                cursor = 0x00;
            }
            break;
        case 34:
            if (is_odd(menu_position)) {
                cursor = 0x00;
            }
            break;
    }
    lcd_move_cursor(cursor);
    lcd_display(true, false, true);
    while (1) {
        pushed_button = read_btn();
        switch (pushed_button) {
            case SEL_BTN:
                if (menu_buffer != *menu) {
                    *menu = menu_buffer;
                    eeprom_write(eeprom_addr, *menu);
                    update_menu_char(*menu);
                    
                }
                lcd_display(true, false, false);
                return;
            case UP_BTN:
            case DW_BTN:
                if (menu_buffer == opt1) {
                    menu_buffer = opt2;
                }
                else {
                    menu_buffer = opt1;
                }
                update_menu_char(menu_buffer);
                break;
        }

    }
}

void update_menu_char(char value) {
    lcd_move_cursor(cursor + MENU_OPTION);
    lcd_write_char(value);
    lcd_move_cursor(cursor);
}

void write_menu_line(char *string, char value, bool top, bool selected, bool is_int) {
    cursor = 0x00;
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
    if (is_int == true) {
        lcd_write_uint8(value);
    }
    else {
        lcd_write_char(value);
    }
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
    switch (displayed_menus) {
        case 12:
            lcd_move_cursor(0x0F);
            lcd_write_char(' ');
            lcd_move_cursor(0x4F);
            lcd_write_char(DOWN_ARROW);
            break;
        case 23:
            lcd_move_cursor(0x0F);
            lcd_write_char(UP_ARROW);
            lcd_move_cursor(0x4F);
            lcd_write_char(DOWN_ARROW);
            break;
        case 34:
            lcd_move_cursor(0x0F);
            lcd_write_char(UP_ARROW);
            lcd_move_cursor(0x4F);
            lcd_write_char(' ');
    }
}