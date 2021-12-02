#include "lcd.h"

uint8_t arrup_cgram[8] = {0x00, 0x00, 0x00, 0x04, 0x0E, 0x1F, 0x00, 0x00};
uint8_t arrdow_cgram[8] = {0x00, 0x00, 0x1F, 0x0E, 0x04, 0x00, 0x00, 0x00};
uint8_t arrmenu_cgram[8] = {0x00, 0x00, 0x10, 0x18, 0x1C, 0x18, 0x10, 0x00};

void lcd_send(bool rs, uint8_t value) {
    RS = rs;
    DATA = ((DATA & 0xF0) | (value >> 4));
    EN = 1;
    __delay_ms(2);
    EN = 0;
    DATA = ((DATA & 0xF0) | (value & 0x0F));
    EN = 1;
    __delay_ms(2);
    EN = 0;
}

void lcd_init(bool display, bool cursor, bool blink) {
    __delay_ms(12);
    PORTD = 0x00;
    TRISD &= 0xC0;
    
    lcd_send(0, 0x02);
    lcd_send(0, 0x28);
    lcd_display(display, cursor, blink);
    lcd_send(0, 0x14);
    lcd_send(0, 0x01);
    lcd_create_char(UP_ARROW_CG, arrup_cgram);
    lcd_create_char(DOWN_ARROW_CG, arrdow_cgram);
    lcd_create_char(SELECT_ARROW_CG, arrmenu_cgram);
}

void lcd_display(bool display, bool cursor, bool blink) {
    char config = 0x08;
    if (display) {
        config |= 0x04;
    }
    if (cursor) {
        config |= 0x02;
    } 
    if (blink) {
        config |= 0x01;
    }
    lcd_send(0, config);
}

void lcd_write_string(char *string) {
    while(*string != '\0') {
        lcd_send(true, *string);
        string++;
    }
}

void lcd_write_char(char character) {
    lcd_send(true, character);
}

void lcd_write_uint8(uint8_t number) {
    lcd_write_char(48 + ((uint8_t)((number / 100) % 10)));
    lcd_write_char(48 + ((uint8_t)((number / 10) % 10)));
    lcd_write_char(48 + ((uint8_t)(number % 10)));
}

void lcd_move_cursor(uint8_t address) {
    lcd_send(0, (0x80 | address));
}

void lcd_clear_display(void) {
    lcd_send(false, 0x01);
}

void lcd_create_char(uint8_t address, uint8_t *bitmap) {
    lcd_send(false, 0x40 | address);
    for (uint8_t count = 0; count != 8; count++) {
        lcd_write_char(*bitmap);
        bitmap++;
    }
}