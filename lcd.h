#ifndef __LCD_GUARD__
#define __LCD_GUARD__

#define DATA PORTD
#define RS PORTDbits.RD4
#define EN PORTDbits.RD5

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define UP_ARROW_CG 0x40
#define UP_ARROW 0x00
#define DOWN_ARROW_CG 0x48
#define DOWN_ARROW 0x01
#define SELECT_ARROW_CG 0x50
#define SELECT_ARROW 0x02

extern unsigned char arrup_cgram[8];
extern unsigned char arrdow_cgram[8];
extern unsigned char arrmenu_cgram[8];

void lcd_send(bool rs, uint8_t value);
void lcd_init(bool display, bool cursor, bool blink);
void lcd_display(bool display, bool cursor, bool blink);
void lcd_write_string(char *string);
void lcd_write_char(char character);
void lcd_write_uint8(uint8_t number);
void lcd_move_cursor(uint8_t address);
void lcd_clear_display(void);
void lcd_create_char(uint8_t address, uint8_t *bitmap);

#endif