#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

void enable_lcd(void);
void send_cmd(unsigned char);
void send_char(unsigned char);
void send_string(const char *);
void init_lcd(void);

#endif
