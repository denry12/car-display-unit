#ifndef LCD_ILI9225_LIB_H
#define LCD_ILI9225_LIB_H

#define LCDCOLOUR_BLACK		0x0000
#define LCDCOLOUR_WHITE		0xFFFF
#define LCDCOLOUR_RED		0xF800
#define LCDCOLOUR_GREEN		0x07E0
#define LCDCOLOUR_BLUE		0x001F
#define LCDCOLOUR_YELLOW	0xFFE0
#define LCDCOLOUR_PURPLE	0xF81F
#define LCDCOLOUR_DIM_RED	0x7800

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	void (*sendSPIpacket)(); // Pretty sure there should be a how to contents inside marked too!
	void (*enableCS)(); //sets CS voltage low
	void (*disableCS)(); //sets CS voltage high
	void (*setRS)(); //
	void (*clrRS)(); //
	void (*enableReset)(); //
	void (*disableReset)(); //
	void (*delay50ms)();

} ili9225_instance;

#endif
