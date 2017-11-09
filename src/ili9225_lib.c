/*
 * ili9225_lib.c
 *
 *  Created on: 5. nov 2017
 *      Author: Denry
 */



#include <stdint.h>
#include <stdbool.h>
#include "ili9225_lib.h"

// if "portrait" uncommented, top left is 0,0 with pins marking top
// if "portrait" commented out,top left is 0,0 with pins marking left
//#define ORIENTATION_PORTRAIT
#define ORIENTATION_LANDSCAPE // this one is just to grab attention lol


// appnote
// http://www.displayfuture.com/Display/datasheet/controller/ILI9225.pdf

// datasheet (for model with B suffix)
// http://www.hpinfotech.ro/ILI9225B.pdf

void ili9225_sendCommand(ili9225_instance *instance, uint16_t data){
	(*instance).clrRS();
	//(*instance).enableCS();
	(*instance).sendSPIpacket((data>>8)&0xFF);
	(*instance).sendSPIpacket(data&0xFF);
	//(*instance).disableCS();
	return;
}

void ili9225_sendData(ili9225_instance *instance, uint16_t data){
	(*instance).setRS();
	//(*instance).enableCS();
	(*instance).sendSPIpacket((data>>8)&0xFF);
	(*instance).sendSPIpacket(data&0xFF);
	//(*instance).disableCS();
	return;
}

static inline void
__attribute__((always_inline)) ili9225_sendData_RSpreset(ili9225_instance *instance, uint16_t data){
	(*instance).sendSPIpacket((data>>8)&0xFF);
	(*instance).sendSPIpacket(data&0xFF);
}

void LCD_CtrlWrite_ILI9225(ili9225_instance *instance, uint16_t registerNumber, uint16_t registerValue){
	// MSByte & MSBit sent first

	// register index
	//(*instance).enableCS();
	ili9225_sendCommand(instance, registerNumber);
	//(*instance).disableCS();
	// register data
	//(*instance).enableCS();
	ili9225_sendData(instance, registerValue);
	//(*instance).disableCS();

	return;
}

bool ili9225_init(ili9225_instance *instance){
	bool response = 0;
	(*instance).disableCS();
	// following based on appnote
	// VCI=2.8V
	//************* Reset LCD Driver ****************//
	(*instance).enableReset();
	(*instance).delay50ms(); // This delay time is necessary (sidenote: was 10ms officially)
	(*instance).disableReset();
	(*instance).delay50ms();

	 //************* Start Initial Sequence **********//
	(*instance).enableCS();
#ifdef ORIENTATION_PORTRAIT
	LCD_CtrlWrite_ILI9225(instance, 0x0001, 0x011C); // set SS and NL bit, portrait mode
#else
	LCD_CtrlWrite_ILI9225(instance, 0x0001, (0<<10)|(0<<9)|(0<<8)|(0b11100)); // set SS and NL bit, landscape mode
#endif

	 LCD_CtrlWrite_ILI9225(instance, 0x0002, 0x0100); // set 1 line inversion
	 LCD_CtrlWrite_ILI9225(instance, 0x0003, 0x1030); // set GRAM write direction and BGR=1.
	 LCD_CtrlWrite_ILI9225(instance, 0x0008, 0x0808); // set BP and FP
	 LCD_CtrlWrite_ILI9225(instance, 0x000C, 0x0000); // RGB interface setting R0Ch=0x0110 for RGB 18Bit and R0Ch=0111for RGB16Bit
	 LCD_CtrlWrite_ILI9225(instance, 0x000F, 0x0801); // Set frame rate
	 LCD_CtrlWrite_ILI9225(instance, 0x0020, 0x0000); // Set GRAM Address
	 LCD_CtrlWrite_ILI9225(instance, 0x0021, 0x0000); // Set GRAM Address
	 //*************Power On sequence ****************//
	 (*instance).delay50ms(); // Delay 50ms
	 LCD_CtrlWrite_ILI9225(instance, 0x0010, 0x0A00); // Set SAP,DSTB,STB
	 LCD_CtrlWrite_ILI9225(instance, 0x0011, 0x1038); // Set APON,PON,AON,VCI1EN,VC
	 (*instance).delay50ms(); // Delay 50ms
	 LCD_CtrlWrite_ILI9225(instance, 0x0012, 0x1121); // Internal reference voltage= Vci;
	 LCD_CtrlWrite_ILI9225(instance, 0x0013, 0x0066); // Set GVDD
	 LCD_CtrlWrite_ILI9225(instance, 0x0014, 0x5F60); // Set VCOMH/VCOML voltage
	//------------------------ Set GRAM area --------------------------------//
	 LCD_CtrlWrite_ILI9225 (instance, 0x30, 0x0000);
	 LCD_CtrlWrite_ILI9225 (instance, 0x31, 0x00DB);
	 LCD_CtrlWrite_ILI9225 (instance, 0x32, 0x0000);
	LCD_CtrlWrite_ILI9225 (instance, 0x33, 0x0000);
	 LCD_CtrlWrite_ILI9225 (instance, 0x34, 0x00DB);
	 LCD_CtrlWrite_ILI9225 (instance, 0x35, 0x0000);
	 LCD_CtrlWrite_ILI9225 (instance, 0x36, 0x00AF);
	 LCD_CtrlWrite_ILI9225 (instance, 0x37, 0x0000);
	 LCD_CtrlWrite_ILI9225 (instance, 0x38, 0x00DB);
	 LCD_CtrlWrite_ILI9225 (instance, 0x39, 0x0000);
	// ----------- Adjust the Gamma Curve ----------//
	 LCD_CtrlWrite_ILI9225(instance, 0x0050, 0x0400);
	 LCD_CtrlWrite_ILI9225(instance, 0x0051, 0x060B);
	 LCD_CtrlWrite_ILI9225(instance, 0x0052, 0x0C0A);
	 LCD_CtrlWrite_ILI9225(instance, 0x0053, 0x0105);
	 LCD_CtrlWrite_ILI9225(instance, 0x0054, 0x0A0C);
	 LCD_CtrlWrite_ILI9225(instance, 0x0055, 0x0B06);
	 LCD_CtrlWrite_ILI9225(instance, 0x0056, 0x0004);
	 LCD_CtrlWrite_ILI9225(instance, 0x0057, 0x0501);
	 LCD_CtrlWrite_ILI9225(instance, 0x0058, 0x0E00);
	 LCD_CtrlWrite_ILI9225(instance, 0x0059, 0x000E);
	 (*instance).delay50ms(); // Delay 50ms
	 LCD_CtrlWrite_ILI9225(instance, 0x0007, 0x1017);
	 (*instance).disableCS();




	//response |= ili9225_sendCommand(instance, 0x);
	return response;
}



bool ili9225_setWindow(ili9225_instance *instance, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
   /* _orientCoordinates(x0, y0);
    _orientCoordinates(x1, y1);

    if (x1<x0) _swap(x0, x1);
    if (y1<y0) _swap(y0, y1);*/

    //startWrite();
	(*instance).enableCS();

#ifdef ORIENTATION_PORTRAIT
	LCD_CtrlWrite_ILI9225(instance,0x36,x1);
	LCD_CtrlWrite_ILI9225(instance,0x37,x0);

	LCD_CtrlWrite_ILI9225(instance,0x38,y1);
	LCD_CtrlWrite_ILI9225(instance,0x39,y0);

	LCD_CtrlWrite_ILI9225(instance,0x20,x0);
	LCD_CtrlWrite_ILI9225(instance,0x21,y0);
#else
	LCD_CtrlWrite_ILI9225(instance,0x36,y1);
	LCD_CtrlWrite_ILI9225(instance,0x37,y0);

	LCD_CtrlWrite_ILI9225(instance,0x38,x1);
	LCD_CtrlWrite_ILI9225(instance,0x39,x0);

	LCD_CtrlWrite_ILI9225(instance,0x20,y0);
	LCD_CtrlWrite_ILI9225(instance,0x21,x0);
#endif
	ili9225_sendCommand(instance, 0x00);
	ili9225_sendCommand(instance, 0x22);



	//(*instance).clrRS();
	//(*instance).sendSPIpacket(0x00);
	//(*instance).sendSPIpacket(0x22);
	(*instance).disableCS();

    //endWrite();
	return 0; // success
}

bool ili9225_fillRectangle(ili9225_instance *instance, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {

	ili9225_setWindow(instance, x1, y1, x2, y2);

	uint16_t t;
	(*instance).enableCS();
	(*instance).setRS();
    //for(t=(y2 - y1 + 1) * (x2 - x1 + 1); t > 0; t--){
    for(t=(y2 - y1 + 1) * (x2 - x1 + 1); t; t--){
    	//ili9225_sendData(instance, color >> 8);
    	ili9225_sendData_RSpreset(instance, color);
}
    (*instance).disableCS();
    	return 0; // success
}

bool ili9225_emptyRectangle(ili9225_instance *instance, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
	uint16_t t;

	ili9225_setWindow(instance, x1, y1, x1, y2);

	(*instance).enableCS();
	(*instance).setRS();
	for(t=y2-y1; t; t--){
		ili9225_sendData_RSpreset(instance, color);
	}
	(*instance).disableCS();

	ili9225_setWindow(instance, x2, y1, x2, y2);

	(*instance).enableCS();
	(*instance).setRS();
	for(t=y2-y1; t; t--){
		ili9225_sendData_RSpreset(instance, color);
	}
	(*instance).disableCS();

	ili9225_setWindow(instance, x1, y1, x2, y1);

	(*instance).enableCS();
	(*instance).setRS();
	for(t=x2-x1; t; t--){
		ili9225_sendData_RSpreset(instance, color);
	}
	(*instance).disableCS();

	ili9225_setWindow(instance, x1, y2, x2, y2);

	(*instance).enableCS();
	(*instance).setRS();
	for(t=x2-x1; t; t--){
		ili9225_sendData_RSpreset(instance, color);
	}
	(*instance).disableCS();



    return 0; // success
}
