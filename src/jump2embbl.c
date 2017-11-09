/*
 * jump2embbl.c
 *
 *  Created on: 4 Aug 2017
 *      Author: denry
 */

#include "stm32f10x.h"

//jump to bootloader if necessary

#define EMB_BL_SP_ADDR 			(0x1FFFF000)	// Embedded bootloader address
#define EMB_BL_ADDR_OFFSET		(0x00000004)	// Embedded bootloader reset vector offset (relative to address)
//required to put quotation marks around defines
#define DEF2STR(x) #x
#define STR(x) DEF2STR(x)

void checkOnBootBLreq(){
	if(BKP->DR1 == 0x0BAD){
		if(BKP->DR2 == 0xC0DE){

			//clear request for next boots
			RCC->APB1ENR |= RCC_APB1ENR_PWREN; //enable clock to power interface
			RCC->APB1ENR |= RCC_APB1ENR_BKPEN; //enable clock to BKP
			PWR->CR |= PWR_CR_DBP; //allow writing to RTC & backup registers
			BKP->DR1 = 0;
			BKP->DR2 = 0;
			PWR->CR &= ~((uint32_t)(PWR_CR_DBP)); //forbid writing to RTC & backup registers

			//perform the jump
			//stack pointer initial value to stack pointer
			asm volatile("ldr r0, =" STR(EMB_BL_SP_ADDR));
			asm volatile("ldr r0, [r0]");  //not sure why, but this line is very necessary
			asm volatile("mov sp, r0");

			//reset vector address to program counter
			asm volatile("ldr r0, =" STR(EMB_BL_SP_ADDR + EMB_BL_ADDR_OFFSET));
			asm volatile("ldr r0, [r0]"); //not sure why, but this line is very necessary
			asm volatile("mov pc, r0");
		}
	}

	return; //it appears no request has been made
}
