/*
 * flash_stm32f103.c
 *
 *  Created on: Sep 29, 2026
 *      Author: PC
 */

#include "flash_stm32f103.h"

static void Flash_WriteKey(void){
	FLASH_KEYR = FLASH_KEY1;
	FLASH_KEYR = FLASH_KEY2;
}

static void Flash_ErrorHandler(void){
	FLASH_SR |= FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;
	FLASH_CR &= ~(FLASH_CR_PER | FLASH_CR_PG);
	FLASH_CR |= FLASH_CR_LOCK;
}

uint32_t Flash_ErasePage(uint32_t addr){
	FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

	if (FLASH_CR & FLASH_CR_LOCK) Flash_WriteKey();
	while(FLASH_SR & FLASH_SR_BSY);

	FLASH_CR |= FLASH_CR_PER;
	FLASH_AR = addr & ~0x3FFUL;
	FLASH_CR |= FLASH_CR_STRT;
	while(FLASH_SR & FLASH_SR_BSY);

	if (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)){
		Flash_ErrorHandler();
		return 0;
	}
	else FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

	FLASH_CR &= ~FLASH_CR_PER;
	FLASH_CR |= FLASH_CR_LOCK;

	return 1;
}

uint8_t Flash_ReadByte(uint32_t addr){
	return *(volatile uint8_t *)addr;
}

uint16_t Flash_ReadHlfWord(uint32_t addr){
	return *(volatile uint16_t *)addr;
}

uint32_t Flash_ReadWord(uint32_t addr){
	return *(volatile uint32_t *)addr;
}

uint32_t Flash_WriteByte(uint32_t addr, uint8_t data){
	FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

	if (FLASH_CR & FLASH_CR_LOCK) Flash_WriteKey();
	while(FLASH_SR & FLASH_SR_BSY);

	FLASH_CR |= FLASH_CR_PG;
	while(FLASH_SR & FLASH_SR_BSY);

    *(volatile uint16_t *)addr = (uint16_t)data;
    while(FLASH_SR & FLASH_SR_BSY);

	if (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)){
		Flash_ErrorHandler();
		return 0;
	}
	else FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    FLASH_CR &= ~FLASH_CR_PG;
    FLASH_CR |= FLASH_CR_LOCK;

    return 1;
}

uint32_t Flash_WriteHlfWord(uint32_t addr, uint16_t data){
	FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

	if (FLASH_CR & FLASH_CR_LOCK) Flash_WriteKey();
	while(FLASH_SR & FLASH_SR_BSY);

	FLASH_CR |= FLASH_CR_PG;
	while(FLASH_SR & FLASH_SR_BSY);

    *(volatile uint16_t *)addr = data;
    while(FLASH_SR & FLASH_SR_BSY);

	if (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)){
		Flash_ErrorHandler();
		return 0;
	}
	else FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    FLASH_CR &= ~FLASH_CR_PG;
    FLASH_CR |= FLASH_CR_LOCK;

    return 1;
}

uint32_t Flash_WriteWord(uint32_t addr, uint32_t data){
	FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

	if (FLASH_CR & FLASH_CR_LOCK) Flash_WriteKey();
	while(FLASH_SR & FLASH_SR_BSY);

	FLASH_CR |= FLASH_CR_PG;
	while(FLASH_SR & FLASH_SR_BSY);

    *(volatile uint16_t *)addr = (uint16_t)(data & 0xFFFF);
    while(FLASH_SR & FLASH_SR_BSY);

	if (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)){
		Flash_ErrorHandler();
		return 0;
	}
	else FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    *(volatile uint16_t *)(addr + 2) = (uint16_t)(data >> 16);
    while(FLASH_SR & FLASH_SR_BSY);

	if (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)){
		Flash_ErrorHandler();
		return 0;
	}
	else FLASH_SR = FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;

    FLASH_CR &= ~FLASH_CR_PG;
    FLASH_CR |= FLASH_CR_LOCK;

    return 1;
}
