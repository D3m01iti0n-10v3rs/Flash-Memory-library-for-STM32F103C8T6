/*
 * flash_stm32f103.c
 *
 *  Created on: Sep 29, 2026
 *      Author: PC
 */

#include "flash_stm32f103.h"

static inline void Flash_WriteKey(void){
	FLASH_KEYR = FLASH_KEY1;
	FLASH_KEYR = FLASH_KEY2;
}

static inline void Flash_ErrorHandler(void){
	FLASH_SR |= FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR;
	FLASH_CR &= ~(FLASH_CR_PER | FLASH_CR_PG);
	FLASH_CR |= FLASH_CR_LOCK;
}

static inline void Flash_Begin(void){
    FLASH_SR = (FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR);
    if (FLASH_CR & FLASH_CR_LOCK) Flash_WriteKey();
    while (FLASH_SR & FLASH_SR_BSY);
}

static inline void Flash_End(void){
    FLASH_CR &= ~(FLASH_CR_PG | FLASH_CR_PER);
    FLASH_SR = (FLASH_SR_EOP | FLASH_SR_WRPRTERR | FLASH_SR_PGERR);
    FLASH_CR |= FLASH_CR_LOCK;
}

static inline uint32_t Flash_ProgramHlfWord(uint32_t addr, uint16_t data){
    FLASH_CR |= FLASH_CR_PG;
    *(volatile uint16_t *)addr = data;
    while (FLASH_SR & FLASH_SR_BSY);
    return !(FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR));
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
    uint16_t hw = (addr & 1) ? (uint16_t)((data << 8) | 0xFF)
                             : (uint16_t)(0xFF00 | data);
    return Flash_WriteHlfWord(addr & ~1UL, hw);
}

uint32_t Flash_WriteHlfWord(uint32_t addr, uint16_t data){
    Flash_Begin();
    uint32_t ok = Flash_ProgramHlfWord(addr, data);
    Flash_End();
    return ok;
}

uint32_t Flash_WriteWord(uint32_t addr, uint32_t data){
    Flash_Begin();
    uint32_t ok = Flash_ProgramHlfWord(addr,     (uint16_t)(data & 0xFFFF))
               && Flash_ProgramHlfWord(addr + 2, (uint16_t)(data >> 16));
    Flash_End();
    return ok;
}
