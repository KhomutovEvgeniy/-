#pragma once

//#define DEBUF_VIEWER_PRINTF_USE

#ifdef DEBUF_VIEWER_PRINTF_USE

	#include <stdio.h>
	#include <stm32f4xx.h>

	#define    DWT_CYCCNT    *(volatile uint32_t *)0xE0001004
	#define    DWT_CONTROL   *(volatile uint32_t *)0xE0001000
	#define    SCB_DEMCR     *(volatile uint32_t *)0xE000EDFC


	static void DWT_Init()
	{
	   SCB_DEMCR  |= 0x01000000;
	   DWT_CYCCNT  = 0;
	   DWT_CONTROL|= 1; // enable the counter
	}

	#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
	#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
	#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

	#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
	#define TRCENA          0x01000000

	int fputc(int ch, FILE *f);
	
#else
	#include <stdbool.h>
	#include <stdio.h>
	#define printf(s) __NOP()// asm("nop")
	#define DWT_Init() __NOP() // asm("nop")
#endif

