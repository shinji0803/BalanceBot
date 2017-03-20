/* Linker script Upperside for STM32F303K8T6! 	*/


OUTPUT_FORMAT ("elf32-littlearm")

/* Memory Spaces Definitions */
MEMORY
{
	RAM1 (xrw)	: ORIGIN = 0x20000000, LENGTH = 12k		/* Main Embedded SRAM 					*/
	RAM3 (xrw)	: ORIGIN = 0x10000000, LENGTH = 4k		/* Core-Coupled SRAM 					*/
	ROM (rx)	: ORIGIN = 0x08000000, LENGTH = 64K		/* Main Embedded FlashROM				*/
}

/* higher address of the stack bottom */
_estack = ORIGIN(RAM3)+LENGTH(RAM3);

/* higher address of the heap end */
_heap_end = ORIGIN(RAM1)+LENGTH(RAM1)-4;	/* due to 4byte alignments */


/* include the section management sub-script */
INCLUDE "STM32F3_FLASH.ld"
