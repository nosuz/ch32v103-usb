/* CH32V103R8T6
    CH32V103C8T6 and CH32V103C8U6 have the same memory map.
*/
/* 1K = 1 KiBi = 1024 bytes */
MEMORY
{
    /* Code Flash, 64KB max */
	FLASH : ORIGIN = 0x0800 * 0x10000, LENGTH = 64k
    /* SRAM, 20KB max */
	RAM : ORIGIN = 0x2000 * 0x10000, LENGTH = 20k
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);
