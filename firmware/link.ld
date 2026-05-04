/*
 * link.ld — linker script for Robo-V SoC
 *
 * Memory map:
 *   IMEM 0x0000_0000 4 KB (instruction memory, read-only at runtime)
 *   DMEM 0x0001_0000 4 KB (data memory, stack lives at top)
 */

MEMORY
{
    IMEM (rx) : ORIGIN = 0x00000000, LENGTH = 4K
    DMEM (rwx) : ORIGIN = 0x00010000, LENGTH = 4K
}

SECTIONS
{
    /* Startup code first, then all other text */
    .text :
    {
        *(.text.start)
        *(.text*)
        *(.rodata*)
    } > IMEM

    /* Initialised data in DMEM */
    .data :
    {
        *(.data*)
    } > DMEM

    /* Zero-initialised data (BSS) */
    .bss :
    {
        *(.bss*)
        *(COMMON)
    } > DMEM

    /* Stack top: end of DMEM minus one word */
    _stack_top = ORIGIN(DMEM) + LENGTH(DMEM) - 4;
}
