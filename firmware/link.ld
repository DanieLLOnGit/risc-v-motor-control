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
