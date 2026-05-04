# crt0.s — minimal startup for Robo-V SoC (RV32I bare-metal)
# Sets stack pointer, calls main, traps on return.

    .section .text.start
    .global _start

_start:
    la sp, _stack_top # load stack top address (defined in link.ld)
    call main # call C entry point
_halt:
    j _halt # loop forever if main returns
