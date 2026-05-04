/*
 * main.c — Robo-V SoC firmware
 *
 * Top-level control loop:
 *   1. Send Dynamixel READ_POSITION command via uart_dynamixel
 *   2. Wait for response, get raw encoder position
 *   3. Push raw position through FIR filter (noise suppression)
 *   4. Store filtered position (PID controller stub — Week 4+)
 *
 * Memory map (matches rtl/ register maps exactly):
 *   0x0002_0000  DSP engine — fir_filter.sv
 *   0x0003_0080  HPC engine — uart_dynamixel.sv (adr_i[7]=1 in hpc_regfile)
 */

#include <stdint.h>

 //DSP engine — fir_filter.sv register offsets
#define FIR_BASE 0x00020000u
#define FIR_DATA_IN (*(volatile uint32_t *)(FIR_BASE + 0x00))
#define FIR_DATA_OUT (*(volatile uint32_t *)(FIR_BASE + 0x04))
#define FIR_STATUS (*(volatile uint32_t *)(FIR_BASE + 0x08))

#define FIR_OUT_VALID (1u << 0)

//HPC engine — uart_dynamixel.sv at 0x0003_0080
//(hpc_regfile routes adr_i[7]=1 to uart_dynamixel)
#define UART_BASE 0x00030080u
#define UART_CTRL (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_STATUS (*(volatile uint32_t *)(UART_BASE + 0x04))
#define UART_BAUD_DIV (*(volatile uint32_t *)(UART_BASE + 0x08))
#define UART_TX_ID (*(volatile uint32_t *)(UART_BASE + 0x0C))
#define UART_RX_POS (*(volatile uint32_t *)(UART_BASE + 0x10))
#define UART_RX_ERR (*(volatile uint32_t *)(UART_BASE + 0x14))

#define UART_TX_BUSY (1u << 0)
#define UART_RX_BUSY (1u << 1)
#define UART_RX_DONE (1u << 2)
#define UART_CRC_ERR (1u << 3)
#define UART_TIMEOUT (1u << 4)

//Shared state (written by control loop, read by future PID code)
volatile int32_t g_filtered_pos = 0;
volatile uint8_t g_motor_error = 0;

//Main control loop
int main(void)
{
    /* One-time setup: servo ID=1, default baud 868 (50MHz/57600) */
    UART_TX_ID = 1;
    UART_BAUD_DIV = 868;

    while (1) {
        //Step 1: request position from servo
        UART_CTRL = 1;
        while (UART_STATUS & UART_TX_BUSY);

        //Step 2: wait for STATUS response
        uint32_t status;
        do {
            status = UART_STATUS;
        } while (!(status & (UART_RX_DONE | UART_CRC_ERR | UART_TIMEOUT)));

        if (status & (UART_CRC_ERR | UART_TIMEOUT)) {
            //Communication error — retry next iteration
            continue;
        }

        int32_t raw_pos = (int32_t)UART_RX_POS;
        g_motor_error = (uint8_t)UART_RX_ERR;

        //Step 3: push raw position through FIR filter
        //FIR accepts signed Q1.15 (16-bit); raw_pos is 12-bit (0–4095)
        //Scale: shift left by 3 to occupy upper bits of 15-bit range
        int16_t sample_q15 = (int16_t)(raw_pos & 0xFFFF);
        FIR_DATA_IN = (uint32_t)(uint16_t)sample_q15;

        while (!(FIR_STATUS & FIR_OUT_VALID));

        int16_t filtered = (int16_t)(FIR_DATA_OUT & 0xFFFF);

        //Step 4: store result (PID stub — Week 4 adds P/I/D terms)
        g_filtered_pos = (int32_t)filtered;
    }

    return 0;
}
