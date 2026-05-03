"""
Cocotb integration testbench for soc.sv
Observes physical ports only: uart_tx_o, uart_rx_i, tx_en_o, clk_i, rst_ni.
No Wishbone access — the CPU drives all transactions internally.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

#Golden model helpers (reused from test_uart.py)

def crc16_ibm(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        for _ in range(8):
            if (crc ^ byte) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            byte >>= 1
    return crc & 0xFFFF


def build_read_position_pkt(servo_id: int) -> bytes:
    payload = bytes([servo_id, 0x07, 0x00, 0x02, 0x92, 0x00, 0x04, 0x00])
    crc = crc16_ibm(payload)
    return bytes([0xFF, 0xFF, 0xFD, 0x00]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_status_response(servo_id: int, position: int, error: int = 0) -> bytes:
    pos_bytes = position.to_bytes(4, byteorder='little', signed=True)
    payload = bytes([servo_id, 0x08, 0x00, 0x55, error]) + pos_bytes
    crc = crc16_ibm(payload)
    return bytes([0xFF, 0xFF, 0xFD, 0x00]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# UART helpers
BAUD_DIV = 868


async def wait_for_tx_start(dut, timeout_cycles: int = 600_000) -> bool:
    """Wait for uart_tx_o to go low (start bit). Returns True if detected."""
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk_i)
        if int(dut.uart_tx_o.value) == 0:
            return True
    return False


async def capture_tx_bytes(dut, n_bytes: int) -> bytes:
    """
    Capture n_bytes from uart_tx_o.
    Called after the start bit of the first byte is already low.
    Center-samples each bit at BAUD_DIV/2 then BAUD_DIV thereafter.
    """
    result = []
    for byte_idx in range(n_bytes):
        if byte_idx > 0:
            # wait for start bit of next byte
            while int(dut.uart_tx_o.value) != 0:
                await RisingEdge(dut.clk_i)

        # advance to center of start bit
        await ClockCycles(dut.clk_i, BAUD_DIV // 2)
        assert int(dut.uart_tx_o.value) == 0, f"byte {byte_idx}: false start bit"

        # sample 8 data bits
        byte_val = 0
        for i in range(8):
            await ClockCycles(dut.clk_i, BAUD_DIV)
            byte_val |= (int(dut.uart_tx_o.value) << i)

        # consume stop bit
        await ClockCycles(dut.clk_i, BAUD_DIV)
        result.append(byte_val)

    return bytes(result)


async def inject_uart_bytes(dut, data: bytes):
    """Drive uart_rx_i with byte stream at BAUD_DIV clocks/bit."""
    for byte in data:
        dut.uart_rx_i.value = 0
        await ClockCycles(dut.clk_i, BAUD_DIV)
        for i in range(8):
            dut.uart_rx_i.value = (byte >> i) & 1
            await ClockCycles(dut.clk_i, BAUD_DIV)
        dut.uart_rx_i.value = 1
        await ClockCycles(dut.clk_i, BAUD_DIV)


async def reset_soc(dut):
    dut.rst_ni.value = 0
    dut.uart_rx_i.value = 1
    await ClockCycles(dut.clk_i, 8)
    dut.rst_ni.value = 1
    await ClockCycles(dut.clk_i, 2)


# Test 1 — CPU boots and issues READ_POSITION command
@cocotb.test()
async def test_cpu_boots_and_tx(dut):
    """
    After reset, the CPU should boot from IMEM, initialise uart_dynamixel,
    and issue a READ_POSITION command. Verify the 14 TX bytes.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_soc(dut)

    dut._log.info("Waiting for first Dynamixel TX packet...")
    detected = await wait_for_tx_start(dut, timeout_cycles=600_000)
    assert detected, "uart_tx_o never went low — CPU did not boot or firmware stalled"

    captured = await capture_tx_bytes(dut, 14)
    golden = build_read_position_pkt(1)

    assert captured == golden, (
        f"TX packet mismatch:\n"
        f"  got     : {captured.hex()}\n"
        f"  expected: {golden.hex()}"
    )

    # tx_en_o should have been high during TX and back low now
    await ClockCycles(dut.clk_i, BAUD_DIV * 2)
    assert int(dut.tx_en_o.value) == 0, "tx_en_o still high after TX"

    dut._log.info(f"TX packet: {captured.hex()}")
    dut._log.info("PASS: CPU booted and issued correct READ_POSITION command")


# Test 2 — CPU loops: inject response, verify second TX
@cocotb.test()
async def test_rx_triggers_loop(dut):
    """
    After the first TX, inject a valid STATUS response.
    The CPU should process the position, push it through the FIR filter,
    then loop back and issue another READ_POSITION command.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_soc(dut)

    # Wait for first TX
    detected = await wait_for_tx_start(dut, timeout_cycles=600_000)
    assert detected, "First TX never appeared"
    await capture_tx_bytes(dut, 14)   # consume first packet

    # Inject a valid servo response (position = 1024)
    response = build_status_response(1, 1024)
    dut._log.info(f"Injecting response: {response.hex()}")
    await inject_uart_bytes(dut, response)

    # CPU processes response + FIR filter (FIR takes 32 cycles in COMPUTE state)
    # then loops back.  Allow up to 600k cycles for second TX.
    dut._log.info("Waiting for second TX (firmware loop)...")
    detected = await wait_for_tx_start(dut, timeout_cycles=600_000)
    assert detected, "Second TX never appeared — CPU did not loop back"

    captured = await capture_tx_bytes(dut, 14)
    golden = build_read_position_pkt(1)
    assert captured == golden, (
        f"Second TX packet mismatch:\n"
        f"  got     : {captured.hex()}\n"
        f"  expected: {golden.hex()}"
    )

    dut._log.info("PASS: CPU looped correctly after RX response")


# Test 3 — Reset recovery: CPU re-boots cleanly mid-run
@cocotb.test()
async def test_reset_recovery(dut):
    """
    Assert reset mid-execution (after first TX), release, verify CPU
    re-boots and issues a fresh READ_POSITION command.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_soc(dut)

    # Wait for first TX to confirm CPU is running
    detected = await wait_for_tx_start(dut, timeout_cycles=600_000)
    assert detected, "First TX never appeared before reset test"

    # Assert reset mid-stream (don't bother consuming the packet)
    dut._log.info("Asserting reset mid-run...")
    dut.rst_ni.value = 0
    await ClockCycles(dut.clk_i, 8)
    dut.rst_ni.value = 1
    dut.uart_rx_i.value = 1   # ensure RX is idle after reset

    # CPU should re-boot and issue TX again
    dut._log.info("Waiting for TX after reset recovery...")
    detected = await wait_for_tx_start(dut, timeout_cycles=600_000)
    assert detected, "TX never appeared after reset — CPU did not recover"

    captured = await capture_tx_bytes(dut, 14)
    golden = build_read_position_pkt(1)
    assert captured == golden, (
        f"Post-reset TX mismatch:\n"
        f"  got     : {captured.hex()}\n"
        f"  expected: {golden.hex()}"
    )

    dut._log.info("PASS: CPU recovered cleanly from mid-run reset")
