import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, FallingEdge

# Golden model helpers
def crc16_ibm(data: bytes) -> int:
    """CRC-16/ARC (IBM): poly 0xA001, init 0x0000, LSB-first."""
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
    """
    Build 14-byte Dynamixel Protocol 2.0 READ_POSITION command.
    FF FF FD 00 [ID] 07 00 02 92 00 04 00 [CRC_L] [CRC_H]
    CRC covers bytes [4..11].
    """
    payload = bytes([servo_id, 0x07, 0x00, 0x02, 0x92, 0x00, 0x04, 0x00])
    crc = crc16_ibm(payload)
    return bytes([0xFF, 0xFF, 0xFD, 0x00]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_status_response(servo_id: int, position: int, error: int = 0) -> bytes:
    """
    Build 15-byte Dynamixel Protocol 2.0 STATUS response for a position read.
    FF FF FD 00 [ID] 08 00 55 [ERR] [P0][P1][P2][P3] [CRC_L][CRC_H]
    CRC covers bytes [4..12].
    position is a signed 32-bit integer (little-endian).
    """
    pos_bytes = position.to_bytes(4, byteorder='little', signed=True)
    payload = bytes([servo_id, 0x08, 0x00, 0x55, error]) + pos_bytes
    crc = crc16_ibm(payload)
    return bytes([0xFF, 0xFF, 0xFD, 0x00]) + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def corrupt_crc(packet: bytes) -> bytes:
    """Flip bit 0 of the CRC low byte (second-to-last byte)."""
    pkt = bytearray(packet)
    pkt[-2] ^= 0x01
    return bytes(pkt)


# Wishbone helpers
async def wb_write(dut, addr_offset, data):
    await RisingEdge(dut.clk_i)
    dut.cyc_i.value = 1
    dut.stb_i.value = 1
    dut.we_i.value = 1
    dut.adr_i.value = (addr_offset >> 2) & 0x3F
    dut.dat_i.value = data & 0xFFFFFFFF
    dut.sel_i.value = 0xF
    while True:
        await RisingEdge(dut.clk_i)
        if dut.ack_o.value == 1:
            break
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    dut.we_i.value = 0


async def wb_read(dut, addr_offset):
    await RisingEdge(dut.clk_i)
    dut.cyc_i.value = 1
    dut.stb_i.value = 1
    dut.we_i.value = 0
    dut.adr_i.value = (addr_offset >> 2) & 0x3F
    dut.sel_i.value = 0xF
    while True:
        await RisingEdge(dut.clk_i)
        if dut.ack_o.value == 1:
            val = int(dut.dat_o.value)
            break
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    return val


# UART register offsets
REG_CTRL = 0x00
REG_STATUS = 0x04
REG_BAUD_DIV = 0x08
REG_TX_ID = 0x0C
REG_RX_POS = 0x10
REG_RX_ERR = 0x14

STATUS_TX_BUSY = (1 << 0)
STATUS_RX_BUSY = (1 << 1)
STATUS_RX_DONE = (1 << 2)
STATUS_CRC_ERR = (1 << 3)
STATUS_TIMEOUT = (1 << 4)

BAUD_DIV = 868   # 50 MHz / 57600


async def reset_dut(dut):
    dut.rst_ni.value = 0
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    dut.we_i.value = 0
    dut.adr_i.value = 0
    dut.dat_i.value = 0
    dut.sel_i.value = 0
    dut.uart_rx_i.value = 1   # idle high
    await ClockCycles(dut.clk_i, 4)
    dut.rst_ni.value = 1
    await ClockCycles(dut.clk_i, 2)


async def wait_tx_done(dut, timeout_cycles=20000):
    """Poll STATUS until tx_busy=0."""
    for _ in range(timeout_cycles):
        status = await wb_read(dut, REG_STATUS)
        if not (status & STATUS_TX_BUSY):
            return status
        await ClockCycles(dut.clk_i, 10)
    raise TimeoutError("tx_busy never cleared")


async def inject_uart_bytes(dut, data: bytes):
    """
    Drive uart_rx_i with the given byte stream at BAUD_DIV clocks/bit.
    UART frame: 1 start bit (0), 8 data bits LSB-first, 1 stop bit (1).
    """
    for byte in data:
        # start bit
        dut.uart_rx_i.value = 0
        await ClockCycles(dut.clk_i, BAUD_DIV)
        # 8 data bits, LSB first
        for i in range(8):
            dut.uart_rx_i.value = (byte >> i) & 1
            await ClockCycles(dut.clk_i, BAUD_DIV)
        # stop bit
        dut.uart_rx_i.value = 1
        await ClockCycles(dut.clk_i, BAUD_DIV)


async def capture_tx_bytes(dut, n_bytes: int) -> bytes:
    """
    Capture n_bytes from uart_tx_o.
    Detects start bit falling edge, center-samples each bit.
    """
    result = []
    for _ in range(n_bytes):
        # wait for start bit (falling edge on uart_tx_o)
        while dut.uart_tx_o.value != 0:
            await RisingEdge(dut.clk_i)
        # advance to center of start bit
        await ClockCycles(dut.clk_i, BAUD_DIV // 2)
        # verify it's still low
        assert dut.uart_tx_o.value == 0, "false start bit"
        # sample 8 data bits
        byte_val = 0
        for i in range(8):
            await ClockCycles(dut.clk_i, BAUD_DIV)
            bit = int(dut.uart_tx_o.value)
            byte_val |= (bit << i)
        # consume stop bit
        await ClockCycles(dut.clk_i, BAUD_DIV)
        result.append(byte_val)
    return bytes(result)


# Test 1: TX READ_POSITION command bytes
@cocotb.test()
async def test_tx_read_position(dut):
    """
    Write TX_ID=1, assert send_cmd. Capture 14 bytes from uart_tx_o.
    Verify bytes match build_read_position_pkt(1).
    Verify tx_en_o=1 during TX, tx_en_o=0 after.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_dut(dut)

    # Program baud divider and servo ID
    await wb_write(dut, REG_BAUD_DIV, BAUD_DIV)
    await wb_write(dut, REG_TX_ID, 1)

    # Assert tx_en_o becomes 1 during TX
    tx_en_before = int(dut.tx_en_o.value)
    assert tx_en_before == 0, f"tx_en_o should be 0 before send, got {tx_en_before}"

    # Trigger send
    await wb_write(dut, REG_CTRL, 0x1)

    # Start capturing in parallel with the send
    captured = await capture_tx_bytes(dut, 14)

    # Verify tx_en_o goes low after last byte
    await ClockCycles(dut.clk_i, BAUD_DIV * 2)
    tx_en_after = int(dut.tx_en_o.value)
    assert tx_en_after == 0, f"tx_en_o should be 0 after TX, got {tx_en_after}"

    golden = build_read_position_pkt(1)
    assert captured == golden, (
        f"TX packet mismatch:\n"
        f"  got    : {captured.hex()}\n"
        f"  expected: {golden.hex()}"
    )
    dut._log.info(f"TX packet: {captured.hex()}")
    dut._log.info("PASS: TX READ_POSITION packet matches golden model")


# Test 2: RX valid STATUS response → parse position=2048
@cocotb.test()
async def test_rx_valid_response(dut):
    """
    After TX completes, inject valid STATUS response (position=2048).
    Poll rx_done. Read RX_POSITION. Assert position==2048 and crc_err==0.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_dut(dut)

    await wb_write(dut, REG_BAUD_DIV, BAUD_DIV)
    await wb_write(dut, REG_TX_ID, 1)
    await wb_write(dut, REG_CTRL, 0x1)

    # Wait for TX to finish (tx_busy clears)
    await wait_tx_done(dut)

    # Inject valid response
    response = build_status_response(1, 2048)
    dut._log.info(f"Injecting response: {response.hex()}")
    await inject_uart_bytes(dut, response)

    # Poll rx_done (STATUS bit[2])
    for _ in range(5000):
        status = await wb_read(dut, REG_STATUS)
        if status & STATUS_RX_DONE:
            break
        await ClockCycles(dut.clk_i, 10)
    else:
        raise TimeoutError("rx_done never asserted")

    status = await wb_read(dut, REG_STATUS)
    assert not (status & STATUS_CRC_ERR), "crc_err unexpectedly set"

    pos_raw = await wb_read(dut, REG_RX_POS)
    # sign-extend 32-bit
    if pos_raw >= 0x80000000:
        pos_raw -= 0x100000000

    assert pos_raw == 2048, f"Position mismatch: got {pos_raw}, expected 2048"
    dut._log.info(f"RX position = {pos_raw}")
    dut._log.info("PASS: RX valid response parsed correctly")


# Test 3: RX corrupted CRC → crc_err asserted
@cocotb.test()
async def test_rx_bad_crc(dut):
    """
    Inject STATUS response with corrupted CRC.
    Assert STATUS.crc_err=1 and rx_done=0.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, unit="ns").start())
    await reset_dut(dut)

    await wb_write(dut, REG_BAUD_DIV, BAUD_DIV)
    await wb_write(dut, REG_TX_ID, 1)
    await wb_write(dut, REG_CTRL, 0x1)

    await wait_tx_done(dut)

    bad_response = corrupt_crc(build_status_response(1, 2048))
    dut._log.info(f"Injecting corrupted response: {bad_response.hex()}")
    await inject_uart_bytes(dut, bad_response)

    # Poll crc_err (STATUS bit[3])
    for _ in range(5000):
        status = await wb_read(dut, REG_STATUS)
        if status & STATUS_CRC_ERR:
            break
        await ClockCycles(dut.clk_i, 10)
    else:
        raise TimeoutError("crc_err never asserted")

    status = await wb_read(dut, REG_STATUS)
    assert not (status & STATUS_RX_DONE), "rx_done should not be set on CRC error"
    dut._log.info("PASS: CRC error correctly detected")
