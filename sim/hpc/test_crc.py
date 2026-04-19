import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge


# Python reference: CRC-16/IBM (ARC), poly=0x8005, init=0, refin, refout
def crc16_ibm_ref(data: bytes, crc: int = 0x0000) -> int:
    for byte in data:
        for i in range(8):
            bit = (byte >> i) & 1
            if (crc ^ bit) & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# Wishbone helpers
async def wb_write(dut, addr_offset, data):
    """Write a 32-bit word to register at byte offset addr_offset."""
    # addr_offset is byte address; adr_i is [7:2] (word address bits)
    await RisingEdge(dut.clk_i)
    dut.cyc_i.value = 1
    dut.stb_i.value = 1
    dut.we_i.value  = 1
    dut.adr_i.value = (addr_offset >> 2) & 0x3F   # 6-bit field [7:2]
    dut.dat_i.value = data & 0xFFFFFFFF
    dut.sel_i.value = 0xF
    # Wait for ACK
    while True:
        await RisingEdge(dut.clk_i)
        if dut.ack_o.value == 1:
            break
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    dut.we_i.value  = 0


async def wb_read(dut, addr_offset) -> int:
    """Read a 32-bit word from register at byte offset addr_offset."""
    await RisingEdge(dut.clk_i)
    dut.cyc_i.value = 1
    dut.stb_i.value = 1
    dut.we_i.value  = 0
    dut.adr_i.value = (addr_offset >> 2) & 0x3F
    dut.dat_i.value = 0
    dut.sel_i.value = 0xF
    while True:
        await RisingEdge(dut.clk_i)
        if dut.ack_o.value == 1:
            val = int(dut.dat_o.value)
            break
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    return val


async def reset_dut(dut):
    """Apply async reset for 5 cycles."""
    dut.rst_ni.value = 0
    dut.cyc_i.value  = 0
    dut.stb_i.value  = 0
    dut.we_i.value   = 0
    dut.adr_i.value  = 0
    dut.dat_i.value  = 0
    dut.sel_i.value  = 0
    for _ in range(5):
        await RisingEdge(dut.clk_i)
    dut.rst_ni.value = 1
    await RisingEdge(dut.clk_i)


async def feed_bytes(dut, data: bytes):
    """Feed each byte in data to DATA_IN register."""
    for b in data:
        await wb_write(dut, 0x00, b)


async def crc_soft_reset(dut):
    """Assert CTRL bit[0] to synchronously reset CRC to 0."""
    await wb_write(dut, 0x08, 0x1)


# Test 1 — Reset sanity
@cocotb.test()
async def test_reset_sanity(dut):
    """After reset, CRC_OUT must be 0x0000."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    crc = await wb_read(dut, 0x04)
    assert crc & 0xFFFF == 0x0000, f"Expected 0x0000 after reset, got 0x{crc:04X}"
    dut._log.info("PASS: CRC is 0x0000 after reset")


# Test 2 — Known vector: "123456789" → 0xBB3D
@cocotb.test()
async def test_known_vector(dut):
    """CRC of b'123456789' must equal 0xBB3D."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    payload = b"123456789"
    await feed_bytes(dut, payload)

    crc = await wb_read(dut, 0x04)
    assert crc & 0xFFFF == 0xBB3D, \
        f"Known vector FAIL: expected 0xBB3D, got 0x{crc & 0xFFFF:04X}"
    dut._log.info(f"PASS: CRC('123456789') = 0x{crc & 0xFFFF:04X}")


# Test 3 — Reset mid-stream
@cocotb.test()
async def test_reset_mid_stream(dut):
    """Reset mid-computation clears CRC; re-running known vector gives 0xBB3D."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    # Feed first 4 bytes of known vector, then soft-reset
    await feed_bytes(dut, b"1234")
    await crc_soft_reset(dut)

    crc = await wb_read(dut, 0x04)
    assert crc & 0xFFFF == 0x0000, \
        f"CRC not cleared after soft reset, got 0x{crc & 0xFFFF:04X}"

    # Re-run the full known vector from scratch
    await feed_bytes(dut, b"123456789")
    crc = await wb_read(dut, 0x04)
    assert crc & 0xFFFF == 0xBB3D, \
        f"Post-reset known vector FAIL: expected 0xBB3D, got 0x{crc & 0xFFFF:04X}"
    dut._log.info("PASS: reset mid-stream and re-run known vector")


# Test 4 — Random batch: 100 sequences cross-checked against Python reference
@cocotb.test()
async def test_random_batch(dut):
    """100 random byte sequences must match Python CRC-16/IBM reference."""
    import random
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    rng = random.Random(0xC0FFEE)
    failures = 0

    for seq_idx in range(100):
        length = rng.randint(1, 64)
        payload = bytes(rng.randint(0, 255) for _ in range(length))
        expected = crc16_ibm_ref(payload)

        # Soft-reset before each sequence
        await crc_soft_reset(dut)
        await feed_bytes(dut, payload)

        crc = await wb_read(dut, 0x04)
        got = crc & 0xFFFF
        if got != expected:
            dut._log.error(
                f"Seq {seq_idx}: payload={payload.hex()} "
                f"expected=0x{expected:04X} got=0x{got:04X}"
            )
            failures += 1

    assert failures == 0, f"{failures}/100 random sequences FAILED"
    dut._log.info("PASS: 100/100 random sequences match Python reference")
