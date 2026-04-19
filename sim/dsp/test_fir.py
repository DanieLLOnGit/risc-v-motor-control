import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles
import numpy as np
from scipy.signal import lfilter

# Coefficient reference (Q1.15 integer values)
COEFFS_Q15 = [
    0x009C, 0x00AE, 0x00E4, 0x013C,
    0x01B3, 0x0242, 0x02E5, 0x0394,
    0x0449, 0x04FD, 0x05A7, 0x0641,
    0x06C4, 0x072C, 0x0773, 0x0798,
    0x0798, 0x0773, 0x072C, 0x06C4,
    0x0641, 0x05A7, 0x04FD, 0x0449,
    0x0394, 0x02E5, 0x0242, 0x01B3,
    0x013C, 0x00E4, 0x00AE, 0x009C,
]
COEFFS_FLOAT = [c / 32768.0 for c in COEFFS_Q15]

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

async def push_sample_wb(dut, sample_q15):
    """Write one signed Q1.15 sample via Wishbone DATA_IN (offset 0x00)."""
    await wb_write(dut, 0x00, sample_q15 & 0xFFFF)

async def wait_output_valid(dut, timeout=100):
    """Poll STATUS register (0x08) bit[0] until output_valid=1."""
    for _ in range(timeout):
        status = await wb_read(dut, 0x08)
        if status & 0x1:
            return
        await ClockCycles(dut.clk_i, 1)
    raise TimeoutError("output_valid never asserted")

async def read_output(dut):
    """Read DATA_OUT (0x04) and sign-extend lower 16 bits."""
    val = await wb_read(dut, 0x04)
    raw = val & 0xFFFF
    if raw >= 0x8000:
        raw -= 0x10000
    return raw

# Reset helper
async def reset_dut(dut):
    dut.rst_ni.value = 0
    dut.cyc_i.value = 0
    dut.stb_i.value = 0
    dut.we_i.value = 0
    dut.adr_i.value = 0
    dut.dat_i.value = 0
    dut.sel_i.value = 0
    dut.s_valid_i.value = 0
    dut.s_data_i.value = 0
    dut.m_ready_i.value = 1
    await ClockCycles(dut.clk_i, 4)
    dut.rst_ni.value = 1
    await ClockCycles(dut.clk_i, 2)

# Test 1: Impulse response — first 32 outputs must match coefficients
@cocotb.test()
async def test_impulse_response(dut):
    """Impulse response: h[n] must equal COEFF_INIT[n] for n=0..31."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    # Full-scale impulse at n=0 (Q1.15 = 0x7FFF ≈ +1.0)
    IMPULSE = 0x7FFF
    samples = [IMPULSE] + [0] * 63   # 64 samples

    outputs = []
    for s in samples:
        await push_sample_wb(dut, s)
        await wait_output_valid(dut)
        out = await read_output(dut)
        outputs.append(out)

    # Golden: lfilter with float coefficients applied to integer input
    stimulus_f = np.array([IMPULSE] + [0] * 63, dtype=float)
    golden_f = lfilter(COEFFS_FLOAT, [1.0], stimulus_f)

    errors = 0
    for i in range(32):
        gold_q15 = int(round(golden_f[i]))
        # allow ±2 LSB for rounding
        if abs(outputs[i] - gold_q15) > 2:
            errors += 1
            dut._log.error(f"Impulse n={i}: got {outputs[i]}, expected {gold_q15}")

    assert errors == 0, f"{errors} impulse response errors"
    dut._log.info("PASS: impulse response matches golden model")


# Test 2: Step response — compare hardware vs scipy golden
@cocotb.test()
async def test_step_response(dut):
    """Step response convergence — must match scipy lfilter within ±2 LSB."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    STEP_VAL = 0x4000   # +0.5 in Q1.15
    N = 64
    stimulus = [STEP_VAL] * N

    outputs = []
    for s in stimulus:
        await push_sample_wb(dut, s)
        await wait_output_valid(dut)
        out = await read_output(dut)
        outputs.append(out)

    golden_f = lfilter(COEFFS_FLOAT, [1.0], np.array(stimulus, dtype=float))

    errors = 0
    for i in range(N):
        gold_q15 = int(round(golden_f[i]))
        if abs(outputs[i] - gold_q15) > 2:
            errors += 1
            dut._log.error(f"Step n={i}: got {outputs[i]}, expected {gold_q15}")

    assert errors == 0, f"{errors} step response errors"
    dut._log.info("PASS: step response matches golden model")


# Test 3: SNR — lowpass input, measure stopband attenuation
@cocotb.test()
async def test_snr(dut):
    """
    SNR test: inject white_noise + 300Hz sinewave.
    At Fc=1kHz, 300Hz is in passband.  Stopband noise should be
    attenuated sufficiently that SNR > 40 dB.
    """
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    rng = np.random.default_rng(42)
    N = 256
    FS = 50e6
    t = np.arange(N) / FS
    tone = 0.5 * np.sin(2 * np.pi * 300 * t)        # 300 Hz tone, amplitude 0.5
    noise = 0.1 * rng.standard_normal(N)              # white noise
    stim_f = tone + noise

    # Quantise to Q1.15
    stim_q15 = np.clip(np.round(stim_f * 32768).astype(int), -32768, 32767)

    outputs = []
    for s in stim_q15:
        await push_sample_wb(dut, int(s) & 0xFFFF)
        await wait_output_valid(dut)
        out = await read_output(dut)
        outputs.append(out)

    golden_f = lfilter(COEFFS_FLOAT, [1.0], stim_q15.astype(float))

    # Compute error signal and SNR
    out_arr = np.array(outputs, dtype=float)
    gold_arr = np.array([int(round(g)) for g in golden_f], dtype=float)
    err = out_arr - gold_arr

    signal_power = np.mean(gold_arr[32:] ** 2)
    error_power = np.mean(err[32:] ** 2) + 1e-12   # avoid log(0)
    snr_db = 10 * np.log10(signal_power / error_power)

    dut._log.info(f"SNR = {snr_db:.1f} dB (threshold 40 dB)")
    assert snr_db > 40.0, f"SNR too low: {snr_db:.1f} dB"
    dut._log.info("PASS: SNR > 40 dB")


# Test 4: Coefficient writeback — overwrite one coeff, verify read-back
@cocotb.test()
async def test_coeff_writeback(dut):
    """Write a new coefficient via Wishbone, read it back, verify."""
    cocotb.start_soon(Clock(dut.clk_i, 20, units="ns").start())
    await reset_dut(dut)

    TARGET_ADDR = 5
    TARGET_VALUE = 0x1234

    # Write coeff_addr register (0x10)
    await wb_write(dut, 0x10, TARGET_ADDR)
    # Write coeff_data register (0x14)
    await wb_write(dut, 0x14, TARGET_VALUE)
    # Trigger coeff write (0x18)
    await wb_write(dut, 0x18, 1)

    # Read back: set addr then read 0x14
    await wb_write(dut, 0x10, TARGET_ADDR)
    readback = await wb_read(dut, 0x14)
    readback_coeff = readback & 0xFFFF

    assert readback_coeff == TARGET_VALUE, \
        f"Coeff readback mismatch: got 0x{readback_coeff:04X}, expected 0x{TARGET_VALUE:04X}"
    dut._log.info("PASS: coefficient writeback verified")
