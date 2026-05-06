# RISC-V Motor Control

Standalone RISC-V SoC for real-time Dynamixel motor control — no host processor required.
The RV32I CPU runs the full position read → FIR filter → PID loop onboard, targeting ASIC fabrication on SkyWater 130nm via TinyTapeout/OpenLane.

---

## Target Hardware

### [Dynamixel XC430-T150BBT](https://www.robotis.us/dynamixel-xc430-t150bb-t/)
The servo communicates over a single TTL half-duplex wire using Dynamixel Protocol 2.0 — the SoC sends a command packet, tri-states the line, then reads back the status response on the same pin. This bus behavior is implemented entirely in `uart_dynamixel.sv`, with CRC-16/IBM packet validation handled by `crc16_ibm.sv`. The servo's 12-bit encoder (4096 counts/rev) is the position feedback source; those raw values are piped through the on-chip FIR filter before entering the PID loop in firmware.

### [SkyWater 130nm (Sky130B)](https://github.com/google/skywater-pdk)
The SoC RTL targets the Sky130B PDK at 50 MHz. The OpenLane flow (`syn/config.json`) drives synthesis through floorplan, placement, and routing to produce a tapeout-ready GDSII. TinyTapeout is the submission vehicle — it aggregates small open-source designs onto a shared reticle, making per-design silicon cost tractable.

### Signal Chain
```
Dynamixel servo
  └─ TTL half-duplex response packet
       └─ uart_dynamixel FSM — strips header, validates CRC-16
            └─ raw 12-bit position written to DMEM
                 └─ FIR filter — 32-tap Hamming window, Q1.15 fixed-point
                      └─ filtered position → PID loop (main.c)
                           └─ goal position command packet → uart_dynamixel TX
                                └─ Dynamixel servo
```

---

## Folder Layout

```
risc-v-motor-control/
│
├── rtl/                        # Synthesizable RTL (SystemVerilog)
│   ├── cpu/
│   │   └── picorv32.v          # PicoRV32 RV32I core (YosysHQ)
│   ├── dsp/
│   │   └── fir_filter.sv       # 32-tap Hamming-windowed FIR, Q1.15 fixed-point
│   ├── hpc/
│   │   ├── crc16_ibm.sv        # CRC-16/IBM (poly 0x8005), Wishbone slave
│   │   └── uart_dynamixel.sv   # Half-duplex UART FSM, Dynamixel Protocol 2.0
│   └── soc/
│       ├── hpc_regfile.sv      # Wishbone mux: CRC @ 0x0000, UART @ 0x0080
│       └── soc.sv              # Top-level: CPU + IMEM + DMEM + DSP + HPC
│
├── firmware/                   # Bare-metal C firmware (RV32I)
│   ├── crt0.s                  # Assembly startup: set sp, call main, trap loop
│   ├── link.ld                 # Linker script: .text @ 0x0000_0000, .data @ 0x0001_0000
│   ├── main.c                  # Servo read → FIR filter → PID control loop
│   └── Makefile                # riscv-gcc → firmware.elf → firmware.hex
│
├── sim/                        # Cocotb + Verilator testbenches
│   ├── dsp/
│   │   ├── Makefile            # Verilator sim for fir_filter
│   │   └── test_fir.py         # 4 tests: impulse, step, SNR, coeff writeback
│   ├── hpc/
│   │   ├── Makefile            # Verilator sim for crc16_ibm
│   │   ├── Makefile.uart       # Verilator sim for uart_dynamixel + hpc_regfile
│   │   ├── test_crc.py         # CRC known-vector + incremental tests
│   │   └── test_uart.py        # 3 tests: TX packet, RX parse, bad CRC
│   └── soc/
│       ├── Makefile            # Verilator sim for full SoC
│       └── test_soc.py         # 3 integration tests: boot, RX loop, reset recovery
│
├── data/
│   └── fir_coeffs.hex          # 32 Q1.15 FIR coefficients (loaded into fir_filter at reset)
│
├── syn/                        # Synthesis + place-and-route
│   ├── synth.tcl               # Yosys synthesis script (standalone, no Docker required)
│   ├── constraints.sdc         # SDC timing constraints: 50 MHz clock, I/O delays
│   ├── config.json             # OpenLane 2 design config: sources, clock, die area
│   ├── Makefile                # Run standalone Yosys: make clean && make
│   └── results/                # Generated outputs (netlist, area report, log)
│
└── docs/                       # Architecture diagrams, timing reports (coming soon)
```

---

## Memory Map

| Address Range          | Size | Description             |
|------------------------|------|-------------------------|
| `0x0000_0000–0x0FFF`  | 4 KB | IMEM — instruction SRAM |
| `0x0001_0000–0x1FFF`  | 4 KB | DMEM — data/stack SRAM  |
| `0x0002_0000–0x2FFF`  | —    | DSP Engine (fir_filter) |
| `0x0003_0000–0x3FFF`  | —    | HPC Engine (CRC + UART) |

---

## Required Software

### Simulation
| Tool | Purpose |
|---|---|
| [Verilator](https://verilator.org) | Compiles SystemVerilog RTL into a C++ simulation model for fast cycle-accurate behavioural verification |
| [Yosys](https://yosyshq.net/yosys/) | Open-source RTL synthesis — converts synthesizable SV to a gate-level netlist and reports area/timing estimates before OpenLane |
| [Cocotb](https://www.cocotb.org) | Python-based testbench framework that drives Verilator simulations; used for the DSP and HPC unit tests |
| [Python 3.10+](https://python.org) | Required runtime for Cocotb testbenches and the FIR golden-model coefficient generation scripts |

### Firmware
| Tool | Purpose |
|---|---|
| RISC-V GCC toolchain | Cross-compiles C firmware (PID loop, Dynamixel protocol driver) to RV32I ELF binaries that are loaded into the SoC instruction SRAM |

### Physical Design
| Tool | Purpose |
|---|---|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) | Provides the containerised environment that OpenLane runs inside, isolating all EDA tool dependencies from the host system |
| [OpenLane](https://github.com/efabless/openlane) | Automated RTL-to-GDSII flow (synthesis → floorplan → placement → routing → signoff) targeting the Sky130 PDK |
| [KLayout](https://www.klayout.de) | GDSII viewer and DRC runner — used to inspect final layout and run SkyWater design-rule checks before tapeout |
| [Magic VLSI](https://github.com/RTimothyEdwards/magic) | Parasitic extraction (RC) and LVS verification against the post-layout netlist |
| [Sky130 PDK](https://github.com/RTimothyEdwards/open_pdks) | SkyWater 130nm process design kit — standard cell libraries, DRC/LVS rules, and SPICE models required by every tool in the flow |

---

## Quick Start

```bash
# Firmware
cd firmware && make               # → firmware.hex (loaded into IMEM at sim time)

# Simulations (each dir has its own Makefile)
cd sim/dsp && make                # FIR filter — 4 tests
cd sim/hpc && make                # CRC engine
cd sim/hpc && make -f Makefile.uart  # UART + HPC regfile — 3 tests
cd sim/soc && make                # Full SoC integration — 3 tests

# Synthesis (Yosys standalone, no Docker)
cd syn && make clean && make      # → results/soc_netlist.v, area.rpt, synth.log
```

