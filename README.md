# Multichannel Signal Processing System — Final Handoff v4

## Elevator Pitch
A fully on-fabric FPGA air quality monitoring system that acquires data from three analog
gas/light sensors, processes it through dedicated hardware FIR filters and threshold
comparators, and classifies alarm states in real-time — all without a microprocessor.
Built on Zynq ZC702, verified across 54 tests.

---

## Resume Bullet
> **Multichannel Signal Processing System** — 3-channel analog air quality pipeline
> (MQ-3/MQ-135/LDR) through arbitrated SPI ADC; per-channel AXI-Lite FIR accelerators
> for noise rejection; on-fabric DSP core with leaky integrator baseline compensation
> and cross-channel event fusion for alarm classification without host dependency.
> Verified 54/54 tests across 4 phases. Target: Zynq ZC702 (xc7z020clg484-1).

---

## System Architecture

```
MQ-3 / MQ-135 / LDR  (Physical Sensors)
         |
  [spi_adc_master_v2]    12-bit hardware ADC, AD7606B SPI protocol,
                         16-bit zero-padded output, 1kHz sampling @ 100MHz
         |
  [channel_mux_fsm]      Round-robin CH0→CH1→CH2, CS_N arbitration,
                         deterministic per-channel tagging
         |
   ch0  ch1  ch2   [15:0]
         |
  [axil_master_ch] ×3    Per-channel AXI-Lite master (23-state FSM)
         |                Drives TWO slaves sequentially per sample:
         |                  1. axi_moving_avg  → 8-tap FIR noise filter
         |                  2. thresh_comparator → alarm detection
         |
  [dsp_core]             Cross-channel fusion engine
         |                - Leaky integrator baseline (no multipliers)
         |                - Dynamic threshold feedback
         |                - Alarm classification
         ↓
  alarm_level[1:0]       00=Clear  01=Warning  10=Danger  11=Critical
  alarm_ch[1:0]          Which channel triggered
  event_valid            Atomic 1-cycle pulse on state change
         |
  ch[0..2]_thresh_hi/lo ────────────────────► (fed back to axil_master_ch)
```

---

## Verification Results

| Phase | Module | Tests | Result |
|-------|--------|-------|--------|
| 1 | SPI Front-End | 19/19 | ✅ |
| 2 | Per-Channel AXI Masters | 12/12 | ✅ |
| 3 | DSP Core | 15/15 | ✅ |
| 4 | Top-Level Integration | 8/8 | ✅ |
| **Total** | | **54/54** | **✅** |

---

## Key Engineering Decisions

**Deadlock-free AXI mastering** — `valid` asserted first, `ready` checked next cycle.
Strictly separates address and data phases to prevent single-cycle combinatorial
deadlocks in the AXI-Lite handshake.

**Multiplier-free baseline tracking** — `baseline = baseline - (baseline >> 6) + sample`.
Pure shift-add leaky integrator, ~64-sample time constant. Zero DSP48 slices used.
Handles MQ sensor warm-up drift automatically.

**Atomic fusion trigger** — `ch_updated[2:0]` bitmask ensures fusion only evaluates
when all 3 channels have fresh data. Cleared in the same clock cycle as `event_valid`
to prevent double-fire. No stale data paths.

**Saturating dynamic thresholds** — `thresh_hi = baseline + offset` with overflow
protection. Fed back to all three AXI masters every sample cycle. System self-calibrates
to ambient conditions without host intervention.

**Cross-channel alarm fusion logic:**
```
ch0=MQ-3 (alcohol/VOC), ch1=MQ-135 (CO2/air quality), ch2=LDR (ambient light)

irq0 && irq1 && irq2  →  CRITICAL  (systemic multi-sensor event)
irq0 && irq1          →  DANGER    (gas confirmed by two sensors — high confidence)
irq1 only             →  WARNING   (MQ-135 air quality degradation)
irq0 only             →  WARNING   (MQ-3 alcohol/VOC spike)
irq2 only             →  WARNING   (LDR ambient light event)
none                  →  CLEAR
```

---

## File Structure

```
rtl/
  top.v                  Top-level structural wrapper + MMCM clock
  spi_adc_master_v2.v    Phase 1 — 7-state SPI FSM
  channel_mux_fsm.v      Phase 1 — Round-robin arbitrator
  axil_master_ch.v       Phase 2 — 23-state AXI-Lite master
  dsp_core.v             Phase 3 — Fusion engine + leaky integrator
external/
  axi_moving_avg.v       8-tap FIR, DATA_WIDTH=16, 20-bit accumulator
  thresh_comparator.v    Hysteresis comparator, latched IRQ, auto-rearm
tb/
  tb_spi_frontend.sv     19 tests — SPI timing, data integrity, CS exclusivity
  tb_axil_master_ch.sv   12 tests — AXI handshake, FIR output, dynamic thresholds
  tb_dsp_core.sv         15 tests — Fusion logic, baseline, alarm classification
  tb_top.sv              8 tests  — End-to-end pipeline
constraints/
  top.xdc                ZC702 pin assignments — PMOD JA (SPI), PMOD JB (alarms)
```

---

## Resource Estimate (Artix-7 equivalent on xc7z020)

From Phase 2 synthesis reference (accelerators only):
| Resource | Phase 2 (2 accel) | Full system estimate |
|----------|-------------------|----------------------|
| Slice LUTs | 162 / 134,600 (0.12%) | ~1,200 (<1%) |
| Slice FFs | 339 / 269,200 (0.13%) | ~2,500 (<1%) |
| DSP48s | 0 | 0 |
| BRAMs | 0 | 0 |
| Fmax | >100MHz (4.38ns path) | >100MHz expected |

Zero DSP48 usage is a deliberate design point — the entire signal processing
chain uses only LUTs and FFs.

---

## Phase 5 — Physical Implementation (Pending Hardware)

**Hardware required:**
- Zynq ZC702 board (from lab)
- MQ-3 sensor module (alcohol/VOC)
- MQ-135 sensor module (CO2/air quality)
- LDR + voltage divider board
- External SPI ADC (MCP3204 or similar — ZC702 PL has no onboard ADC)
- Breadboard, jumpers, 3.3V logic

**Vivado steps before hardware arrives:**
1. Add `clk_wiz_0` IP (200MHz differential → 100MHz) to project
2. Run synthesis — verify LUT/FF utilization and timing closure at 100MHz
3. Run implementation — check routing and WNS > 0
4. Generate bitstream — ready to flash on arrival

**Bring-up plan:**
1. Flash bitstream via JTAG
2. Verify MMCM lock (LED indicator)
3. Probe PMOD JA with oscilloscope — confirm SCLK at 12.5MHz, CS_N toggling
4. Connect ADC + sensor board
5. Monitor alarm_level on PMOD JB LEDs
6. Expose alcohol near MQ-3 — expect WARNING → DANGER escalation

---

## Simulation Environment
Icarus Verilog (`iverilog -g2012`) + GTKWave, WSL Ubuntu

```bash
# Run all phases
iverilog -g2012 -o p1.out rtl/spi_adc_master_v2.v rtl/channel_mux_fsm.v tb/tb_spi_frontend.sv && vvp p1.out
iverilog -g2012 -o p2.out external/axi_moving_avg.v external/thresh_comparator.v rtl/axil_master_ch.v tb/tb_axil_master_ch.sv && vvp p2.out
iverilog -g2012 -o p3.out rtl/dsp_core.v tb/tb_dsp_core.sv && vvp p3.out
iverilog -g2012 -o p4.out external/axi_moving_avg.v external/thresh_comparator.v rtl/spi_adc_master_v2.v rtl/channel_mux_fsm.v rtl/axil_master_ch.v rtl/dsp_core.v rtl/top.v tb/tb_top.sv && vvp p4.out
```

---

## Target Hardware
Zynq ZC702 (xc7z020clg484-1), Vivado 2025.1
GitHub: github.com/Jehka (opticalFPGADataAcquistion, fpgaAxiAccelerator)