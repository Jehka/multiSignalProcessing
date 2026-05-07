# =============================================================================
# top.xdc — ZC702 Constraints
# PMOD JA (J58): SPI bus
# PMOD JB (J59): Alarm outputs
# =============================================================================

# -----------------------------------------------------------------------------
# Clock — 200MHz differential input, MMCM to 100MHz in top.v
# -----------------------------------------------------------------------------
set_property PACKAGE_PIN H9  [get_ports clk]
set_property IOSTANDARD LVDS [get_ports clk]
create_clock -period 5.000 -name sys_clk [get_ports clk]

# If using single-ended 200MHz (U18/GCLK):
# set_property PACKAGE_PIN Y9  [get_ports clk]
# set_property IOSTANDARD LVCMOS18 [get_ports clk]
# create_clock -period 5.000 -name sys_clk [get_ports clk]

# -----------------------------------------------------------------------------
# Reset — CPU_RESET pushbutton (active-high on board, invert in RTL or here)
# -----------------------------------------------------------------------------
set_property PACKAGE_PIN AB7  [get_ports rst_n]
set_property IOSTANDARD LVCMOS15 [get_ports rst_n]

# -----------------------------------------------------------------------------
# PMOD JA (J58) — SPI Bus
# Pin 1=JA1, 2=JA2, 3=JA3, 4=JA4, 7=JA7, 8=JA8, 9=JA9, 10=JA10
# -----------------------------------------------------------------------------
set_property PACKAGE_PIN Y11  [get_ports sclk]
set_property PACKAGE_PIN AA11 [get_ports convst_n]
set_property PACKAGE_PIN Y10  [get_ports miso]
set_property PACKAGE_PIN AA9  [get_ports busy]
set_property PACKAGE_PIN AB11 [get_ports {cs_n[0]}]
set_property PACKAGE_PIN AB10 [get_ports {cs_n[1]}]
set_property PACKAGE_PIN AB9  [get_ports {cs_n[2]}]

set_property IOSTANDARD LVCMOS18 [get_ports sclk]
set_property IOSTANDARD LVCMOS18 [get_ports convst_n]
set_property IOSTANDARD LVCMOS18 [get_ports miso]
set_property IOSTANDARD LVCMOS18 [get_ports busy]
set_property IOSTANDARD LVCMOS18 [get_ports {cs_n[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {cs_n[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {cs_n[2]}]

# Output drive strength for SPI
set_property DRIVE 8  [get_ports sclk]
set_property DRIVE 8  [get_ports convst_n]
set_property DRIVE 8  [get_ports {cs_n[*]}]
set_property SLEW FAST [get_ports sclk]
set_property SLEW FAST [get_ports convst_n]
set_property SLEW FAST [get_ports {cs_n[*]}]

# -----------------------------------------------------------------------------
# PMOD JB (J59) — Alarm Outputs
# -----------------------------------------------------------------------------
set_property PACKAGE_PIN W12  [get_ports {alarm_level[0]}]
set_property PACKAGE_PIN W11  [get_ports {alarm_level[1]}]
set_property PACKAGE_PIN V10  [get_ports {alarm_ch[0]}]
set_property PACKAGE_PIN W10  [get_ports {alarm_ch[1]}]
set_property PACKAGE_PIN V9   [get_ports event_valid]

set_property IOSTANDARD LVCMOS18 [get_ports {alarm_level[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {alarm_ch[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports event_valid]

# -----------------------------------------------------------------------------
# Timing constraints
# -----------------------------------------------------------------------------
# SPI MISO input — max SCLK=12.5MHz (80ns period), generous setup/hold
set_input_delay  -clock sys_clk -max 10.0 [get_ports miso]
set_input_delay  -clock sys_clk -min  2.0 [get_ports miso]
set_input_delay  -clock sys_clk -max  5.0 [get_ports busy]
set_input_delay  -clock sys_clk -min  1.0 [get_ports busy]

set_output_delay -clock sys_clk -max  5.0 [get_ports sclk]
set_output_delay -clock sys_clk -max  5.0 [get_ports convst_n]
set_output_delay -clock sys_clk -max  5.0 [get_ports {cs_n[*]}]

# Alarm outputs are slow control signals — relax timing
set_output_delay -clock sys_clk -max 10.0 [get_ports {alarm_level[*]}]
set_output_delay -clock sys_clk -max 10.0 [get_ports {alarm_ch[*]}]
set_output_delay -clock sys_clk -max 10.0 [get_ports event_valid]

# -----------------------------------------------------------------------------
# Bitstream config
# -----------------------------------------------------------------------------
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLDOWN [current_design]
