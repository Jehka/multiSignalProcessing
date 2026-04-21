`timescale 1ns/1ps
// =============================================================================
// spi_adc_master_v2.v
// 3-Channel SPI ADC Master — AD7606B-style protocol (12-bit hardware, 16-bit output)
//
// Protocol summary (per channel):
//   1. Assert CONVST (1 cycle pulse) to trigger conversion
//   2. Wait BUSY_CYCLES for ADC busy period (~4 µs at 100 MHz = 400 cycles)
//   3. Assert CS_N, clock out 12 bits MSB-first on falling SCLK edge
//   4. Deassert CS_N, assert sample_valid with 16-bit zero-padded result
//
// Parameterisation:
//   SYS_CLK_HZ  system clock frequency (default 100 MHz)
//   SCLK_DIV    SCLK = SYS_CLK / (2 * SCLK_DIV)  (default 4 → 12.5 MHz)
//   BUSY_CYCLES ADC conversion time in sys clk cycles (default 400 → 4 µs)
//
// Outputs:
//   sample_data [15:0]  16-bit zero-padded ADC result
//   sample_ch   [1:0]   channel tag (0/1/2)
//   sample_valid        1-cycle pulse when data is valid
// =============================================================================

module spi_adc_master_v2 #(
    parameter SYS_CLK_HZ  = 100_000_000,
    parameter SCLK_DIV    = 4,           // SCLK = 100MHz / (2*4) = 12.5 MHz
    parameter BUSY_CYCLES = 400          // 4 µs @ 100 MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // Trigger from channel_mux_fsm
    input  wire        start,            // pulse: begin conversion on ch_sel
    input  wire [1:0]  ch_sel,           // which channel to convert

    // SPI physical signals (shared bus, CS decoded externally)
    output reg         sclk,
    output reg         convst_n,         // active-low conversion start
    input  wire        miso,
    input  wire        busy,             // ADC busy flag (active-high)

    // Result output (Padded to 16 bits)
    output reg  [15:0] sample_data,
    output reg  [1:0]  sample_ch,
    output reg         sample_valid,

    // Handshake back to channel_mux_fsm
    output reg         done              // pulse when transaction complete
);

    // =========================================================================
    // State Encoding
    // =========================================================================
    localparam ST_IDLE      = 3'd0;  // waiting for start
    localparam ST_CONVST    = 3'd1;  // assert CONVST pulse (1 cycle)
    localparam ST_WAIT_BUSY = 3'd2;  // wait for BUSY to assert then deassert
    localparam ST_CS_SETUP  = 3'd3;  // CS setup hold (2 cycles)
    localparam ST_SHIFT     = 3'd4;  // clock out 12 bits
    localparam ST_CS_HOLD   = 3'd5;  // CS deassert hold
    localparam ST_DONE      = 3'd6;  // pulse done, valid

    reg [2:0]  state;

    // =========================================================================
    // Internal Signals
    // =========================================================================
    reg [11:0] shift_reg;            // incoming serial data (physically 12 bits)
    reg [3:0]  bit_cnt;              // counts 0..11 (12 bits)
    reg [9:0]  wait_cnt;             // busy/setup wait counter
    reg [2:0]  sclk_cnt;             // SCLK divider counter
    reg        sclk_en;              // SCLK gating
    reg        sclk_fall;            // 1-cycle flag: SCLK just fell
    reg        sclk_rise;            // 1-cycle flag: SCLK just rose
    reg [1:0]  ch_lat;               // latched channel for this transaction
    reg        busy_seen;            // BUSY was asserted at least once

    // =========================================================================
    // SCLK Generator
    // Divides clk by 2*SCLK_DIV. sclk_fall/sclk_rise are single-cycle strobes.
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            sclk      <= 1'b1;       // idle high (CPOL=1 for AD7606B-like)
            sclk_cnt  <= 3'b0;
            sclk_fall <= 1'b0;
            sclk_rise <= 1'b0;
        end else begin
            sclk_fall <= 1'b0;
            sclk_rise <= 1'b0;
            if (sclk_en) begin
                if (sclk_cnt == SCLK_DIV - 1) begin
                    sclk_cnt <= 3'b0;
                    sclk     <= ~sclk;
                    if (sclk)  sclk_fall <= 1'b1;   // was high, going low
                    else       sclk_rise <= 1'b1;   // was low, going high
                end else begin
                    sclk_cnt <= sclk_cnt + 1;
                end
            end else begin
                sclk     <= 1'b1;    // idle high when disabled
                sclk_cnt <= 3'b0;
            end
        end
    end

    // =========================================================================
    // Main State Machine
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            convst_n     <= 1'b1;
            sclk_en      <= 1'b0;
            shift_reg    <= 12'b0;
            sample_data  <= 16'b0;
            sample_ch    <= 2'b0;
            sample_valid <= 1'b0;
            done         <= 1'b0;
            bit_cnt      <= 4'b0;
            wait_cnt     <= 10'b0;
            ch_lat       <= 2'b0;
            busy_seen    <= 1'b0;
        end else begin
            // Default single-cycle pulses
            sample_valid <= 1'b0;
            done         <= 1'b0;
            convst_n     <= 1'b1;

            case (state)

                // -----------------------------------------------------------------
                ST_IDLE: begin
                    sclk_en  <= 1'b0;
                    bit_cnt  <= 4'd0;
                    wait_cnt <= 10'd0;
                    if (start) begin
                        ch_lat    <= ch_sel;
                        busy_seen <= 1'b0;
                        state     <= ST_CONVST;
                    end
                end

                // -----------------------------------------------------------------
                // Assert CONVST_N for 1 cycle to trigger ADC conversion
                // -----------------------------------------------------------------
                ST_CONVST: begin
                    convst_n <= 1'b0;     // active-low pulse
                    state    <= ST_WAIT_BUSY;
                end

                // -----------------------------------------------------------------
                // Wait: BUSY asserts within a few cycles, then deasserts when
                // conversion is complete. We wait for the full busy cycle.
                // Fallback: if BUSY never asserts, time out after BUSY_CYCLES.
                // -----------------------------------------------------------------
                ST_WAIT_BUSY: begin
                    if (busy)
                        busy_seen <= 1'b1;

                    // Proceed once busy has been seen AND has deasserted,
                    // OR after timeout (handles sim models without BUSY)
                    if ((busy_seen && !busy) ||
                        (wait_cnt == BUSY_CYCLES - 1)) begin
                        wait_cnt <= 10'd0;
                        state    <= ST_CS_SETUP;
                    end else begin
                        wait_cnt <= wait_cnt + 1;
                    end
                end

                // -----------------------------------------------------------------
                // CS setup: 2 system clock cycles before first SCLK edge
                // CS_N is driven by channel_mux_fsm based on ch_lat — we just
                // enable SCLK here after the setup window
                // -----------------------------------------------------------------
                ST_CS_SETUP: begin
                    if (wait_cnt == 10'd2) begin
                        wait_cnt <= 10'd0;
                        sclk_en  <= 1'b1;
                        state    <= ST_SHIFT;
                    end else begin
                        wait_cnt <= wait_cnt + 1;
                    end
                end

                // -----------------------------------------------------------------
                // Shift in 12 bits, MSB first, sample on rising SCLK edge
                // (data changes on falling edge, stable on rising — CPHA=1)
                // -----------------------------------------------------------------
                ST_SHIFT: begin
                    if (sclk_rise) begin
                        shift_reg <= {shift_reg[10:0], miso};
                        if (bit_cnt == 4'd11) begin
                            sclk_en <= 1'b0;
                            state   <= ST_CS_HOLD;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // CS hold: 2 cycles after last SCLK before CS deassert
                // -----------------------------------------------------------------
                ST_CS_HOLD: begin
                    if (wait_cnt == 10'd2) begin
                        wait_cnt <= 10'd0;
                        state    <= ST_DONE;
                    end else begin
                        wait_cnt <= wait_cnt + 1;
                    end
                end

                // -----------------------------------------------------------------
                // Latch padded result, pulse valid and done
                // -----------------------------------------------------------------
                ST_DONE: begin
                    sample_data  <= {4'b0000, shift_reg}; // Zero pad hardware bits to 16
                    sample_ch    <= ch_lat;
                    sample_valid <= 1'b1;
                    done         <= 1'b1;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule