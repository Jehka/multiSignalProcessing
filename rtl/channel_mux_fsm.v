`timescale 1ns/1ps
// =============================================================================
// channel_mux_fsm.v
// Round-Robin Channel Arbitrator for 3-Channel SPI ADC Pipeline
//
// Responsibilities:
//   1. Sequences channels CH0 → CH1 → CH2 → CH0 continuously
//   2. Asserts the correct CS_N line per channel during SPI transactions
//   3. Drives start/ch_sel into spi_adc_master_v2
//   4. Forwards tagged sample_data + sample_valid to downstream pipeline
//   5. Exposes per-channel sample outputs for the AXI master layer
//
// Timing:
//   One full acquisition cycle = CONVST + BUSY_WAIT + CS_SETUP + 16 SCLK
//   cycles + CS_HOLD + DONE. The FSM idles between done and the next start
//   for one pipeline cycle to allow registered outputs to settle.
//
// CS Routing:
//   cs_n[0] → Channel 0 ADC
//   cs_n[1] → Channel 1 ADC
//   cs_n[2] → Channel 2 ADC
//   All three deasserted (high) except during the active channel's shift phase.
// =============================================================================

module channel_mux_fsm (
    input  wire        clk,
    input  wire        rst_n,

    // Enable — hold low to pause acquisition
    input  wire        enable,

    // To spi_adc_master_v2
    output reg         spi_start,
    output reg  [1:0]  spi_ch_sel,

    // From spi_adc_master_v2
    input  wire [15:0] spi_sample_data,
    input  wire [1:0]  spi_sample_ch,
    input  wire        spi_sample_valid,
    input  wire        spi_done,

    // CS_N outputs (active low, one per ADC chip select)
    output reg  [2:0]  cs_n,

    // Per-channel sample outputs (registered on sample_valid)
    output reg  [15:0] ch0_data,
    output reg         ch0_valid,
    output reg  [15:0] ch1_data,
    output reg         ch1_valid,
    output reg  [15:0] ch2_data,
    output reg         ch2_valid,

    // Current channel indicator (for debug / status)
    output reg  [1:0]  current_ch
);

    // =========================================================================
    // State Encoding
    // =========================================================================
    localparam ST_IDLE      = 2'd0;  // wait, then issue start
    localparam ST_ACTIVE    = 2'd1;  // spi_adc_master_v2 running
    localparam ST_LATCH     = 2'd2;  // latch result, deassert CS
    localparam ST_ADVANCE   = 2'd3;  // advance channel, back to IDLE

    reg [1:0] state;
    reg [1:0] ch_ptr;               // current channel pointer 0/1/2

    // =========================================================================
    // CS_N Decode
    // CS is asserted (low) during ST_ACTIVE so the SPI master can clock data.
    // Deasserted in all other states.
    // =========================================================================
    always @(*) begin
        cs_n = 3'b111;              // default: all deasserted
        if (state == ST_ACTIVE) begin
            case (ch_ptr)
                2'd0: cs_n = 3'b110;
                2'd1: cs_n = 3'b101;
                2'd2: cs_n = 3'b011;
                default: cs_n = 3'b111;
            endcase
        end
    end

    // =========================================================================
    // Round-Robin State Machine
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            ch_ptr     <= 2'd0;
            spi_start  <= 1'b0;
            spi_ch_sel <= 2'd0;
            ch0_data   <= 16'b0;
            ch0_valid  <= 1'b0;
            ch1_data   <= 16'b0;
            ch1_valid  <= 1'b0;
            ch2_data   <= 16'b0;
            ch2_valid  <= 1'b0;
            current_ch <= 2'd0;
        end else begin
            // Default: clear single-cycle pulses
            spi_start  <= 1'b0;
            ch0_valid  <= 1'b0;
            ch1_valid  <= 1'b0;
            ch2_valid  <= 1'b0;

            case (state)

                // -----------------------------------------------------------------
                // ST_IDLE: Issue start for current channel if enabled
                // -----------------------------------------------------------------
                ST_IDLE: begin
                    if (enable) begin
                        spi_ch_sel <= ch_ptr;
                        spi_start  <= 1'b1;
                        current_ch <= ch_ptr;
                        state      <= ST_ACTIVE;
                    end
                end

                // -----------------------------------------------------------------
                // ST_ACTIVE: Wait for SPI master to finish
                // CS_N is asserted combinationally above during this state
                // -----------------------------------------------------------------
                ST_ACTIVE: begin
                    if (spi_done)
                        state <= ST_LATCH;
                end

                // -----------------------------------------------------------------
                // ST_LATCH: Register result to the correct channel output
                // -----------------------------------------------------------------
                ST_LATCH: begin
                    case (spi_sample_ch)
                        2'd0: begin ch0_data <= spi_sample_data; ch0_valid <= 1'b1; end
                        2'd1: begin ch1_data <= spi_sample_data; ch1_valid <= 1'b1; end
                        2'd2: begin ch2_data <= spi_sample_data; ch2_valid <= 1'b1; end
                        default: ;
                    endcase
                    state <= ST_ADVANCE;
                end

                // -----------------------------------------------------------------
                // ST_ADVANCE: Move to next channel (0→1→2→0), back to IDLE
                // -----------------------------------------------------------------
                ST_ADVANCE: begin
                    ch_ptr <= (ch_ptr == 2'd2) ? 2'd0 : ch_ptr + 1;
                    state  <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule