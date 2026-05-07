`timescale 1ns/1ps
// =============================================================================
// uart_framer.v
// Builds 9-byte frames from DSP core outputs and queues them to packet_fifo.
// Triggers on:
//   1. event_valid pulse (immediate alarm event)
//   2. 10Hz tick (continuous background stream)
//
// Frame format:
//   Byte 0: 0xAA          start marker
//   Byte 1: {4'b0, alarm_ch[1:0], alarm_level[1:0]}
//   Byte 2: ch0_fir[15:8]
//   Byte 3: ch0_fir[7:0]
//   Byte 4: ch1_fir[15:8]
//   Byte 5: ch1_fir[7:0]
//   Byte 6: ch2_fir[15:8]
//   Byte 7: ch2_fir[7:0]
//   Byte 8: 0x55          end marker
//
// Downstream: uart_controller reads packet_fifo and sends via uart_tx.
// uart_controller must be adapted to handle 72-bit wide FIFO entries
// (serialize 9 bytes sequentially using the existing byte-at-a-time tx).
// =============================================================================

module uart_framer #(
    parameter CLK_FREQ_HZ  = 100_000_000,
    parameter STREAM_RATE  = 10          // Hz background stream rate
)(
    input  wire        clk,
    input  wire        rst_n,

    // From dsp_core
    input  wire [1:0]  alarm_level,
    input  wire [1:0]  alarm_ch,
    input  wire        event_valid,
    input  wire [15:0] ch0_fir_result,
    input  wire [15:0] ch1_fir_result,
    input  wire [15:0] ch2_fir_result,

    // To packet_fifo
    output reg  [71:0] fifo_wr_data,
    output reg         fifo_wr_en,
    input  wire        fifo_full
);

    localparam TICK_COUNT = CLK_FREQ_HZ / STREAM_RATE;  // 10,000,000 cycles

    // =========================================================================
    // 10Hz tick generator
    // =========================================================================
    reg [26:0] tick_cnt;
    reg        tick;

    always @(posedge clk) begin
        if (!rst_n) begin
            tick_cnt <= 0;
            tick     <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (tick_cnt == TICK_COUNT - 1) begin
                tick_cnt <= 0;
                tick     <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 1;
            end
        end
    end

    // =========================================================================
    // Frame builder
    // Fires on event_valid OR 10Hz tick (whichever comes first)
    // Latches inputs at trigger time — no glitches mid-frame
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_wr_en   <= 1'b0;
            fifo_wr_data <= 72'b0;
        end else begin
            fifo_wr_en <= 1'b0;

            if ((event_valid || tick) && !fifo_full) begin
                fifo_wr_data <= {
                    8'hAA,                                    // [71:64] start
                    4'b0000, alarm_ch, alarm_level,           // [63:56] status
                    ch0_fir_result,                           // [55:40] ch0
                    ch1_fir_result,                           // [39:24] ch1
                    ch2_fir_result,                           // [23:8]  ch2
                    8'h55                                     // [7:0]   end
                };
                fifo_wr_en <= 1'b1;
            end
        end
    end

endmodule