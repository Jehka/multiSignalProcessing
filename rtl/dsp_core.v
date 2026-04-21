`timescale 1ns/1ps
// =============================================================================
// dsp_core.v — On-Fabric DSP Core (Hysteresis & Pipeline Patched)
// =============================================================================

module dsp_core #(
    parameter DATA_WIDTH       = 16,
    parameter LEAK_SHIFT       = 6,
    parameter THRESH_HI_OFFSET = 16'h0800,
    parameter THRESH_LO_OFFSET = 16'h0400
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire [DATA_WIDTH-1:0]   ch0_fir_result,
    input  wire                    ch0_fir_valid,
    input  wire                    ch0_thresh_irq,
    input  wire                    ch0_thresh_crossing,
    input  wire                    ch0_thresh_valid,

    input  wire [DATA_WIDTH-1:0]   ch1_fir_result,
    input  wire                    ch1_fir_valid,
    input  wire                    ch1_thresh_irq,
    input  wire                    ch1_thresh_crossing,
    input  wire                    ch1_thresh_valid,

    input  wire [DATA_WIDTH-1:0]   ch2_fir_result,
    input  wire                    ch2_fir_valid,
    input  wire                    ch2_thresh_irq,
    input  wire                    ch2_thresh_crossing,
    input  wire                    ch2_thresh_valid,

    output reg  [1:0]              alarm_level,
    output reg  [1:0]              alarm_ch,
    output reg                     event_valid,

    output reg  [DATA_WIDTH-1:0]   ch0_thresh_hi,
    output reg  [DATA_WIDTH-1:0]   ch0_thresh_lo,
    output reg  [DATA_WIDTH-1:0]   ch1_thresh_hi,
    output reg  [DATA_WIDTH-1:0]   ch1_thresh_lo,
    output reg  [DATA_WIDTH-1:0]   ch2_thresh_hi,
    output reg  [DATA_WIDTH-1:0]   ch2_thresh_lo
);

    localparam ALARM_CLEAR    = 2'b00;
    localparam ALARM_WARNING  = 2'b01;
    localparam ALARM_DANGER   = 2'b10;
    localparam ALARM_CRITICAL = 2'b11;

    localparam ACCUM_WIDTH = DATA_WIDTH + LEAK_SHIFT;  // 22-bit accumulator

    reg [DATA_WIDTH-1:0]  ch_irq_lat  [0:2]; 
    reg [2:0]             ch_updated;        

    reg [ACCUM_WIDTH-1:0] baseline [0:2];

    reg [DATA_WIDTH:0] hi_raw;  
    reg [DATA_WIDTH:0] lo_raw;

    always @(posedge clk) begin
        if (!rst_n) begin
            ch_irq_lat[0] <= 1'b0;
            ch_irq_lat[1] <= 1'b0;
            ch_irq_lat[2] <= 1'b0;
            ch_updated    <= 3'b000;
            baseline[0]   <= {ACCUM_WIDTH{1'b0}};
            baseline[1]   <= {ACCUM_WIDTH{1'b0}};
            baseline[2]   <= {ACCUM_WIDTH{1'b0}};
            alarm_level   <= ALARM_CLEAR;
            alarm_ch      <= 2'b0;
            event_valid   <= 1'b0;
            ch0_thresh_hi <= THRESH_HI_OFFSET;
            ch0_thresh_lo <= {DATA_WIDTH{1'b0}};
            ch1_thresh_hi <= THRESH_HI_OFFSET;
            ch1_thresh_lo <= {DATA_WIDTH{1'b0}};
            ch2_thresh_hi <= THRESH_HI_OFFSET;
            ch2_thresh_lo <= {DATA_WIDTH{1'b0}};
        end else begin
            event_valid <= 1'b0;

            // ---------------------------------------------------------------
            // CH0 update
            // ---------------------------------------------------------------
            if (ch0_fir_valid) begin
                baseline[0]   <= baseline[0]
                               - (baseline[0] >> LEAK_SHIFT)
                               + {{LEAK_SHIFT{1'b0}}, ch0_fir_result};
            end
            if (ch0_thresh_valid) begin
                ch_updated[0] <= 1'b1;
                ch_irq_lat[0] <= ch0_thresh_irq;
            end

            // ---------------------------------------------------------------
            // CH1 update
            // ---------------------------------------------------------------
            if (ch1_fir_valid) begin
                baseline[1]   <= baseline[1]
                               - (baseline[1] >> LEAK_SHIFT)
                               + {{LEAK_SHIFT{1'b0}}, ch1_fir_result};
            end
            if (ch1_thresh_valid) begin
                ch_updated[1] <= 1'b1;
                ch_irq_lat[1] <= ch1_thresh_irq;
            end

            // ---------------------------------------------------------------
            // CH2 update
            // ---------------------------------------------------------------
            if (ch2_fir_valid) begin
                baseline[2]   <= baseline[2]
                               - (baseline[2] >> LEAK_SHIFT)
                               + {{LEAK_SHIFT{1'b0}}, ch2_fir_result};
            end
            if (ch2_thresh_valid) begin
                ch_updated[2] <= 1'b1;
                ch_irq_lat[2] <= ch2_thresh_irq;
            end

            // ---------------------------------------------------------------
            // Fusion
            // ---------------------------------------------------------------
            if (ch_updated == 3'b111) begin
                event_valid <= 1'b1;
                ch_updated  <= 3'b000;

                if (ch_irq_lat[0] && ch_irq_lat[1] && ch_irq_lat[2]) begin
                    alarm_level <= ALARM_CRITICAL;
                    alarm_ch    <= 2'd0;
                end else if (ch_irq_lat[0] && ch_irq_lat[1]) begin
                    alarm_level <= ALARM_DANGER;
                    alarm_ch    <= 2'd0;
                end else if (ch_irq_lat[1]) begin
                    alarm_level <= ALARM_WARNING;
                    alarm_ch    <= 2'd1;
                end else if (ch_irq_lat[0]) begin
                    alarm_level <= ALARM_WARNING;
                    alarm_ch    <= 2'd0;
                end else if (ch_irq_lat[2]) begin
                    alarm_level <= ALARM_WARNING;
                    alarm_ch    <= 2'd2;
                end else begin
                    alarm_level <= ALARM_CLEAR;
                    alarm_ch    <= 2'd0;
                end
            end

            // ---------------------------------------------------------------
            // Dynamic threshold output (Hysteresis fixed to ADD)
            // ---------------------------------------------------------------
            hi_raw = {1'b0, baseline[0][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_HI_OFFSET};
            lo_raw = {1'b0, baseline[0][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_LO_OFFSET};
            ch0_thresh_hi <= hi_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : hi_raw[DATA_WIDTH-1:0];
            ch0_thresh_lo <= lo_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : lo_raw[DATA_WIDTH-1:0];

            hi_raw = {1'b0, baseline[1][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_HI_OFFSET};
            lo_raw = {1'b0, baseline[1][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_LO_OFFSET};
            ch1_thresh_hi <= hi_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : hi_raw[DATA_WIDTH-1:0];
            ch1_thresh_lo <= lo_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : lo_raw[DATA_WIDTH-1:0];

            hi_raw = {1'b0, baseline[2][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_HI_OFFSET};
            lo_raw = {1'b0, baseline[2][ACCUM_WIDTH-1:LEAK_SHIFT]} + {1'b0, THRESH_LO_OFFSET};
            ch2_thresh_hi <= hi_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : hi_raw[DATA_WIDTH-1:0];
            ch2_thresh_lo <= lo_raw[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : lo_raw[DATA_WIDTH-1:0];
        end
    end

endmodule