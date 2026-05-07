`timescale 1ns/1ps
// =============================================================================
// packet_fifo.v
// 32-deep synchronous FIFO, 72-bit wide (one complete 9-byte frame per entry)
//
// Frame format (72 bits):
//   [71:64] = 0xAA          start marker
//   [63:62] = alarm_level
//   [61:60] = alarm_ch
//   [59:44] = ch0_fir_result [15:0]
//   [43:28] = ch1_fir_result [15:0]
//   [27:12] = ch2_fir_result [15:0]
//   [11:8]  = 4'b0000        reserved
//   [7:0]   = 0x55           end marker
// =============================================================================

module packet_fifo (
    input  wire        clk,
    input  wire        rst_n,

    // Write port
    input  wire [71:0] wr_data,
    input  wire        wr_en,
    output wire        full,

    // Read port
    output wire [71:0] rd_data,
    input  wire        rd_en,
    output wire        empty,

    // Status
    output wire [4:0]  fill_level
);

    localparam DEPTH = 32;
    localparam ADDR_W = 5;

    reg [71:0] mem [0:DEPTH-1];
    reg [ADDR_W-1:0] wr_ptr, rd_ptr;
    reg [ADDR_W:0]   count;

    assign full       = (count == DEPTH);
    assign empty      = (count == 0);
    assign fill_level = count[4:0];
    assign rd_data    = mem[rd_ptr];

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr      <= wr_ptr + 1;
            end
            if (rd_en && !empty) begin
                rd_ptr <= rd_ptr + 1;
            end
            case ({wr_en && !full, rd_en && !empty})
                2'b10: count <= count + 1;
                2'b01: count <= count - 1;
                default: ;
            endcase
        end
    end

endmodule