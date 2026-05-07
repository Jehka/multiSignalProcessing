`timescale 1ns/1ps
// =============================================================================
// uart_controller_packet.v
// Extended uart_controller for 72-bit packet FIFO.
// Reads one 9-byte frame from packet_fifo and sends all 9 bytes sequentially
// using the existing uart_tx byte-at-a-time interface.
// =============================================================================

module uart_controller_packet (
    input  wire        clk,
    input  wire        rst_n,

    // Packet FIFO side
    input  wire [71:0] fifo_data,
    input  wire        fifo_empty,
    output reg         fifo_rd_en,

    // uart_tx side
    input  wire        tx_busy,
    output reg         tx_start,
    output reg  [7:0]  tx_data
);

    localparam IDLE      = 3'd0;
    localparam READ_FIFO = 3'd1;
    localparam SEND_BYTE = 3'd2;
    localparam WAIT_BYTE = 3'd3;

    reg [2:0]  state;
    reg [71:0] frame_reg;
    reg [3:0]  byte_idx;   // 0..8 (9 bytes)

    // Extract current byte MSB-first from frame register
    // byte 0 = bits [71:64], byte 1 = [63:56], ... byte 8 = [7:0]
    wire [7:0] current_byte = frame_reg[71 - (byte_idx * 8) -: 8];

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= IDLE;
            fifo_rd_en <= 1'b0;
            tx_start   <= 1'b0;
            tx_data    <= 8'b0;
            frame_reg  <= 72'b0;
            byte_idx   <= 4'd0;
        end else begin
            fifo_rd_en <= 1'b0;
            tx_start   <= 1'b0;

            case (state)

                IDLE: begin
                    if (!fifo_empty)
                        state <= READ_FIFO;
                end

                READ_FIFO: begin
                    fifo_rd_en <= 1'b1;
                    frame_reg  <= fifo_data;
                    byte_idx   <= 4'd0;
                    state      <= SEND_BYTE;
                end

                SEND_BYTE: begin
                    if (!tx_busy) begin
                        tx_data  <= current_byte;
                        tx_start <= 1'b1;
                        state    <= WAIT_BYTE;
                    end
                end

                WAIT_BYTE: begin
                    if (!tx_busy) begin
                        if (byte_idx == 4'd8) begin
                            byte_idx <= 4'd0;
                            state    <= IDLE;
                        end else begin
                            byte_idx <= byte_idx + 1;
                            state    <= SEND_BYTE;
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule