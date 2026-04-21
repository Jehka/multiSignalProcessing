`timescale 1ns/1ps
// =============================================================================
// axil_master_ch.v
// Per-Channel AXI-Lite Master (Deadlock Patched)
// =============================================================================

module axil_master_ch #(
    parameter CH_ID      = 0,
    parameter DATA_WIDTH = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,

    input  wire [DATA_WIDTH-1:0]   sample,
    input  wire                    sample_valid,

    input  wire [DATA_WIDTH-1:0]   thresh_hi,
    input  wire [DATA_WIDTH-1:0]   thresh_lo,

    output reg  [4:0]  ma_awaddr,
    output reg         ma_awvalid,
    input  wire        ma_awready,
    output reg  [31:0] ma_wdata,
    output reg  [3:0]  ma_wstrb,
    output reg         ma_wvalid,
    input  wire        ma_wready,
    input  wire [1:0]  ma_bresp,
    input  wire        ma_bvalid,
    output reg         ma_bready,
    output reg  [4:0]  ma_araddr,
    output reg         ma_arvalid,
    input  wire        ma_arready,
    input  wire [31:0] ma_rdata,
    input  wire [1:0]  ma_rresp,
    input  wire        ma_rvalid,
    output reg         ma_rready,

    output reg  [4:0]  tc_awaddr,
    output reg         tc_awvalid,
    input  wire        tc_awready,
    output reg  [31:0] tc_wdata,
    output reg  [3:0]  tc_wstrb,
    output reg         tc_wvalid,
    input  wire        tc_wready,
    input  wire [1:0]  tc_bresp,
    input  wire        tc_bvalid,
    output reg         tc_bready,
    output reg  [4:0]  tc_araddr,
    output reg         tc_arvalid,
    input  wire        tc_arready,
    input  wire [31:0] tc_rdata,
    input  wire [1:0]  tc_rresp,
    input  wire        tc_rvalid,
    output reg         tc_rready,

    output reg  [DATA_WIDTH-1:0]   fir_result,
    output reg                     fir_valid,
    output reg                     thresh_irq,
    output reg                     thresh_crossing,
    output reg                     thresh_valid
);

    localparam MA_CTRL    = 5'h00;
    localparam MA_CONFIG  = 5'h04;
    localparam MA_DATA_IN = 5'h08;
    localparam MA_RESULT  = 5'h0C;

    localparam TC_CTRL      = 5'h00;
    localparam TC_THRESH_HI = 5'h04;
    localparam TC_THRESH_LO = 5'h08;
    localparam TC_DATA_IN   = 5'h0C;
    localparam TC_STATUS    = 5'h10;

    // CHANGE THIS LINE from 0x0A to 0x08 (DIR_ABOVE)
    localparam TC_CTRL_VAL  = 32'h0000_0008;

    localparam ST_INIT_MA_CTRL  = 5'd0;
    localparam ST_INIT_MA_WRESP = 5'd1;
    localparam ST_INIT_MA_CFG   = 5'd2;
    localparam ST_INIT_MA_CRESP = 5'd3;
    localparam ST_INIT_TC_CTRL  = 5'd4;
    localparam ST_INIT_TC_WRESP = 5'd5;
    localparam ST_INIT_TC_HI    = 5'd6;
    localparam ST_INIT_TC_HRESP = 5'd7;
    localparam ST_INIT_TC_LO    = 5'd8;
    localparam ST_INIT_TC_LRESP = 5'd9;
    localparam ST_IDLE          = 5'd10;
    localparam ST_WR_MA         = 5'd11;
    localparam ST_WR_MA_RESP    = 5'd12;
    localparam ST_RD_MA_ADDR    = 5'd13;
    localparam ST_RD_MA_DATA    = 5'd14;
    localparam ST_WR_TC_HI      = 5'd15;
    localparam ST_WR_TC_HI_RESP = 5'd16;
    localparam ST_WR_TC_LO      = 5'd17;
    localparam ST_WR_TC_LO_RESP = 5'd18;
    localparam ST_WR_TC_DIN     = 5'd19;
    localparam ST_WR_TC_DRESP   = 5'd20;
    localparam ST_RD_TC_ADDR    = 5'd21;
    localparam ST_RD_TC_DATA    = 5'd22;

    reg [4:0] state;

    reg [DATA_WIDTH-1:0] sample_lat;
    reg [DATA_WIDTH-1:0] thresh_hi_lat;
    reg [DATA_WIDTH-1:0] thresh_lo_lat;

    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= ST_INIT_MA_CTRL;
            sample_lat      <= {DATA_WIDTH{1'b0}};
            thresh_hi_lat   <= {DATA_WIDTH{1'b0}};
            thresh_lo_lat   <= {DATA_WIDTH{1'b0}};

            ma_awaddr  <= 5'b0; ma_awvalid <= 1'b0;
            ma_wdata   <= 32'b0; ma_wstrb  <= 4'hF; ma_wvalid <= 1'b0;
            ma_bready  <= 1'b0;
            ma_araddr  <= 5'b0; ma_arvalid <= 1'b0;
            ma_rready  <= 1'b0;

            tc_awaddr  <= 5'b0; tc_awvalid <= 1'b0;
            tc_wdata   <= 32'b0; tc_wstrb  <= 4'hF; tc_wvalid <= 1'b0;
            tc_bready  <= 1'b0;
            tc_araddr  <= 5'b0; tc_arvalid <= 1'b0;
            tc_rready  <= 1'b0;

            fir_result      <= {DATA_WIDTH{1'b0}};
            fir_valid       <= 1'b0;
            thresh_irq      <= 1'b0;
            thresh_crossing <= 1'b0;
            thresh_valid    <= 1'b0;
        end else begin
            fir_valid    <= 1'b0;
            thresh_valid <= 1'b0;

            case (state)

                ST_INIT_MA_CTRL: begin
                    ma_awaddr <= MA_CTRL;
                    ma_wdata  <= 32'h0000_0001;
                    ma_wstrb  <= 4'hF;
                    ma_bready <= 1'b1;
                    if (!ma_awvalid) begin
                        ma_awvalid <= 1'b1;
                        ma_wvalid  <= 1'b1;
                    end else if (ma_awready && ma_wready) begin
                        ma_awvalid <= 1'b0;
                        ma_wvalid  <= 1'b0;
                        state      <= ST_INIT_MA_WRESP;
                    end
                end

                ST_INIT_MA_WRESP: begin
                    if (ma_bvalid) begin
                        ma_bready <= 1'b0;
                        state     <= ST_INIT_MA_CFG;
                    end
                end

                ST_INIT_MA_CFG: begin
                    ma_awaddr <= MA_CONFIG;
                    ma_wdata  <= 32'h0000_0007;
                    ma_wstrb  <= 4'hF;
                    ma_bready <= 1'b1;
                    if (!ma_awvalid) begin
                        ma_awvalid <= 1'b1;
                        ma_wvalid  <= 1'b1;
                    end else if (ma_awready && ma_wready) begin
                        ma_awvalid <= 1'b0;
                        ma_wvalid  <= 1'b0;
                        state      <= ST_INIT_MA_CRESP;
                    end
                end

                ST_INIT_MA_CRESP: begin
                    if (ma_bvalid) begin
                        ma_bready <= 1'b0;
                        state     <= ST_INIT_TC_CTRL;
                    end
                end

                ST_INIT_TC_CTRL: begin
                    tc_awaddr <= TC_CTRL;
                    tc_wdata  <= TC_CTRL_VAL;
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_INIT_TC_WRESP;
                    end
                end

                ST_INIT_TC_WRESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_INIT_TC_HI;
                    end
                end

                ST_INIT_TC_HI: begin
                    tc_awaddr <= TC_THRESH_HI;
                    tc_wdata  <= {{(32-DATA_WIDTH){1'b0}}, thresh_hi};
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_INIT_TC_HRESP;
                    end
                end

                ST_INIT_TC_HRESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_INIT_TC_LO;
                    end
                end

                ST_INIT_TC_LO: begin
                    tc_awaddr <= TC_THRESH_LO;
                    tc_wdata  <= {{(32-DATA_WIDTH){1'b0}}, thresh_lo};
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_INIT_TC_LRESP;
                    end
                end

                ST_INIT_TC_LRESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                ST_IDLE: begin
                    if (sample_valid) begin
                        sample_lat    <= sample;
                        thresh_hi_lat <= thresh_hi;
                        thresh_lo_lat <= thresh_lo;
                        state         <= ST_WR_MA;
                    end
                end

                ST_WR_MA: begin
                    ma_awaddr <= MA_DATA_IN;
                    ma_wdata  <= {{(32-DATA_WIDTH){1'b0}}, sample_lat};
                    ma_wstrb  <= 4'hF;
                    ma_bready <= 1'b1;
                    if (!ma_awvalid) begin
                        ma_awvalid <= 1'b1;
                        ma_wvalid  <= 1'b1;
                    end else if (ma_awready && ma_wready) begin
                        ma_awvalid <= 1'b0;
                        ma_wvalid  <= 1'b0;
                        state      <= ST_WR_MA_RESP;
                    end
                end

                ST_WR_MA_RESP: begin
                    if (ma_bvalid) begin
                        ma_bready <= 1'b0;
                        state     <= ST_RD_MA_ADDR;
                    end
                end

                ST_RD_MA_ADDR: begin
                    ma_araddr <= MA_RESULT;
                    ma_rready <= 1'b1;
                    if (!ma_arvalid) begin
                        ma_arvalid <= 1'b1;
                    end else if (ma_arready) begin
                        ma_arvalid <= 1'b0;
                        state      <= ST_RD_MA_DATA;
                    end
                end

                ST_RD_MA_DATA: begin
                    if (ma_rvalid) begin
                        fir_result <= ma_rdata[DATA_WIDTH-1:0];
                        fir_valid  <= 1'b1;
                        ma_rready  <= 1'b0;
                        state      <= ST_WR_TC_HI;
                    end
                end

                ST_WR_TC_HI: begin
                    tc_awaddr <= TC_THRESH_HI;
                    tc_wdata  <= {{(32-DATA_WIDTH){1'b0}}, thresh_hi_lat};
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_WR_TC_HI_RESP;
                    end
                end

                ST_WR_TC_HI_RESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_WR_TC_LO;
                    end
                end

                ST_WR_TC_LO: begin
                    tc_awaddr <= TC_THRESH_LO;
                    tc_wdata  <= {{(32-DATA_WIDTH){1'b0}}, thresh_lo_lat};
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_WR_TC_LO_RESP;
                    end
                end

                ST_WR_TC_LO_RESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_WR_TC_DIN;
                    end
                end

                ST_WR_TC_DIN: begin
                    tc_awaddr <= TC_DATA_IN;
                    tc_wdata  <= {{(32-DATA_WIDTH){1'b0}}, sample_lat};
                    tc_wstrb  <= 4'hF;
                    tc_bready <= 1'b1;
                    if (!tc_awvalid) begin
                        tc_awvalid <= 1'b1;
                        tc_wvalid  <= 1'b1;
                    end else if (tc_awready && tc_wready) begin
                        tc_awvalid <= 1'b0;
                        tc_wvalid  <= 1'b0;
                        state      <= ST_WR_TC_DRESP;
                    end
                end

                ST_WR_TC_DRESP: begin
                    if (tc_bvalid) begin
                        tc_bready <= 1'b0;
                        state     <= ST_RD_TC_ADDR;
                    end
                end

                ST_RD_TC_ADDR: begin
                    tc_araddr <= TC_STATUS;
                    tc_rready <= 1'b1;
                    if (!tc_arvalid) begin
                        tc_arvalid <= 1'b1;
                    end else if (tc_arready) begin
                        tc_arvalid <= 1'b0;
                        state      <= ST_RD_TC_DATA;
                    end
                end

                ST_RD_TC_DATA: begin
                    if (tc_rvalid) begin
                        thresh_irq      <= tc_rdata[0];
                        thresh_crossing <= tc_rdata[1];
                        thresh_valid    <= 1'b1;
                        tc_rready       <= 1'b0;
                        state           <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule