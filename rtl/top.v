`timescale 1ns/1ps
// =============================================================================
// top.v — Multichannel Signal Processing System (Top Level)
//
// Integrates:
//   1x spi_adc_master_v2 (12-bit hardware, 16-bit datapath)
//   1x channel_mux_fsm
//   3x axil_master_ch (CH0=MQ-3, CH1=MQ-135, CH2=LDR)
//   3x axi_moving_avg
//   3x thresh_comparator
//   1x dsp_core
//
// Clock: 200MHz differential LVDS input → MMCM → 100MHz internal
// Reset: active-low (CPU_RESET button is active-high — invert externally
//        or add: assign rst_n_int = ~rst_n_btn; in this file)
// =============================================================================

module top (
    // 200MHz differential clock from ZC702 U18
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,

    input  wire        rst_n,       // CPU_RESET (active-low after inversion)

    // SPI bus — PMOD JA
    output wire        sclk,
    output wire        convst_n,
    input  wire        miso,
    input  wire        busy,
    output wire [2:0]  cs_n,

    // Alarm outputs — PMOD JB
    output wire [1:0]  alarm_level,
    output wire [1:0]  alarm_ch,
    output wire        event_valid
);

    localparam DATA_WIDTH = 16;

    // =========================================================================
    // Clock — MMCM: 200MHz differential → 100MHz single-ended
    // Instantiate Vivado clocking wizard IP named clk_wiz_0
    // Config: CLK_IN1_D=200MHz, CLK_OUT1=100MHz, RESET active-high, LOCKED out
    // =========================================================================
    wire clk;        // 100MHz internal clock
    wire mmcm_locked;
    wire rst_n_int;

    clk_wiz_0 u_clk_wiz (
        .clk_in1_p  (sys_clk_p),
        .clk_in1_n  (sys_clk_n),
        .clk_out1   (clk),
        .reset      (1'b0),
        .locked     (mmcm_locked)
    );

    // Hold reset until MMCM is locked
    assign rst_n_int = rst_n & mmcm_locked;

    // =========================================================================
    // SPI Front-End Wires
    // =========================================================================
    wire        spi_start;
    wire [1:0]  spi_ch_sel;
    wire [15:0] spi_sample_data;
    wire [1:0]  spi_sample_ch;
    wire        spi_sample_valid;
    wire        spi_done;

    wire [15:0] mux_ch0_data; wire mux_ch0_valid;
    wire [15:0] mux_ch1_data; wire mux_ch1_valid;
    wire [15:0] mux_ch2_data; wire mux_ch2_valid;

    // =========================================================================
    // DSP Feedback & Result Wires
    // =========================================================================
    wire [DATA_WIDTH-1:0] ch0_thresh_hi, ch0_thresh_lo;
    wire [DATA_WIDTH-1:0] ch1_thresh_hi, ch1_thresh_lo;
    wire [DATA_WIDTH-1:0] ch2_thresh_hi, ch2_thresh_lo;

    wire [DATA_WIDTH-1:0] ch0_fir_result; wire ch0_fir_valid;
    wire ch0_thresh_irq; wire ch0_thresh_crossing; wire ch0_thresh_valid;

    wire [DATA_WIDTH-1:0] ch1_fir_result; wire ch1_fir_valid;
    wire ch1_thresh_irq; wire ch1_thresh_crossing; wire ch1_thresh_valid;

    wire [DATA_WIDTH-1:0] ch2_fir_result; wire ch2_fir_valid;
    wire ch2_thresh_irq; wire ch2_thresh_crossing; wire ch2_thresh_valid;

    // =========================================================================
    // SPI Master
    // =========================================================================
    spi_adc_master_v2 u_spi_master (
        .clk(clk), .rst_n(rst_n_int),
        .start(spi_start), .ch_sel(spi_ch_sel),
        .sclk(sclk), .convst_n(convst_n), .miso(miso), .busy(busy),
        .sample_data(spi_sample_data), .sample_ch(spi_sample_ch),
        .sample_valid(spi_sample_valid), .done(spi_done)
    );

    // =========================================================================
    // Channel Mux FSM
    // =========================================================================
    channel_mux_fsm u_channel_mux (
        .clk(clk), .rst_n(rst_n_int),
        .enable(1'b1),
        .spi_start(spi_start), .spi_ch_sel(spi_ch_sel),
        .spi_sample_data(spi_sample_data), .spi_sample_ch(spi_sample_ch),
        .spi_sample_valid(spi_sample_valid), .spi_done(spi_done),
        .cs_n(cs_n),
        .ch0_data(mux_ch0_data), .ch0_valid(mux_ch0_valid),
        .ch1_data(mux_ch1_data), .ch1_valid(mux_ch1_valid),
        .ch2_data(mux_ch2_data), .ch2_valid(mux_ch2_valid),
        .current_ch()
    );

    // =========================================================================
    // DSP Core
    // =========================================================================
    dsp_core #(.DATA_WIDTH(DATA_WIDTH), .LEAK_SHIFT(6)) u_dsp_core (
        .clk(clk), .rst_n(rst_n_int),
        .ch0_fir_result(ch0_fir_result), .ch0_fir_valid(ch0_fir_valid),
        .ch0_thresh_irq(ch0_thresh_irq), .ch0_thresh_crossing(ch0_thresh_crossing),
        .ch0_thresh_valid(ch0_thresh_valid),
        .ch1_fir_result(ch1_fir_result), .ch1_fir_valid(ch1_fir_valid),
        .ch1_thresh_irq(ch1_thresh_irq), .ch1_thresh_crossing(ch1_thresh_crossing),
        .ch1_thresh_valid(ch1_thresh_valid),
        .ch2_fir_result(ch2_fir_result), .ch2_fir_valid(ch2_fir_valid),
        .ch2_thresh_irq(ch2_thresh_irq), .ch2_thresh_crossing(ch2_thresh_crossing),
        .ch2_thresh_valid(ch2_thresh_valid),
        .alarm_level(alarm_level), .alarm_ch(alarm_ch), .event_valid(event_valid),
        .ch0_thresh_hi(ch0_thresh_hi), .ch0_thresh_lo(ch0_thresh_lo),
        .ch1_thresh_hi(ch1_thresh_hi), .ch1_thresh_lo(ch1_thresh_lo),
        .ch2_thresh_hi(ch2_thresh_hi), .ch2_thresh_lo(ch2_thresh_lo)
    );

    // =========================================================================
    // CH0 (MQ-3) AXI Subsystem
    // =========================================================================
    wire [4:0]  ch0_ma_awaddr; wire ch0_ma_awvalid; wire ch0_ma_awready;
    wire [31:0] ch0_ma_wdata;  wire [3:0] ch0_ma_wstrb;
    wire        ch0_ma_wvalid; wire ch0_ma_wready;
    wire [1:0]  ch0_ma_bresp;  wire ch0_ma_bvalid; wire ch0_ma_bready;
    wire [4:0]  ch0_ma_araddr; wire ch0_ma_arvalid; wire ch0_ma_arready;
    wire [31:0] ch0_ma_rdata;  wire [1:0] ch0_ma_rresp;
    wire        ch0_ma_rvalid; wire ch0_ma_rready;

    wire [4:0]  ch0_tc_awaddr; wire ch0_tc_awvalid; wire ch0_tc_awready;
    wire [31:0] ch0_tc_wdata;  wire [3:0] ch0_tc_wstrb;
    wire        ch0_tc_wvalid; wire ch0_tc_wready;
    wire [1:0]  ch0_tc_bresp;  wire ch0_tc_bvalid; wire ch0_tc_bready;
    wire [4:0]  ch0_tc_araddr; wire ch0_tc_arvalid; wire ch0_tc_arready;
    wire [31:0] ch0_tc_rdata;  wire [1:0] ch0_tc_rresp;
    wire        ch0_tc_rvalid; wire ch0_tc_rready;

    axil_master_ch #(.CH_ID(0),.DATA_WIDTH(DATA_WIDTH)) u_axil_m0 (
        .clk(clk),.rst_n(rst_n_int),
        .sample(mux_ch0_data),.sample_valid(mux_ch0_valid),
        .thresh_hi(ch0_thresh_hi),.thresh_lo(ch0_thresh_lo),
        .ma_awaddr(ch0_ma_awaddr),.ma_awvalid(ch0_ma_awvalid),.ma_awready(ch0_ma_awready),
        .ma_wdata(ch0_ma_wdata),.ma_wstrb(ch0_ma_wstrb),.ma_wvalid(ch0_ma_wvalid),.ma_wready(ch0_ma_wready),
        .ma_bresp(ch0_ma_bresp),.ma_bvalid(ch0_ma_bvalid),.ma_bready(ch0_ma_bready),
        .ma_araddr(ch0_ma_araddr),.ma_arvalid(ch0_ma_arvalid),.ma_arready(ch0_ma_arready),
        .ma_rdata(ch0_ma_rdata),.ma_rresp(ch0_ma_rresp),.ma_rvalid(ch0_ma_rvalid),.ma_rready(ch0_ma_rready),
        .tc_awaddr(ch0_tc_awaddr),.tc_awvalid(ch0_tc_awvalid),.tc_awready(ch0_tc_awready),
        .tc_wdata(ch0_tc_wdata),.tc_wstrb(ch0_tc_wstrb),.tc_wvalid(ch0_tc_wvalid),.tc_wready(ch0_tc_wready),
        .tc_bresp(ch0_tc_bresp),.tc_bvalid(ch0_tc_bvalid),.tc_bready(ch0_tc_bready),
        .tc_araddr(ch0_tc_araddr),.tc_arvalid(ch0_tc_arvalid),.tc_arready(ch0_tc_arready),
        .tc_rdata(ch0_tc_rdata),.tc_rresp(ch0_tc_rresp),.tc_rvalid(ch0_tc_rvalid),.tc_rready(ch0_tc_rready),
        .fir_result(ch0_fir_result),.fir_valid(ch0_fir_valid),
        .thresh_irq(ch0_thresh_irq),.thresh_crossing(ch0_thresh_crossing),.thresh_valid(ch0_thresh_valid)
    );

    axi_moving_avg #(.DATA_WIDTH(DATA_WIDTH)) u_ma0 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch0_ma_awaddr),.s_axil_awvalid(ch0_ma_awvalid),.s_axil_awready(ch0_ma_awready),
        .s_axil_wdata(ch0_ma_wdata),.s_axil_wstrb(ch0_ma_wstrb),.s_axil_wvalid(ch0_ma_wvalid),.s_axil_wready(ch0_ma_wready),
        .s_axil_bresp(ch0_ma_bresp),.s_axil_bvalid(ch0_ma_bvalid),.s_axil_bready(ch0_ma_bready),
        .s_axil_araddr(ch0_ma_araddr),.s_axil_arvalid(ch0_ma_arvalid),.s_axil_arready(ch0_ma_arready),
        .s_axil_rdata(ch0_ma_rdata),.s_axil_rresp(ch0_ma_rresp),.s_axil_rvalid(ch0_ma_rvalid),.s_axil_rready(ch0_ma_rready),
        .irq_result_valid()
    );

    thresh_comparator #(.DATA_WIDTH(DATA_WIDTH)) u_tc0 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch0_tc_awaddr),.s_axil_awvalid(ch0_tc_awvalid),.s_axil_awready(ch0_tc_awready),
        .s_axil_wdata(ch0_tc_wdata),.s_axil_wstrb(ch0_tc_wstrb),.s_axil_wvalid(ch0_tc_wvalid),.s_axil_wready(ch0_tc_wready),
        .s_axil_bresp(ch0_tc_bresp),.s_axil_bvalid(ch0_tc_bvalid),.s_axil_bready(ch0_tc_bready),
        .s_axil_araddr(ch0_tc_araddr),.s_axil_arvalid(ch0_tc_arvalid),.s_axil_arready(ch0_tc_arready),
        .s_axil_rdata(ch0_tc_rdata),.s_axil_rresp(ch0_tc_rresp),.s_axil_rvalid(ch0_tc_rvalid),.s_axil_rready(ch0_tc_rready),
        .irq_out()
    );

    // =========================================================================
    // CH1 (MQ-135) AXI Subsystem
    // =========================================================================
    wire [4:0]  ch1_ma_awaddr; wire ch1_ma_awvalid; wire ch1_ma_awready;
    wire [31:0] ch1_ma_wdata;  wire [3:0] ch1_ma_wstrb;
    wire        ch1_ma_wvalid; wire ch1_ma_wready;
    wire [1:0]  ch1_ma_bresp;  wire ch1_ma_bvalid; wire ch1_ma_bready;
    wire [4:0]  ch1_ma_araddr; wire ch1_ma_arvalid; wire ch1_ma_arready;
    wire [31:0] ch1_ma_rdata;  wire [1:0] ch1_ma_rresp;
    wire        ch1_ma_rvalid; wire ch1_ma_rready;

    wire [4:0]  ch1_tc_awaddr; wire ch1_tc_awvalid; wire ch1_tc_awready;
    wire [31:0] ch1_tc_wdata;  wire [3:0] ch1_tc_wstrb;
    wire        ch1_tc_wvalid; wire ch1_tc_wready;
    wire [1:0]  ch1_tc_bresp;  wire ch1_tc_bvalid; wire ch1_tc_bready;
    wire [4:0]  ch1_tc_araddr; wire ch1_tc_arvalid; wire ch1_tc_arready;
    wire [31:0] ch1_tc_rdata;  wire [1:0] ch1_tc_rresp;
    wire        ch1_tc_rvalid; wire ch1_tc_rready;

    axil_master_ch #(.CH_ID(1),.DATA_WIDTH(DATA_WIDTH)) u_axil_m1 (
        .clk(clk),.rst_n(rst_n_int),
        .sample(mux_ch1_data),.sample_valid(mux_ch1_valid),
        .thresh_hi(ch1_thresh_hi),.thresh_lo(ch1_thresh_lo),
        .ma_awaddr(ch1_ma_awaddr),.ma_awvalid(ch1_ma_awvalid),.ma_awready(ch1_ma_awready),
        .ma_wdata(ch1_ma_wdata),.ma_wstrb(ch1_ma_wstrb),.ma_wvalid(ch1_ma_wvalid),.ma_wready(ch1_ma_wready),
        .ma_bresp(ch1_ma_bresp),.ma_bvalid(ch1_ma_bvalid),.ma_bready(ch1_ma_bready),
        .ma_araddr(ch1_ma_araddr),.ma_arvalid(ch1_ma_arvalid),.ma_arready(ch1_ma_arready),
        .ma_rdata(ch1_ma_rdata),.ma_rresp(ch1_ma_rresp),.ma_rvalid(ch1_ma_rvalid),.ma_rready(ch1_ma_rready),
        .tc_awaddr(ch1_tc_awaddr),.tc_awvalid(ch1_tc_awvalid),.tc_awready(ch1_tc_awready),
        .tc_wdata(ch1_tc_wdata),.tc_wstrb(ch1_tc_wstrb),.tc_wvalid(ch1_tc_wvalid),.tc_wready(ch1_tc_wready),
        .tc_bresp(ch1_tc_bresp),.tc_bvalid(ch1_tc_bvalid),.tc_bready(ch1_tc_bready),
        .tc_araddr(ch1_tc_araddr),.tc_arvalid(ch1_tc_arvalid),.tc_arready(ch1_tc_arready),
        .tc_rdata(ch1_tc_rdata),.tc_rresp(ch1_tc_rresp),.tc_rvalid(ch1_tc_rvalid),.tc_rready(ch1_tc_rready),
        .fir_result(ch1_fir_result),.fir_valid(ch1_fir_valid),
        .thresh_irq(ch1_thresh_irq),.thresh_crossing(ch1_thresh_crossing),.thresh_valid(ch1_thresh_valid)
    );

    axi_moving_avg #(.DATA_WIDTH(DATA_WIDTH)) u_ma1 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch1_ma_awaddr),.s_axil_awvalid(ch1_ma_awvalid),.s_axil_awready(ch1_ma_awready),
        .s_axil_wdata(ch1_ma_wdata),.s_axil_wstrb(ch1_ma_wstrb),.s_axil_wvalid(ch1_ma_wvalid),.s_axil_wready(ch1_ma_wready),
        .s_axil_bresp(ch1_ma_bresp),.s_axil_bvalid(ch1_ma_bvalid),.s_axil_bready(ch1_ma_bready),
        .s_axil_araddr(ch1_ma_araddr),.s_axil_arvalid(ch1_ma_arvalid),.s_axil_arready(ch1_ma_arready),
        .s_axil_rdata(ch1_ma_rdata),.s_axil_rresp(ch1_ma_rresp),.s_axil_rvalid(ch1_ma_rvalid),.s_axil_rready(ch1_ma_rready),
        .irq_result_valid()
    );

    thresh_comparator #(.DATA_WIDTH(DATA_WIDTH)) u_tc1 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch1_tc_awaddr),.s_axil_awvalid(ch1_tc_awvalid),.s_axil_awready(ch1_tc_awready),
        .s_axil_wdata(ch1_tc_wdata),.s_axil_wstrb(ch1_tc_wstrb),.s_axil_wvalid(ch1_tc_wvalid),.s_axil_wready(ch1_tc_wready),
        .s_axil_bresp(ch1_tc_bresp),.s_axil_bvalid(ch1_tc_bvalid),.s_axil_bready(ch1_tc_bready),
        .s_axil_araddr(ch1_tc_araddr),.s_axil_arvalid(ch1_tc_arvalid),.s_axil_arready(ch1_tc_arready),
        .s_axil_rdata(ch1_tc_rdata),.s_axil_rresp(ch1_tc_rresp),.s_axil_rvalid(ch1_tc_rvalid),.s_axil_rready(ch1_tc_rready),
        .irq_out()
    );

    // =========================================================================
    // CH2 (LDR) AXI Subsystem
    // =========================================================================
    wire [4:0]  ch2_ma_awaddr; wire ch2_ma_awvalid; wire ch2_ma_awready;
    wire [31:0] ch2_ma_wdata;  wire [3:0] ch2_ma_wstrb;
    wire        ch2_ma_wvalid; wire ch2_ma_wready;
    wire [1:0]  ch2_ma_bresp;  wire ch2_ma_bvalid; wire ch2_ma_bready;
    wire [4:0]  ch2_ma_araddr; wire ch2_ma_arvalid; wire ch2_ma_arready;
    wire [31:0] ch2_ma_rdata;  wire [1:0] ch2_ma_rresp;
    wire        ch2_ma_rvalid; wire ch2_ma_rready;

    wire [4:0]  ch2_tc_awaddr; wire ch2_tc_awvalid; wire ch2_tc_awready;
    wire [31:0] ch2_tc_wdata;  wire [3:0] ch2_tc_wstrb;
    wire        ch2_tc_wvalid; wire ch2_tc_wready;
    wire [1:0]  ch2_tc_bresp;  wire ch2_tc_bvalid; wire ch2_tc_bready;
    wire [4:0]  ch2_tc_araddr; wire ch2_tc_arvalid; wire ch2_tc_arready;
    wire [31:0] ch2_tc_rdata;  wire [1:0] ch2_tc_rresp;
    wire        ch2_tc_rvalid; wire ch2_tc_rready;

    axil_master_ch #(.CH_ID(2),.DATA_WIDTH(DATA_WIDTH)) u_axil_m2 (
        .clk(clk),.rst_n(rst_n_int),
        .sample(mux_ch2_data),.sample_valid(mux_ch2_valid),
        .thresh_hi(ch2_thresh_hi),.thresh_lo(ch2_thresh_lo),
        .ma_awaddr(ch2_ma_awaddr),.ma_awvalid(ch2_ma_awvalid),.ma_awready(ch2_ma_awready),
        .ma_wdata(ch2_ma_wdata),.ma_wstrb(ch2_ma_wstrb),.ma_wvalid(ch2_ma_wvalid),.ma_wready(ch2_ma_wready),
        .ma_bresp(ch2_ma_bresp),.ma_bvalid(ch2_ma_bvalid),.ma_bready(ch2_ma_bready),
        .ma_araddr(ch2_ma_araddr),.ma_arvalid(ch2_ma_arvalid),.ma_arready(ch2_ma_arready),
        .ma_rdata(ch2_ma_rdata),.ma_rresp(ch2_ma_rresp),.ma_rvalid(ch2_ma_rvalid),.ma_rready(ch2_ma_rready),
        .tc_awaddr(ch2_tc_awaddr),.tc_awvalid(ch2_tc_awvalid),.tc_awready(ch2_tc_awready),
        .tc_wdata(ch2_tc_wdata),.tc_wstrb(ch2_tc_wstrb),.tc_wvalid(ch2_tc_wvalid),.tc_wready(ch2_tc_wready),
        .tc_bresp(ch2_tc_bresp),.tc_bvalid(ch2_tc_bvalid),.tc_bready(ch2_tc_bready),
        .tc_araddr(ch2_tc_araddr),.tc_arvalid(ch2_tc_arvalid),.tc_arready(ch2_tc_arready),
        .tc_rdata(ch2_tc_rdata),.tc_rresp(ch2_tc_rresp),.tc_rvalid(ch2_tc_rvalid),.tc_rready(ch2_tc_rready),
        .fir_result(ch2_fir_result),.fir_valid(ch2_fir_valid),
        .thresh_irq(ch2_thresh_irq),.thresh_crossing(ch2_thresh_crossing),.thresh_valid(ch2_thresh_valid)
    );

    axi_moving_avg #(.DATA_WIDTH(DATA_WIDTH)) u_ma2 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch2_ma_awaddr),.s_axil_awvalid(ch2_ma_awvalid),.s_axil_awready(ch2_ma_awready),
        .s_axil_wdata(ch2_ma_wdata),.s_axil_wstrb(ch2_ma_wstrb),.s_axil_wvalid(ch2_ma_wvalid),.s_axil_wready(ch2_ma_wready),
        .s_axil_bresp(ch2_ma_bresp),.s_axil_bvalid(ch2_ma_bvalid),.s_axil_bready(ch2_ma_bready),
        .s_axil_araddr(ch2_ma_araddr),.s_axil_arvalid(ch2_ma_arvalid),.s_axil_arready(ch2_ma_arready),
        .s_axil_rdata(ch2_ma_rdata),.s_axil_rresp(ch2_ma_rresp),.s_axil_rvalid(ch2_ma_rvalid),.s_axil_rready(ch2_ma_rready),
        .irq_result_valid()
    );

    thresh_comparator #(.DATA_WIDTH(DATA_WIDTH)) u_tc2 (
        .aclk(clk),.aresetn(rst_n_int),
        .s_axil_awaddr(ch2_tc_awaddr),.s_axil_awvalid(ch2_tc_awvalid),.s_axil_awready(ch2_tc_awready),
        .s_axil_wdata(ch2_tc_wdata),.s_axil_wstrb(ch2_tc_wstrb),.s_axil_wvalid(ch2_tc_wvalid),.s_axil_wready(ch2_tc_wready),
        .s_axil_bresp(ch2_tc_bresp),.s_axil_bvalid(ch2_tc_bvalid),.s_axil_bready(ch2_tc_bready),
        .s_axil_araddr(ch2_tc_araddr),.s_axil_arvalid(ch2_tc_arvalid),.s_axil_arready(ch2_tc_arready),
        .s_axil_rdata(ch2_tc_rdata),.s_axil_rresp(ch2_tc_rresp),.s_axil_rvalid(ch2_tc_rvalid),.s_axil_rready(ch2_tc_rready),
        .irq_out()
    );

endmodule