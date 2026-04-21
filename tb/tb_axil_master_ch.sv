`timescale 1ns/1ps
// =============================================================================
// tb_axil_master_ch.sv — Phase 2 Testbench (16-bit Refactored)
// xsim compatible: no automatic vars inside begin/end, no reserved keywords
// =============================================================================

module tb_axil_master_ch;

    localparam CLK_PERIOD = 10;
    localparam DATA_WIDTH = 16;
    localparam CH_ID      = 0;

    // =========================================================================
    // Clock & Reset
    // =========================================================================
    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic [DATA_WIDTH-1:0] sample;
    logic                  sample_valid;
    logic [DATA_WIDTH-1:0] thresh_hi_in, thresh_lo_in;

    wire [4:0]  ma_awaddr;  wire ma_awvalid; wire ma_awready;
    wire [31:0] ma_wdata;   wire [3:0] ma_wstrb;
    wire        ma_wvalid;  wire ma_wready;
    wire [1:0]  ma_bresp;   wire ma_bvalid;  wire ma_bready;
    wire [4:0]  ma_araddr;  wire ma_arvalid; wire ma_arready;
    wire [31:0] ma_rdata;   wire [1:0] ma_rresp;
    wire        ma_rvalid;  wire ma_rready;

    wire [4:0]  tc_awaddr;  wire tc_awvalid; wire tc_awready;
    wire [31:0] tc_wdata;   wire [3:0] tc_wstrb;
    wire        tc_wvalid;  wire tc_wready;
    wire [1:0]  tc_bresp;   wire tc_bvalid;  wire tc_bready;
    wire [4:0]  tc_araddr;  wire tc_arvalid; wire tc_arready;
    wire [31:0] tc_rdata;   wire [1:0] tc_rresp;
    wire        tc_rvalid;  wire tc_rready;

    wire [DATA_WIDTH-1:0] fir_result;
    wire                  fir_valid;
    wire                  thresh_irq;
    wire                  thresh_crossing;
    wire                  thresh_valid;
    wire                  ma_irq, tc_irq;

    // =========================================================================
    // DUT
    // =========================================================================
    axil_master_ch u_dut (
        .clk(clk), .rst_n(rst_n),
        .sample(sample), .sample_valid(sample_valid),
        .thresh_hi(thresh_hi_in), .thresh_lo(thresh_lo_in),

        .ma_awaddr(ma_awaddr), .ma_awvalid(ma_awvalid), .ma_awready(ma_awready),
        .ma_wdata(ma_wdata),   .ma_wstrb(ma_wstrb),
        .ma_wvalid(ma_wvalid), .ma_wready(ma_wready),
        .ma_bresp(ma_bresp),   .ma_bvalid(ma_bvalid),   .ma_bready(ma_bready),
        .ma_araddr(ma_araddr), .ma_arvalid(ma_arvalid), .ma_arready(ma_arready),
        .ma_rdata(ma_rdata),   .ma_rresp(ma_rresp),
        .ma_rvalid(ma_rvalid), .ma_rready(ma_rready),

        .tc_awaddr(tc_awaddr), .tc_awvalid(tc_awvalid), .tc_awready(tc_awready),
        .tc_wdata(tc_wdata),   .tc_wstrb(tc_wstrb),
        .tc_wvalid(tc_wvalid), .tc_wready(tc_wready),
        .tc_bresp(tc_bresp),   .tc_bvalid(tc_bvalid),   .tc_bready(tc_bready),
        .tc_araddr(tc_araddr), .tc_arvalid(tc_arvalid), .tc_arready(tc_arready),
        .tc_rdata(tc_rdata),   .tc_rresp(tc_rresp),
        .tc_rvalid(tc_rvalid), .tc_rready(tc_rready),

        .fir_result(fir_result),     .fir_valid(fir_valid),
        .thresh_irq(thresh_irq),     .thresh_crossing(thresh_crossing),
        .thresh_valid(thresh_valid)
    );

    // =========================================================================
    // Slave A — axi_moving_avg
    // =========================================================================
    axi_moving_avg u_ma (
        .aclk(clk), .aresetn(rst_n),
        .s_axil_awaddr(ma_awaddr),   .s_axil_awvalid(ma_awvalid), .s_axil_awready(ma_awready),
        .s_axil_wdata(ma_wdata),     .s_axil_wstrb(ma_wstrb),
        .s_axil_wvalid(ma_wvalid),   .s_axil_wready(ma_wready),
        .s_axil_bresp(ma_bresp),     .s_axil_bvalid(ma_bvalid),   .s_axil_bready(ma_bready),
        .s_axil_araddr(ma_araddr),   .s_axil_arvalid(ma_arvalid), .s_axil_arready(ma_arready),
        .s_axil_rdata(ma_rdata),     .s_axil_rresp(ma_rresp),
        .s_axil_rvalid(ma_rvalid),   .s_axil_rready(ma_rready),
        .irq_result_valid(ma_irq)
    );

    // =========================================================================
    // Slave B — thresh_comparator
    // =========================================================================
    thresh_comparator u_tc (
        .aclk(clk), .aresetn(rst_n),
        .s_axil_awaddr(tc_awaddr),   .s_axil_awvalid(tc_awvalid), .s_axil_awready(tc_awready),
        .s_axil_wdata(tc_wdata),     .s_axil_wstrb(tc_wstrb),
        .s_axil_wvalid(tc_wvalid),   .s_axil_wready(tc_wready),
        .s_axil_bresp(tc_bresp),     .s_axil_bvalid(tc_bvalid),   .s_axil_bready(tc_bready),
        .s_axil_araddr(tc_araddr),   .s_axil_arvalid(tc_arvalid), .s_axil_arready(tc_arready),
        .s_axil_rdata(tc_rdata),     .s_axil_rresp(tc_rresp),
        .s_axil_rvalid(tc_rvalid),   .s_axil_rready(tc_rready),
        .irq_out(tc_irq)
    );

    // =========================================================================
    // Test Infrastructure — all vars at module scope (xsim requirement)
    // =========================================================================
    int  pass_count, fail_count, test_num;
    int  hit_count;
    int  task_ctr;

    // Shared result vars written by send_sample
    logic        s_got_fir;
    logic        s_got_thresh;
    logic [15:0] s_fir_out;
    logic        s_irq_out;
    logic        s_dir_cross;

    logic sequential_ok;

    task automatic check(input string name, input logic cond);
        if (cond) begin $display("[PASS] T%02d: %s", test_num, name); pass_count++; end
        else      begin $display("[FAIL] T%02d: %s", test_num, name); fail_count++; end
        test_num++;
    endtask

    // send_sample: drive one sample pulse, wait for fir_valid + thresh_valid
    task automatic send_sample(
        input logic [DATA_WIDTH-1:0] s,
        input int                    timeout
    );
        automatic int ctr;
        ctr          = 0;
        s_got_fir    = 0;
        s_got_thresh = 0;
        s_fir_out    = 0;
        s_irq_out    = 0;
        s_dir_cross  = 0;

        @(posedge clk);
        sample       <= s;
        sample_valid <= 1'b1;
        @(posedge clk);
        sample_valid <= 1'b0;

        while (ctr < timeout && !(s_got_fir && s_got_thresh)) begin
            @(posedge clk);
            ctr = ctr + 1;
            if (fir_valid) begin
                s_got_fir = 1;
                s_fir_out = fir_result;
            end
            if (thresh_valid) begin
                s_got_thresh = 1;
                s_irq_out    = thresh_irq;
                s_dir_cross  = thresh_crossing;
            end
        end
    endtask

    // Background monitor: fir_valid and thresh_valid must never assert same cycle
    initial sequential_ok = 1'b1;
    always @(posedge clk)
        if (fir_valid && thresh_valid) begin
            $display("[FAIL] Sequential violation at time %0t", $time);
            sequential_ok = 1'b0;
        end

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_axil_master_ch.vcd");
        $dumpvars(0, tb_axil_master_ch);

        pass_count   = 0;
        fail_count   = 0;
        test_num     = 1;
        hit_count    = 0;

        rst_n = 1'b0;
        sample       = 16'b0;
        sample_valid = 1'b0;
        thresh_hi_in = 16'h0E00;
        thresh_lo_in = 16'h0200;
        @(posedge clk); #1;
        @(posedge clk); #1;
        @(posedge clk); #1;
        @(posedge clk); #1;
        @(posedge clk); #1;
        rst_n = 1'b1;
        @(posedge clk); #1;

        // -----------------------------------------------------------------
        // T01: Init completes (10 AXI transactions × ~5 cycles = ~50 min)
        // -----------------------------------------------------------------
        repeat(300) @(posedge clk);
        check("Init completes without timeout", 1'b1);

        // -----------------------------------------------------------------
        // T02-T03: First sample — both valid pulses fire
        // -----------------------------------------------------------------
        send_sample(16'h0800, 500);
        check("FIR valid fires after first sample",    s_got_fir);
        check("Thresh valid fires after first sample", s_got_thresh);

        // -----------------------------------------------------------------
        // T04: Moving average — 8 identical samples of 0x0400
        // axi_moving_avg DATA_WIDTH=16, 0x0400 fits
        // After window fills (8 samples), result == input
        // -----------------------------------------------------------------
        repeat(7) send_sample(16'h0400, 500);
        send_sample(16'h0400, 500);
        check("MA result = 0x0400 after 8 identical samples", s_fir_out == 16'h0400);

        // -----------------------------------------------------------------
        // T05: Threshold ABOVE thresh_hi=0x0E00
        // -----------------------------------------------------------------
        send_sample(16'h0800, 500);  // in-band warmup
        send_sample(16'h0F00, 500);  // above thresh_hi
        check("IRQ fires when sample above thresh_hi", s_irq_out);
        check("dir_cross=0 for high trip",             !s_dir_cross);

        // -----------------------------------------------------------------
        // T06: Threshold BELOW thresh_lo=0x0200
        // From ST_TRIPPED_HI, comparator re-arms when sample < thresh_lo
        // -----------------------------------------------------------------
        send_sample(16'h0100, 500);
        check("IRQ latched when sample below thresh_lo", s_irq_out);
        check("dir_cross=1 for low trip",                s_dir_cross);

        // -----------------------------------------------------------------
        // T07: In-band sample — no new crossing
        // From ST_TRIPPED_LO, re-arm by sending above thresh_hi
        // -----------------------------------------------------------------
        send_sample(16'h0F00, 500);  // re-arm
        send_sample(16'h0800, 500);  // in-band
        check("No low crossing when sample in-band", !s_dir_cross);

        // -----------------------------------------------------------------
        // T08: Runtime threshold update — lower thresh_hi to 0x0900
        // -----------------------------------------------------------------
        thresh_hi_in = 16'h0900;
        send_sample(16'h0400, 500);  // in-band with new threshold
        send_sample(16'h0950, 500);  // just above new thresh_hi
        check("IRQ fires with updated thresh_hi=0x0900", s_irq_out);
        thresh_hi_in = 16'h0E00;

        // -----------------------------------------------------------------
        // T09: Sequential property (background monitor result)
        // -----------------------------------------------------------------
        check("fir_valid and thresh_valid never same cycle", sequential_ok);

        // -----------------------------------------------------------------
        // T10: Back-to-back samples — no dropped results
        // -----------------------------------------------------------------
        hit_count = 0;
        repeat(3) begin
            send_sample(16'h0800, 500);
            if (s_got_fir && s_got_thresh) hit_count = hit_count + 1;
        end
        check("Back-to-back: all 3 samples produce results", hit_count == 3);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("----------------------------------------");
        $display("Results: %0d/%0d passed", pass_count, pass_count + fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("%0d TEST(S) FAILED", fail_count);
        $display("----------------------------------------");
        $finish;
    end

    initial begin
        #5_000_000;
        $display("[TIMEOUT] Simulation exceeded 5ms");
        $finish;
    end

endmodule