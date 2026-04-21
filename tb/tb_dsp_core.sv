`timescale 1ns/1ps
// =============================================================================
// tb_dsp_core.sv — Phase 3 Testbench
// Tests:
//   T01  Reset state — alarm_level=CLEAR, event_valid=0
//   T02  No event before all 3 channels updated
//   T03  CLEAR — all 3 updated, no IRQs
//   T04  WARNING — MQ-3 (ch0) IRQ only
//   T05  WARNING — MQ-135 (ch1) IRQ only
//   T06  WARNING — LDR (ch2) IRQ only
//   T07  DANGER  — MQ-3 + MQ-135 both IRQ
//   T08  CRITICAL — all 3 IRQ
//   T09  alarm_ch correct for DANGER (ch0)
//   T10  Dynamic thresholds update after baseline settles
//   T11  event_valid is 1-cycle pulse only
//   T12  ch_updated clears after fusion — second round fires independently
// =============================================================================

module tb_dsp_core;

    localparam CLK_PERIOD = 10;
    localparam DATA_WIDTH = 16;

    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // DUT ports
    logic [DATA_WIDTH-1:0] ch0_fir_result, ch1_fir_result, ch2_fir_result;
    logic                  ch0_fir_valid,  ch1_fir_valid,  ch2_fir_valid;
    logic                  ch0_thresh_irq, ch1_thresh_irq, ch2_thresh_irq;
    logic                  ch0_thresh_crossing, ch1_thresh_crossing, ch2_thresh_crossing;
    logic                  ch0_thresh_valid,    ch1_thresh_valid,    ch2_thresh_valid;

    wire [1:0]             alarm_level;
    wire [1:0]             alarm_ch;
    wire                   event_valid;
    wire [DATA_WIDTH-1:0]  ch0_thresh_hi, ch0_thresh_lo;
    wire [DATA_WIDTH-1:0]  ch1_thresh_hi, ch1_thresh_lo;
    wire [DATA_WIDTH-1:0]  ch2_thresh_hi, ch2_thresh_lo;

    dsp_core #(
        .DATA_WIDTH      (DATA_WIDTH),
        .LEAK_SHIFT      (6),
        .THRESH_HI_OFFSET(16'h0800),
        .THRESH_LO_OFFSET(16'h0400)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
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
    // Test Infrastructure
    // =========================================================================
    int pass_count, fail_count, test_num;

    task automatic check(input string name, input logic cond);
        if (cond) begin $display("[PASS] T%02d: %s", test_num, name); pass_count++; end
        else      begin $display("[FAIL] T%02d: %s", test_num, name); fail_count++; end
        test_num++;
    endtask

    // Drive all three channels with given IRQ states, wait for event_valid
    // fir_valid and thresh_valid pulse on same cycle for simplicity
    task automatic send_round(
        input logic [DATA_WIDTH-1:0] s0, s1, s2,
        input logic irq0, irq1, irq2,
        input int   timeout
    );
        automatic int ctr = 0;
        @(posedge clk); #1;
        // CH0
        ch0_fir_result = s0; ch0_fir_valid = 1;
        ch0_thresh_irq = irq0; ch0_thresh_crossing = 0; ch0_thresh_valid = 1;
        @(posedge clk); #1;
        ch0_fir_valid = 0; ch0_thresh_valid = 0;
        // CH1
        ch1_fir_result = s1; ch1_fir_valid = 1;
        ch1_thresh_irq = irq1; ch1_thresh_crossing = 0; ch1_thresh_valid = 1;
        @(posedge clk); #1;
        ch1_fir_valid = 0; ch1_thresh_valid = 0;
        // CH2
        ch2_fir_result = s2; ch2_fir_valid = 1;
        ch2_thresh_irq = irq2; ch2_thresh_crossing = 0; ch2_thresh_valid = 1;
        @(posedge clk); #1;
        ch2_fir_valid = 0; ch2_thresh_valid = 0;
        // Wait for event_valid
        while (!event_valid && ctr < timeout) begin
            @(posedge clk); #1;
            ctr++;
        end
    endtask

    // Track event_valid pulse width
    int ev_pulse_count;
    always @(posedge clk)
        if (event_valid) ev_pulse_count++;

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_dsp_core.vcd");
        $dumpvars(0, tb_dsp_core);

        pass_count = 0; fail_count = 0; test_num = 1;
        ev_pulse_count = 0;

        // Init all inputs
        ch0_fir_result = 0; ch0_fir_valid = 0;
        ch0_thresh_irq = 0; ch0_thresh_crossing = 0; ch0_thresh_valid = 0;
        ch1_fir_result = 0; ch1_fir_valid = 0;
        ch1_thresh_irq = 0; ch1_thresh_crossing = 0; ch1_thresh_valid = 0;
        ch2_fir_result = 0; ch2_fir_valid = 0;
        ch2_thresh_irq = 0; ch2_thresh_crossing = 0; ch2_thresh_valid = 0;

        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        @(posedge clk); #1;

        // T01: Reset state
        check("Reset: alarm_level=CLEAR",  alarm_level == 2'b00);
        check("Reset: event_valid=0",      !event_valid);

        // T02: Only 2 channels updated — no event yet
        @(posedge clk); #1;
        ch0_fir_result = 16'h0800; ch0_fir_valid = 1;
        ch0_thresh_irq = 0; ch0_thresh_valid = 1;
        @(posedge clk); #1;
        ch0_fir_valid = 0; ch0_thresh_valid = 0;
        ch1_fir_result = 16'h0800; ch1_fir_valid = 1;
        ch1_thresh_irq = 0; ch1_thresh_valid = 1;
        @(posedge clk); #1;
        ch1_fir_valid = 0; ch1_thresh_valid = 0;
        @(posedge clk); #1;
        check("No event with only 2 channels updated", !event_valid);

        // T03: CLEAR — all 3 updated, no IRQs
        send_round(16'h0800, 16'h0800, 16'h0800, 0, 0, 0, 20);
        check("CLEAR when no IRQs",  alarm_level == 2'b00);

        // T04: WARNING — ch0 only
        send_round(16'h0F00, 16'h0800, 16'h0800, 1, 0, 0, 20);
        check("WARNING: ch0 IRQ only", alarm_level == 2'b01);

        // T05: WARNING — ch1 only
        send_round(16'h0800, 16'h0F00, 16'h0800, 0, 1, 0, 20);
        check("WARNING: ch1 IRQ only", alarm_level == 2'b01);

        // T06: WARNING — ch2 only
        send_round(16'h0800, 16'h0800, 16'h0F00, 0, 0, 1, 20);
        check("WARNING: ch2 IRQ only", alarm_level == 2'b01);

        // T07: DANGER — ch0 + ch1
        send_round(16'h0F00, 16'h0F00, 16'h0800, 1, 1, 0, 20);
        check("DANGER: ch0+ch1 IRQ",   alarm_level == 2'b10);

        // T08: CRITICAL — all 3
        send_round(16'h0F00, 16'h0F00, 16'h0F00, 1, 1, 1, 20);
        check("CRITICAL: all 3 IRQ",   alarm_level == 2'b11);

        // T09: alarm_ch correct on DANGER
        send_round(16'h0F00, 16'h0F00, 16'h0800, 1, 1, 0, 20);
        check("alarm_ch=0 on DANGER",  alarm_ch == 2'd0);

        // T10: Dynamic thresholds non-zero after baseline settles
        // Run 100 rounds of mid-range samples to build baseline
        repeat(100) send_round(16'h0800, 16'h0800, 16'h0800, 0, 0, 0, 20);
        check("ch0_thresh_hi non-zero after baseline build", ch0_thresh_hi > 0);
        check("ch0_thresh_hi > ch0_thresh_lo",               ch0_thresh_hi > ch0_thresh_lo);
        check("ch1_thresh_hi non-zero after baseline build", ch1_thresh_hi > 0);

        // T11: event_valid is single-cycle — count pulses over one round
        
        // --- THE FIX: Wait for the previous round's pulse to clear ---
        while (event_valid) @(posedge clk);
        // -------------------------------------------------------------
        
        ev_pulse_count = 0;
        send_round(16'h0800, 16'h0800, 16'h0800, 0, 0, 0, 20);
        repeat(5) @(posedge clk);
        check("event_valid is 1-cycle pulse", ev_pulse_count == 1);

        // T12: ch_updated clears — second independent round fires
        send_round(16'h0F00, 16'h0F00, 16'h0800, 1, 1, 0, 20);
        check("Second round fires independently", alarm_level == 2'b10);

        $display("----------------------------------------");
        $display("Results: %0d/%0d passed", pass_count, pass_count + fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("%0d TEST(S) FAILED", fail_count);
        $display("----------------------------------------");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("[TIMEOUT] Exceeded 2ms");
        $finish;
    end

endmodule