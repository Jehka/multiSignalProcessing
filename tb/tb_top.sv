`timescale 1ns/1ps
// =============================================================================
// tb_top.sv — Phase 4 Top-Level Integration Testbench
//
// Simulates the physical AD7606B-style ADC and tests the entire pipeline:
// SPI fetch -> MUX -> AXI Masters -> FIR -> Comparators -> DSP Fusion
// =============================================================================

module tb_top;

    localparam CLK_PERIOD = 10; // 100 MHz System Clock

    // =========================================================================
    // Clock & Reset
    // =========================================================================
    logic clk, rst_n;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // Top-Level Ports
    // =========================================================================
    wire sclk, convst_n;
    wire [2:0] cs_n;
    logic miso, busy;

    wire [1:0] alarm_level;
    wire [1:0] alarm_ch;
    wire       event_valid;

    top u_top (
        .clk(clk), .rst_n(rst_n),
        .sclk(sclk), .convst_n(convst_n), .miso(miso), .busy(busy), .cs_n(cs_n),
        .alarm_level(alarm_level), .alarm_ch(alarm_ch), .event_valid(event_valid)
    );

    // =========================================================================
    // Simulated Physical ADC (The "Outside World")
    // =========================================================================
    // These registers hold the "real-time analog voltage" of the sensors
    logic [11:0] adc_val_ch0 = 12'h0800; // MQ-3  (Base: 2048)
    logic [11:0] adc_val_ch1 = 12'h0800; // MQ-135(Base: 2048)
    logic [11:0] adc_val_ch2 = 12'h0800; // LDR   (Base: 2048)

    // 1. Simulate the ~4us ADC Busy conversion period
    initial busy = 1'b0;
    always @(negedge convst_n) begin
        busy = 1'b1;
        #4000; // Wait 4us (matches BUSY_CYCLES = 400 in spi_master)
        busy = 1'b0;
    end

    // 2. SPI Shift Register Logic (Slave Mode)
    logic [11:0] shift_reg = 12'b0;

    // Load shift register when a specific CS_N falls
    always @(cs_n) begin
        if      (cs_n == 3'b110) shift_reg = adc_val_ch0;
        else if (cs_n == 3'b101) shift_reg = adc_val_ch1;
        else if (cs_n == 3'b011) shift_reg = adc_val_ch2;
    end

    // Shift data out on falling edge of SCLK (CPHA=1)
    always @(negedge sclk) begin
        if (cs_n != 3'b111) begin
            shift_reg <= {shift_reg[10:0], 1'b0};
        end
    end

    // Drive MISO pin
    assign miso = (cs_n != 3'b111) ? shift_reg[11] : 1'b0;

    // =========================================================================
    // Test Infrastructure
    // =========================================================================
    int pass_count = 0, fail_count = 0, test_num = 1;

    task automatic check(input string name, input logic cond);
        if (cond) begin $display("[PASS] T%02d: %s", test_num, name); pass_count++; end
        else      begin $display("[FAIL] T%02d: %s", test_num, name); fail_count++; end
        test_num++;
    endtask

    // Wait for the DSP to output an event_valid pulse
    task automatic wait_for_event();
        while (!event_valid) @(posedge clk);
        while (event_valid)  @(posedge clk); // wait for pulse to finish
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        $display("=================================================");
        $display("   MULTICHANNEL AIR QUALITY SYSTEM - TOP TEST    ");
        $display("=================================================");

        // Power-on Reset
        rst_n = 0;
        repeat(10) @(posedge clk);
        rst_n = 1;

        // ---------------------------------------------------------------------
        // Stage 1: Initialization & Baseline Build
        // ---------------------------------------------------------------------
        $display("\n[INFO] Powering up AXI Subsystems and settling leaky integrators...");
        #3_500_000; 
        check("System powers up, baseline settles to CLEAR state", alarm_level == 2'b00);

        // ---------------------------------------------------------------------
        // Stage 2: Gas Leaks (Single Channel Alarms)
        // ---------------------------------------------------------------------
        $display("\n[INFO] Simulating MQ-3 (Alcohol/VOC) Gas Spike...");
        adc_val_ch0 = 12'h0F00; // Spike CH0
        repeat(2) wait_for_event(); // Allow 2 rounds for SPI fetch & AXI processing
        check("CH0 Spike registers as WARNING (01)", alarm_level == 2'b01);
        check("Alarm CH points to CH0", alarm_ch == 2'd0);

        $display("\n[INFO] Clearing MQ-3, Spiking MQ-135 (Air Quality)...");
        adc_val_ch0 = 12'h0800; // Clear CH0
        adc_val_ch1 = 12'h0F00; // Spike CH1
        repeat(3) wait_for_event(); // Give pipeline time to clear old CH0 and fetch new CH1
        check("CH1 Spike registers as WARNING (01)", alarm_level == 2'b01);
        check("Alarm CH points to CH1", alarm_ch == 2'd1);

        // ---------------------------------------------------------------------
        // Stage 3: Multi-Sensor Hazards
        // ---------------------------------------------------------------------
        $display("\n[INFO] Simulating both Gas Sensors Spiking...");
        adc_val_ch0 = 12'h0F00; // Spike CH0 (CH1 is already spiking)
        repeat(2) wait_for_event();
        check("CH0 + CH1 Spike registers as DANGER (10)", alarm_level == 2'b10);

        $display("\n[INFO] Simulating Catastrophic Event (All 3 Sensors)...");
        adc_val_ch2 = 12'h0F00; // Spike CH2 (LDR detects flash/fire)
        repeat(2) wait_for_event();
        check("CH0 + CH1 + CH2 Spike registers as CRITICAL (11)", alarm_level == 2'b11);

        // ---------------------------------------------------------------------
        // Stage 4: System Recovery
        // ---------------------------------------------------------------------
        $display("\n[INFO] Event over. Sensors returning to baseline...");
        adc_val_ch0 = 12'h0800;
        adc_val_ch1 = 12'h0800;
        adc_val_ch2 = 12'h0800;
        repeat(5) wait_for_event(); // Wait for data to drop below hysteresis threshold
        check("System correctly recovers to CLEAR (00)", alarm_level == 2'b00);

        // ---------------------------------------------------------------------
        // Summary
        // ---------------------------------------------------------------------
        $display("\n----------------------------------------");
        $display("Top-Level Results: %0d/%0d passed", pass_count, pass_count + fail_count);
        if (fail_count == 0) $display(">>> FULL PIPELINE VERIFIED <<<");
        $display("----------------------------------------");
        $finish;
    end

endmodule
