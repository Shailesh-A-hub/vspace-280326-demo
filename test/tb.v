`default_nettype none
`timescale 1ns / 1ps

module tb ();

  // Dump the signals to a FST file.
  initial begin
    $display("Force dumping data now");
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1; // The template includes this 1ns delay to ensure simulator stability
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Template's way of defining power wires for Gate Level simulation
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

// Instantiate OUR module. Hide the parameter during Gate Level tests.
    tt_um_advaittej_stopwatch 
`ifndef GL_TEST
    #(
        .CLOCKS_PER_SECOND(24'd9) // 10 clocks = 1 second for fast testing
    )
`endif
    user_project (
        
        // Include power ports for the Gate Level test:
`ifdef GL_TEST
        .VPWR(VPWR),
        .VGND(VGND),
`endif

        .ui_in  (ui_in),    // Dedicated inputs
        .uo_out (uo_out),   // Dedicated outputs
        .uio_in (uio_in),   // IOs: Input path
        .uio_out(uio_out),  // IOs: Output path
        .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
        .ena    (ena),      // enable - goes high when design is selected
        .clk    (clk),      // clock
        .rst_n  (rst_n)     // not reset
    );

endmodule
