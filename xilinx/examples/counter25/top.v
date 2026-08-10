// 25-bit PRBS (maximal LFSR, x^25+x^3+1) -> tick once per 2^25-1 cycles
// (~6 Hz @200MHz) -> 8-bit Johnson (twisted-ring) counter -> LEDs.
// Fully synchronous to one 200 MHz clock.  No CARRY4 (LFSR=FF+XOR,
// Johnson=shift, tick=compare): critical path is the 25-input compare.
module top (
    input  wire sysclk_p, sysclk_n, rst,
    output wire [7:0] led
);
    wire clk_raw, clk;
    IBUFDS #(.DIFF_TERM("TRUE"), .IBUF_LOW_PWR("FALSE"), .IOSTANDARD("LVDS"))
        ibufds (.I(sysclk_p), .IB(sysclk_n), .O(clk_raw));
    BUFG bufg (.I(clk_raw), .O(clk));

    reg  [24:0] prbs = 25'h1;
    wire        fb   = prbs[24] ^ prbs[2];     // x^25 + x^3 + 1 (primitive)
    wire        tick = (prbs == 25'h1);        // terminal -> once per period
    always @(posedge clk)
        if (rst) prbs <= 25'h1;
        else     prbs <= {prbs[23:0], fb};

    reg [7:0] johnson = 8'h00;
    always @(posedge clk)
        if (rst)       johnson <= 8'h00;
        else if (tick) johnson <= {johnson[6:0], ~johnson[7]};  // twisted ring
    assign led = johnson;
endmodule
