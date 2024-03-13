/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned_pipelined (
    input wire clk, rst,
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
// should be something like this except we need to for loop 16 times twice
    // Instantiate pipeline registers
    reg [31:0] remainder_stage1;
    reg [31:0] quotient_stage1;
    reg [31:0] dividend_stage2;
    reg [31:0] remainder_stage2;
    reg [31:0] quotient_stage2;

    // Instantiate first stage divider module
    divu_1iter stage1 (
        .i_dividend(i_dividend),
        .i_divisor(i_divisor),
        .i_remainder(32'b0),  // No remainder input for first stage
        .i_quotient(32'b0),   // No quotient input for first stage
        .o_dividend(dividend_stage2),
        .o_remainder(remainder_stage1),
        .o_quotient(quotient_stage1)
    );

    // Instantiate second stage divider module
    divu_1iter stage2 (
        .i_dividend(dividend_stage2),
        .i_divisor(i_divisor),
        .i_remainder(remainder_stage1),
        .i_quotient(quotient_stage1),
        .o_dividend(i_dividend),  // No output needed for the second stage
        .o_remainder(remainder_stage2),
        .o_quotient(quotient_stage2)
    );

    // Output signals from second stage
    assign o_remainder = remainder_stage2;
    assign o_quotient = quotient_stage2;

endmodule


module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire[31:0] shifted_remainder = (i_remainder << 1) | ((i_dividend >> 31) & 32'b1);

    //if remainder < divisor, then quotient = (quotient << 1)
    //else remainder = remainder - divisor
    assign o_remainder = (shifted_remainder < i_divisor) ? shifted_remainder : shifted_remainder - i_divisor;
    
    //if remainder < divisor, then quotient = (quotient << 1)
    //else quotient = (quotient << 1) | 0x1
    assign o_quotient = (shifted_remainder < i_divisor) ? (i_quotient << 1) : ((i_quotient << 1) | 32'b1);
    
    //update dividend 
    assign o_dividend = i_dividend << 1;

endmodule
