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

    // TODO: your code here

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
