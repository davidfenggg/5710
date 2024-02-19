/* Victoria Zammit - vzammit; David Feng - dfenggg */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
    wire [31:0] remainder_wires[0:32]; 
    wire [31:0] quotient_wires[0:32];  
    wire [31:0] dividend_wires[0:32];

    assign remainder_wires[0] = 0;
    assign quotient_wires[0] = 0;
    assign dividend_wires[0] = i_dividend;

    genvar i;
    generate
        for (i = 0; i < 32; i++) begin : divu_iter
            divu_1iter div_step(
                .i_dividend(dividend_wires[i]), 
                .i_divisor(i_divisor),
                .i_remainder(remainder_wires[i]),
                .i_quotient(quotient_wires[i]),
                .o_dividend(dividend_wires[i+1]),
                .o_remainder(remainder_wires[i + 1]),
                .o_quotient(quotient_wires[i + 1])
            );
        end
    endgenerate

    assign o_remainder = remainder_wires[32];
    assign o_quotient = quotient_wires[32];
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
