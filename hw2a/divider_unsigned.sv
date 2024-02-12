/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
    // for (0 to 31)
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
    /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */
    wire [31:0] shifted_remainder;

    always_comb begin
        shifted_remainder = {i_remainder << 1} | (i_dividend >> 31);
        if(shifted_remainder < i_divisor) begin
            o_quotient = (i_quotient << 1);
            o_remainder = shifted_remainder;
        end
        else begin
            o_quotient = (i_quotient << 1) | 4'b1;
            o_remainder = shifted_remainder - i_divisor;
        end
        assign o_dividend = i_dividend << 1;
    end

endmodule
