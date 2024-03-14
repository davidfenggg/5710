/* INSERT NAME AND PENNKEY HERE */
// David Feng: dfenggg
// Victoria Zammit: vzammit

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

    wire [31:0] remainder_wires[0:32]; 
    wire [31:0] quotient_wires[0:32];  
    wire [31:0] dividend_wires[0:32];

    assign remainder_wires[0] = 0;
    assign quotient_wires[0] = 0;
    assign dividend_wires[0] = i_dividend;

    // first stage of pipeline
    genvar j;
    generate
        for (j = 0; j < 16; j++) begin : divu_iter
            divu_1iter div_step(
                .i_dividend(dividend_wires[j]), 
                .i_divisor(i_divisor),
                .i_remainder(remainder_wires[j]),
                .i_quotient(quotient_wires[j]),
                .o_dividend(dividend_wires[j+1]),
                .o_remainder(remainder_wires[j + 1]),
                .o_quotient(quotient_wires[j + 1])
            );
        end
    endgenerate

    // second stage of pipeline
    genvar i;
    generate
        for (i = 16; i < 32; i++) begin : divu_iter
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

    logic [31:0] lo_remainder, lo_quotient;
    always @(posedge clk) begin
        if (rst) begin
            lo_remainder <= 0;
            lo_quotient <= 0;
        end else begin
            lo_remainder <= remainder_wires[32];
            lo_quotient <= quotient_wires[32];
        end
    end

    // // Output signals from second stage
    assign o_remainder = lo_remainder;
    assign o_quotient = lo_quotient;

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
