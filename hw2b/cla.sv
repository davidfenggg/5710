`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

    assign cout[0] = gin[0] | (pin[0] & cin);
    assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
    assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

    assign pout = (pin[0] & pin[1]) & (pin[2] & pin[3]);
    assign gout = (((((gin[0] & pin[1]) | gin[1]) & pin[2]) | gin[2]) & pin[3]) | gin[3];
    
endmodule

module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

    wire gout_lower, pout_lower, gout_upper, pout_upper;
    wire [2:0] cout_lower, cout_upper; 

    gp4 gp4_lower(
        .gin(gin[3:0]),
        .pin(pin[3:0]),
        .cin(cin),
        .gout(gout_lower),
        .pout(pout_lower),
        .cout(cout_lower)
    );

    wire carry_mid = gout_lower | (pout_lower & cin);

    gp4 gp4_upper(
        .gin(gin[7:4]),
        .pin(pin[7:4]),
        .cin(carry_mid),
        .gout(gout_upper),
        .pout(pout_upper),
        .cout(cout_upper) 
    );

    assign cout[2:0] = cout_lower; 
    assign cout[6:3] = {cout_upper, carry_mid}; 

    assign gout = gout_upper | (pout_upper & gout_lower);
    assign pout = pout_lower & pout_upper;
endmodule

module cla(
    input wire [31:0] a, b,
    input wire cin,
    output wire [31:0] sum
);
    wire [31:0] g, p; 
    wire [31:0] carry; 
    wire [3:0] gout, pout; 
    wire [6:0] cout0, cout1, cout2, cout3; 

    genvar i;
    for (i = 0; i < 32; i = i + 1) begin
        gp1 gp1_inst(
            .a(a[i]), 
            .b(b[i]), 
            .g(g[i]), 
            .p(p[i])
        );
    end

    gp8 gp8_0(
        .gin(g[7:0]),
        .pin(p[7:0]),
        .cin(cin),
        .gout(gout[0]),
        .pout(pout[0]),
        .cout(cout0)
    );

    assign carry[8] = gout[0] | (pout[0] & cin);

    gp8 gp8_1(
        .gin(g[15:8]),
        .pin(p[15:8]),
        .cin(carry[8]),
        .gout(gout[1]),
        .pout(pout[1]),
        .cout(cout1)
    );

     assign carry[16] = gout[1] | (pout[1] & (gout[0] | (pout[0] & cin)));

     gp8 gp8_2(
        .gin(g[23:16]),
        .pin(p[23:16]),
        .cin(carry[16]),
        .gout(gout[2]),
        .pout(pout[2]),
        .cout(cout2)
    );

     assign carry[24] = gout[2] | (pout[2] & (gout[1] | (pout[1] & (gout[0] | (pout[0] & cin)))));

     gp8 gp8_3(
        .gin(g[31:24]),
        .pin(p[31:24]),
        .cin(carry[24]),
        .gout(gout[3]),
        .pout(pout[3]),
        .cout(cout3)
    );

    assign carry[0] = cin;
    assign carry[7:1] = cout0;
    assign carry[15:9] = cout1;
    assign carry[23:17] = cout2;
    assign carry[31:25] = cout3;

    assign sum = a ^ b ^ carry;

endmodule
