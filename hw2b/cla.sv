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

    assign cout[0] = (pin[0] & cin) | gin[0];
    assign cout[1] = (pin[1] & (pin[0] & cin) | gin[0]) | gin[1];
    assign cout[2] = (pin[2] & (pin[1] & (pin[0] & cin) | gin[0]) | gin[1]) | gin[2];

    assign pout = (pin[0] & pin[1]) & (pin[2] & pin[3]);
    assign gout = (((((gin[0] & pin[1]) | gin[1]) & pin[2]) | gin[2]) & pin[3]) | gin[3];
    
endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   wire gout0, pout0, gout1, pout1;
    wire carry_out4;

   //Lower 4 bits
    gp4 gp4_lower(
        .gin(gin[3:0]),
        .pin(pin[3:0]),
        .cin(cin),
        .gout(gout0),
        .pout(pout0),
        .cout(cout[2:0]) // These are the carry outs for bits 0, 1, 2
    );

    assign carry_out4 = gout0 | (pout0 & cin);

    // Upper 45
    gp4 gp4_upper(
        .gin(gin[7:4]),
        .pin(pin[7:4]),
        .cin(carry_out4), // This is the carry in for the upper gp4 block
        .gout(gout1),
        .pout(pout1),
        .cout(cout[6:4]) // These are the carry outs for bits 4, 5, 6
    );

    // Compute the carry in for the upper 4 bits using the lower 4 bits
    assign cout[3] = gout0 | (pout0 & cin);

    // The overall gout and pout for the 8-bit block
    assign gout = gout1 | (pout1 & gout0);
    assign pout = pout0 & pout1;

endmodule

module cla
  (input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

   // XOR (a, b, carry) -> sum
   // to get carry, we need 4 gp8s
   // each gp8 has g and p inputs
   // we need a for loop to calculate g and p using gp1 for each bit
   // how to fill in missing carries

   // bit by bit xor: a[0] ^ b[0] ^ c[0]


   wire [31:0] g, p; // Generate and propagate signals for each bit
    wire [31:0] carry; // Carry for each bit
    wire [3:0] gout, pout; // Output generate and propagate signals for each 8-bit block
    wire [7:0] cout0, cout1, cout2; // Internal carry-outs for gp8 modules

    // Generate generate and propagate signals for each bit
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gp_gen
            gp1 gp1_inst(.a(a[i]), .b(b[i]), .g(g[i]), .p(p[i]));
        end
    endgenerate


    // First 8-bit block
    gp8 gp8_0(
        .gin(g[7:0]),
        .pin(p[7:0]),
        .cin(cin),
        .gout(gout[0]),
        .pout(pout[0]),
        .cout({cout0[6:0], carry[0]})
    );

    // Second 8-bit block
    gp8 gp8_1(
        .gin(g[15:8]),
        .pin(p[15:8]),
        .cin(carry[8]),
        .gout(gout[1]),
        .pout(pout[1]),
        .cout({cout1[6:0], carry[8]})
    );

    // Third 8-bit block
    gp8 gp8_2(
        .gin(g[23:16]),
        .pin(p[23:16]),
        .cin(carry[16]),
        .gout(gout[2]),
        .pout(pout[2]),
        .cout({cout2[6:0], carry[16]})
    );

    // Fourth 8-bit block
    gp8 gp8_3(
        .gin(g[31:24]),
        .pin(p[31:24]),
        .cin(carry[24]),
        .gout(gout[3]),
        .pout(pout[3]),
        .cout(carry[31:24])
    );

    // Compute the sum for each bit
    assign sum = a ^ b ^ carry;

    // Connect carry chain between gp8 modules
    assign carry[8] = gout[0] | (pout[0] & cin);
    assign carry[16] = gout[1] | (pout[1] & carry[8]);
    assign carry[24] = gout[2] | (pout[2] & carry[16]);

endmodule
