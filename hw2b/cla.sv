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

   //Lower 4 bits
    gp4 gp4_lower(
        .gin(gin[3:0]),
        .pin(pin[3:0]),
        .cin(cin),
        .gout(gout0),
        .pout(pout0),
        .cout(cout[2:0]) // These are the carry outs for bits 0, 1, 2
    );

    // Upper 45
    gp4 gp4_upper(
        .gin(gin[7:4]),
        .pin(pin[7:4]),
        .cin(carry_out4), // This is the carry in for the upper gp4 block
        .gout(gout1),
        .pout(pout1),
        .cout(cout[6:3]) // These are the carry outs for bits 4, 5, 6
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

   wire carry[31:0]

   // XOR (a, b, carry) -> sum
   // to get carry, we need 4 gp8s
   // each gp8 has g and p inputs
   // we need a for loop to calculate g and p using gp1 for each bit
   // how to fill in missing carries

    // bit by bit xor: a[0] ^ b[0] ^ c[0]

endmodule
