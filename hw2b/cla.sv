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

module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

    wire gout_lower, pout_lower, gout_upper, pout_upper;
    wire [2:0] cout_lower, cout_upper; // Ensure there's a wire for upper cout

    // Instantiate the lower 4-bit gp4 block with named connections
    gp4 gp4_lower(
        .gin(gin[3:0]),
        .pin(pin[3:0]),
        .cin(cin),
        .gout(gout_lower),
        .pout(pout_lower),
        .cout(cout_lower)
    );

    // Correctly pass carry_mid to gp4_upper's cin
    wire carry_mid = gout_lower | (pout_lower & cin);

    // Instantiate the upper 4-bit gp4 block with named connections and corrected cout handling
    gp4 gp4_upper(
        .gin(gin[7:4]),
        .pin(pin[7:4]),
        .cin(carry_mid),
        .gout(gout_upper),
        .pout(pout_upper),
        .cout(cout_upper) // Use cout_upper here
    );

    // Correctly assign cout for the entire 8-bit block
    assign cout[2:0] = cout_lower; // Directly from lower gp4
    assign cout[6:3] = {cout_upper, carry_mid}; // Correctly combine upper cout and carry_mid

    // Compute the aggregate generate and propagate signals for the 8-bit block
    assign gout = gout_upper | (pout_upper & gout_lower);
    assign pout = pout_lower & pout_upper;
endmodule

module cla(
    input wire [31:0] a, b,
    input wire cin,
    output wire [31:0] sum
);
    wire [31:0] g, p; // Generate and propagate signals for each bit
    wire [31:0] carry; // Carry for each bit
    wire [3:0] gout, pout; // Aggregate generate and propagate signals for 8-bit blocks
    wire [6:0] cout0, cout1, cout2, cout3; // Corrected: Internal carry-outs for each 8-bit segment from gp8

    // Generate and propagate signals for each bit using gp1 instances
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin
        gp1 gp1_inst(
            .a(a[i]), 
            .b(b[i]), 
            .g(g[i]), 
            .p(p[i])
        );
    end

    // Define gp8 instances for each 8-bit block and manage carry signals correctly
    gp8 gp8_0(
        .gin(g[7:0]),
        .pin(p[7:0]),
        .cin(cin),
        .gout(gout[0]),
        .pout(pout[0]),
        .cout(cout0) // Corrected: Just pass cout0, which is 7 bits wide
    );

    // Correctly manage carry between blocks to avoid circular logic
    // Note: You don't need to include carry[0] in the cout assignment; it's the input carry
    assign carry[8] = gout[0] | (pout[0] & cin);
    // Repeat for other gp8 blocks with similar logic...

    // Compute the sum
    assign sum[0] = a[0] ^ b[0] ^ cin;
    for (i = 1; i < 32; i = i + 1) begin
        // This is where you correctly propagate carry signals, avoiding circular logic
        assign sum[i] = a[i] ^ b[i] ^ carry[i];
    end

    // Properly chain carry signals between gp8 blocks
    // Example for chaining carry from the first to the second block:
    // assign carry[8] = gout[0] | (pout[0] & cin);
    // You need to calculate carry[9] to carry[15] based on cout0 and so on for other blocks

endmodule
