`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/29/2021 01:02:32 PM
// Design Name: 
// Module Name: noise
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:

//NR41 FF20 --LL LLLL Length load (64-L)
//NR42 FF21 VVVV APPP Starting volume, Envelope add mode, period
//NR43 FF22 SSSS WDDD Clock shift, Width mode of LFSR, Divisor code
//NR44 FF23 TL-- ---- Trigger, Length enable


// frequency timer period is set by a divisor shifted left some number of bits
// the frequency timer clocks the LFSR: a 15 bit shift register with feedback
// on a clock: bit[0], bit[1] XORed, all bits are shifted right by 1
// the result from XOR is set as the high bit.
// width mode enabled: XOR result is a also put into bit 6 after the shift, 
// resulting in a 7-bit LFSR. Wave output is bit 0 of the LFSR, inverted.
//////////////////////////////////////////////////////////////////////////////////

//done with noise, needs testing
module noise(
    input clk, len_clk, env_clk,
    input [7:0] NR41,
    input [7:0] NR42,
    input [7:0] NR43,
    input [7:0] NR44,
    
    output [3:0] noise_out
    );
    
    logic width_mode;
    logic [14:0] shift_reg = 15'b1;
    logic clk_shift;
    logic [2:0] divisor_code = NR43[2:0];
    logic [6:0] period; //8 - 112
    logic [3:0] vol_out;
    
    logic en_out;
    
    logic xor_result;
    
    assign width_mode = NR43[3];
    assign period = divisor_code == 0 ? 8 : (16 * divisor_code);  
    
    period_timer timer(
    .clk(clk), .period(period),
    .clk_out(clk_shift)
    );
    
    len_counter len(
    .clk(len_clk), .len_load(NR41[5:0]),
    .trigger(NR44[7]), .en(NR44[6]),
    .is_wave(0), .out(en_out)
    );
    
    envelope vol(
    .clk(env_clk), .vol_init(NR42[7:4]),
    .vol_dir(NR42[3]), .trigger(NR44[7]),
    .period(NR42[2:0]), .volume(vol_out)
    );
    
    
    always_ff @(posedge NR44[7]) //trigger
    begin
        shift_reg <= 15'b1;
    end
    
    always_ff @(posedge clk_shift)
    begin
        xor_result = shift_reg[0] ^ shift_reg[1];
        shift_reg = {xor_result, shift_reg >> 1};
        if(width_mode) shift_reg[6] = xor_result;
    end
    
    assign noise_out = (en_out && !shift_reg[0]) ? vol_out : 0;
    
endmodule
