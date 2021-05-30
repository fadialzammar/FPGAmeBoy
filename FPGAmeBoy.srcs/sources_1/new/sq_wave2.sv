`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/30/2021 11:52:56 AM
// Design Name: 
// Module Name: sq_wave2
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
// 
//////////////////////////////////////////////////////////////////////////////////

//done needs testing
module sq_wave2(
    input clk,
    
    input [7:0] NR16, //NR16 DDLL LLLL
    input [7:0] NR17, //NR17 VVVV APPP
    input [7:0] NR18, //NR18 FFFF FFFF
    input [7:0] NR19, //NR19 [3:0] TL-- -FFF
    
    output [3:0] chan2
    );
    





    logic freq_clk;
    logic env_clk;
    logic len_clk;
    
    logic [10:0] freq_cat;

    logic dc_out;
    logic len_en_out;
    logic len_dc;
    logic vol_out;
      

    


    assign freq_cat = {NR19[2:0],NR18};
    
    
    frame_seq frame (
    .clk(clk), .len_clk(len_clk),
    .env_clk(env_clk), .sweep_clk()
    );
    
    freq_timer timer (
    .clk(clk),
    .freq(freq_cat), .clk_out(freq_clk)
    );
    
    duty_cycler duty (
    .clk(freq_clk), .dc(NR16[7:6]),
    .out(dc_out)
    );
    
    len_counter len_counter (
    .clk(len_clk), .len_load(NR16[5:0]),
    .trigger(NR19[7]), .en(NR19[6]),
    .is_wave(0), .out(len_en_out)
    );
    
    envelope volume (
    .clk(env_clk), .vol_init(NR17[7:4]),
    .vol_dir(NR17[3]), .period(NR17[2:0]),
    .volume(vol_out)
    );
    
    assign len_dc = (dc_out && len_en_out);    
    assign chan2 = (len_dc) ? vol_out : 0;
endmodule
