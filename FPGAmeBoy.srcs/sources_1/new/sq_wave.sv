`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2021 04:38:46 PM
// Design Name: 
// Module Name: sq_wave
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
// Name Addr 7654 3210 Function
//	- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//			Square 1
//	NR10 FF10 -PPP NSSS	Sweep period, negate, shift
//	NR11 FF11 DDLL LLLL	Duty, Length load (64-L)
//	NR12 FF12 VVVV APPP	Starting volume, Envelope add mode, period
//	NR13 FF13 FFFF FFFF	Frequency LSB
//	NR14 FF14 TL-- -FFF	Trigger, Length enable, Frequency MSB
//			Square 2
//	     FF15 ---- ---- Not used
//	NR21 FF16 DDLL LLLL	Duty, Length load (64-L)
//	NR22 FF17 VVVV APPP	Starting volume, Envelope add mode, period
//	NR23 FF18 FFFF FFFF	Frequency LSB
//	NR24 FF19 TL-- -FFF	Trigger, Length enable, Frequency MSB
//  Sweep -> timer -> duty -> length counter -> envelope -> mixer
//////////////////////////////////////////////////////////////////////////////////


module sq_wave(
    input clk,
    input en, 
    
    input [7:0] sweep_data, //NR10 -PPP NSSS
    input [7:0] dc_ld, //NR11 DDLL LLLL
    input [7:0] vol_reg, //NR12 VVVV APPP
    input [7:0] freq_LSB, //NR13 FFFF FFFF
    input [7:0] freq_MSB, //NR14 [3:0] -FFF
    
    output chan1
    

    );
    logic freq_clk;
    logic env_clk;
    logic sweep_clk;
    logic len_clk;
        
    logic [10:0] sweep_freq;
  
    


    logic [10:0] freq_cat;
    assign freq_cat = {freq_MSB[2:0],freq_LSB};
    
    sweep freq_sweep(
    .clk(sweep_clk), .freq(freq_cat),
    .period(sweep_data[6:4]), .trigger(),
    .dir(sweep_data[3]), .shift(sweep_data[2:0]),
    .freq_out(sweep_freq)
    );
    
    frame_seq frame (
    .clk(clk), .len_clk(len_clk),
    .env_clk(env_clk), .sweep_clk(sweep_clk)
    );
    
    freq_timer timer (
    .clk(clk), .en(),
    .freq(sweep_freq), .clk_out(freq_clk)
    );
    
    duty_cycler duty (
    .clk(freq_clk), .dc(dc_ld[7:6]),
    .out()
    );
    
    len_counter len_counter (
    .clk(len_clk), .len_load(dc_ld[5:0]),
    .trigger(), .en(),
    .is_wave(0), .out()
    );
    
    envelope volume (
    .clk(env_clk), .vol_init(vol_reg[7:4]),
    .vol_dir(vol_reg[3]), .period(vol_reg[2:0]),
    .volume()
    );
    
    
    
    
    
    
    
endmodule
