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

//chan 1 done, needs test 
module sq_wave(
    input clk,
    input en, 
    
    input [7:0] NR10, //NR10 -PPP NSSS
    input [7:0] NR11, //NR11 DDLL LLLL
    input [7:0] NR12, //NR12 VVVV APPP
    input [7:0] NR13, //NR13 FFFF FFFF
    input [7:0] NR14, //NR14 [3:0] TL-- -FFF
    
    output [3:0] chan1
    

    );
    logic freq_clk;
    logic env_clk;
    logic sweep_clk;
    logic len_clk;
    
    logic [10:0] freq_cat;        
    logic [10:0] sweep_freq;

    logic dc_out;
    logic len_en_out;
    logic len_dc;
    logic vol_out;
      

    


    assign freq_cat = {NR14[2:0],NR13};
    
    sweep freq_sweep(
    .clk(sweep_clk), .freq(freq_cat),
    .period(NR10[6:4]), .trigger(NR14[7]),
    .dir(NR10[3]), .shift(NR10[2:0]),
    .freq_out(sweep_freq)
    );
    
    frame_seq frame (
    .clk(clk), .len_clk(len_clk),
    .env_clk(env_clk), .sweep_clk(sweep_clk)
    );
    
    freq_timer timer (
    .clk(clk),
    .freq(sweep_freq), .clk_out(freq_clk)
    );
    
    duty_cycler duty (
    .clk(freq_clk), .dc(NR11[7:6]),
    .out(dc_out)
    );
    
    len_counter len_counter (
    .clk(len_clk), .len_load(NR11[5:0]),
    .trigger(NR14[7]), .en(NR14[6]),
    .is_wave(0), .out(len_en_out)
    );
    
    envelope volume (
    .clk(env_clk), .vol_init(NR12[7:4]),
    .vol_dir(NR12[3]), .period(NR12[2:0]),
    .volume(vol_out)
    );
    
    assign len_dc = (dc_out && len_en_out);    
    assign chan1 = (len_dc) ? vol_out : 0;
    
    
    
    
    
    
endmodule
