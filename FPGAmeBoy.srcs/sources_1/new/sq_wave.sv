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
    input [1:0] dc,
    input [10:0] freq
    
    

    );
    logic freq_clk;
    freq_timer timer(
    .clk(clk), .en(),
    .period(), .clk_out(freq_clk)
    );
    
    
    
    
    
    
endmodule
