`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2021 04:15:42 PM
// Design Name: 
// Module Name: Sound_Controller
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
//-----------------------------------------------------------------
//       Square 1
//NR10 FF10 -PPP NSSS Sweep period, negate, shift
//NR11 FF11 DDLL LLLL Duty, Length load (64-L)
//NR12 FF12 VVVV APPP Starting volume, Envelope add mode, period
//NR13 FF13 FFFF FFFF Frequency LSB
//NR14 FF14 TL-- -FFF Trigger, Length enable, Frequency MSB

//       Square 2
//     FF15 ---- ---- Not used
//NR21 FF16 DDLL LLLL Duty, Length load (64-L)
//NR22 FF17 VVVV APPP Starting volume, Envelope add mode, period
//NR23 FF18 FFFF FFFF Frequency LSB
//NR24 FF19 TL-- -FFF Trigger, Length enable, Frequency MSB

//       Wave
//NR30 FF1A E--- ---- DAC power
//NR31 FF1B LLLL LLLL Length load (256-L)
//NR32 FF1C -VV- ---- Volume code (00=0%, 01=100%, 10=50%, 11=25%)
//NR33 FF1D FFFF FFFF Frequency LSB
//NR34 FF1E TL-- -FFF Trigger, Length enable, Frequency MSB

//       Noise
//     FF1F ---- ---- Not used
//NR41 FF20 --LL LLLL Length load (64-L)
//NR42 FF21 VVVV APPP Starting volume, Envelope add mode, period
//NR43 FF22 SSSS WDDD Clock shift, Width mode of LFSR, Divisor code
//NR44 FF23 TL-- ---- Trigger, Length enable

//       Control/Status
//NR50 FF24 ALLL BRRR Vin L enable, Left vol, Vin R enable, Right vol
//NR51 FF25 NW21 NW21 Left enables, Right enables
//NR52 FF26 P--- NW21 Power control/status, Channel length statuses

//       Not used
//     FF27 ---- ----
//     .... ---- ----
//     FF2F ---- ----

//       Wave Table
//     FF30 0000 1111 Samples 0 and 1
//     ....
//     FF3F 0000 1111 Samples 30 and 31
//////////////////////////////////////////////////////////////////////////////////


module Sound_Controller(
    //sq wave 1
    input [7:0] NR10,
    input [7:0] NR11,
    input [7:0] NR12,
    input [7:0] NR13,
    input [7:0] NR14,
    
    //sq wave 2
    input [7:0] NR21,
    input [7:0] NR22,
    input [7:0] NR23,
    input [7:0] NR24,
    
    //wave
    input [7:0] NR30,
    input [7:0] NR31,
    input [7:0] NR32,
    input [7:0] NR33,
    input [7:0] NR34,
    
    //noise
    input [7:0] NR41,
    input [7:0] NR42,
    input [7:0] NR43,
    input [7:0] NR44,
    
    //control/status
    input [7:0] NR50,
    input [7:0] NR51,
    input [7:0] NR52,
    
    //wave table
    input [7:0] sample_01,
    input [7:0] sample_3031,
    
    output [3:0] CH_1,
    output [3:0] CH_2,
    output [3:0] CH_3,
    output [3:0] CH_4

    );
endmodule
