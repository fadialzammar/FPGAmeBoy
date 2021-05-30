`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/30/2021 12:05:34 PM
// Design Name: 
// Module Name: wave
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
//			Wave
//	NR30 FF1A E--- ----	DAC power
//	NR31 FF1B LLLL LLLL	Length load (256-L)
//	NR32 FF1C -VV- ----	Volume code (00=0%, 01=100%, 10=50%, 11=25%)
//	NR33 FF1D FFFF FFFF	Frequency LSB
//	NR34 FF1E TL-- -FFF	Trigger, Length enable, Frequency MSB
//  WAVE RAM: FF30 - FF3F
//  Wave:  Timer -> Wave -> Length Counter -> Volume   -> Mixer
//			Wave Table
//	     FF30 0000 1111	Samples 0 and 1
//	     ....
//	     FF3F 0000 1111	Samples 30 and 31

//////////////////////////////////////////////////////////////////////////////////

//done, needs testing
module wave(
    input clk,
    input DAC_en,
    input [7:0] NR31,
    input [7:0] NR32,
    input [7:0] NR33,
    input [7:0] NR34,
    
    input [15:0][7:0] wave_table,
    
    output logic [3:0]chan3
    );
    
    logic freq_clk;
    logic env_clk;
    logic len_clk;
    
    logic [10:0] freq_cat;
    logic [1:0] vol_shift;
    logic [4:0] pos = 0; //32 samples, sample 0 to sample 31
    logic [15:0] table_pos = 16'hFF30;
    logic [3:0] sample;
    
    //even or odd sample number
    assign sample = (pos[0]) ? wave_table[table_pos][7:4] : wave_table[table_pos][3:0]; 
    
    assign vol_shift = NR32[6:5] - 1;
    
    frame_seq frame (
    .clk(clk), .len_clk(len_clk),
    .env_clk(env_clk), .sweep_clk()
    );
    

    assign freq_cat = {NR34[2:0],NR33};
    
    freq_timer timer (
    .clk(clk),
    .freq(freq_cat), .clk_out(freq_clk)
    );
    
    len_counter#(.DATA_SIZE(8)) len_counter(
    .clk(len_clk), .len_load(NR31[7:0]),
    .trigger(NR34[7]), .en(NR34[6]),
    .is_wave(1), .out(len_en_out)
    );
    
    always_ff@(posedge freq_clk)
    begin
        chan3 = (len_en_out == 0) ? 0 : sample >> vol_shift;
        pos = pos+1;
        table_pos = (pos[0]== 0) ? table_pos + 1 : table_pos;
        table_pos = (table_pos > 16'hFF3F) ? 16'hFF30 : table_pos;
    end
    
    
    
    
    
endmodule
