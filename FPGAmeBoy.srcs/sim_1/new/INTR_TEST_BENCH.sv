`timescale 1ns / 1ns
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2021 03:53:35 PM
// Design Name: 
// Module Name: INTR_in_TEST_BENCH
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


module INTR_TEST_BENCH();

     logic CLK=0, RST=0;
     logic [15:0] SWITCHES,LEDS;
     logic [7:0] CATHODES,VGA_RGB;
     logic [3:0] ANODES;
     logic [4:0] counter;
     logic [2:0] INTR_in = 0;
     logic VGA_HS, VGA_VS;
     logic [3:0] VGA_RED, VGA_GREEN, VGA_BLUE;
     
     top DUT (.*);
    
     initial forever  #10  CLK =  ! CLK; 
     
     initial begin
        RST = 1;
        INTR_in = 0;
        #50
        RST = 0;
     end
endmodule