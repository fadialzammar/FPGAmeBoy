`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/18/2021 05:45:35 PM
// Design Name: 
// Module Name: CPU_TEST_BENCH
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


module CPU_TEST_BENCH();

     logic CLK=0, RST=0;
     logic [15:0] SWITCHES,LEDS;
     logic [7:0] CATHODES,VGA_RGB;
     logic [3:0] ANODES;
     logic [4:0] counter;
    // Display outputs
     logic VGA_CLK, VGA_HS, VGA_VS, VGA_PIXEL_VALID;
     logic [1:0] VGA_PIXEL;
     
     top DUT (.*);
    
     initial forever  #10  CLK =  ! CLK; 


endmodule
