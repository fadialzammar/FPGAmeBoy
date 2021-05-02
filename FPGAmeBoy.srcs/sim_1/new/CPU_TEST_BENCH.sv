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

     logic CLK100=0, RST=0;
     logic PIXEL_CLK, VGA_HS, VGA_VS, PIXEL_VALID;
     // logic [1:0] VGA_PIXEL;
     logic [3:0] VGA_RED, VGA_GREEN, VGA_BLUE;
     
     top DUT (.*);
    
     initial forever  #10  CLK100 =  ! CLK100; 


endmodule
