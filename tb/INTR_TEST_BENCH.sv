`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2021 03:53:35 PM
// Design Name: 
// Module Name: INTR_TEST_BENCH
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
     logic [4:0] INTR = 0;
     
     top DUT (.*);
    
     initial forever  #10  CLK =  ! CLK; 
     
     initial begin
        INTR = 1;
        #5
        INTR = 0;
        #25
        INTR = 3;
        #5
        INTR = 0;
        #50
        INTR = 2;
        #100
        INTR = 0;
        #300
        INTR = 2;
        #315
        INTR = 0;
        #40
        INTR = 2;
        #10
        INTR = 0;
     end
endmodule