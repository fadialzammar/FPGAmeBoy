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
     
     Wrapper DUT (.*);

    parameter REG_B = 3'b000;
    parameter REG_C = 3'b001;
    parameter REG_D = 3'b010;
    parameter REG_E = 3'b011;
    parameter REG_H = 3'b100;
    parameter REG_L = 3'b101;
    parameter REG_HL = 3'b110;
    parameter REG_A = 3'b111;
    
    initial forever  #10  CLK =  ! CLK; 


endmodule
