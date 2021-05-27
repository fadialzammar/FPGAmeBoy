`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2021 07:37:10 PM
// Design Name: 
// Module Name: FRAME_SEQ_TEST
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


module FRAME_SEQ_TEST();
    
    logic clk = 0;
    logic len_clk;
    logic env_clk;
    logic sweep_clk;
    
    frame_seq fs (.*);
    
    always
    begin
        clk = 0; #5; clk = 1; #5;
    end
endmodule
