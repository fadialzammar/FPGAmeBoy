`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2021 12:52:41 AM
// Design Name: 
// Module Name: clk_div
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


module  C_DIV(
    input CLK,
    output logic CLK_DIV=0
    );

    logic [5:0] clk_div_counter = 6'h00;
    
    // Clock Divider to create 10 MHz refresh from 100 MHz clock
    always_ff @(posedge CLK) 
    begin
        clk_div_counter = clk_div_counter + 1;   
        if (clk_div_counter == 6'd12) 
        begin    
            clk_div_counter = 6'h00;
            CLK_DIV = ~CLK_DIV;    
        end
    end  
      
endmodule