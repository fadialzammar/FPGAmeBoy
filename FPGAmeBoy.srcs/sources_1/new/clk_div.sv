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


module clk_div(
    input CLK,
    output logic CLK_DIV=0,
    output logic CLK_DIV_DISP=0
    );

    logic [23:0] clk_div_counter = 24'h000000;
    logic [19:0] clk_div_counter2 = 20'h00000;
    
        // Clock Divider to create 500 Hz refresh from 100 MHz clock
	always_ff @(posedge CLK) begin
        clk_div_counter = clk_div_counter + 1;        
        if (clk_div_counter == 24'hFFFFFF) begin     // x186A0 = 1*10^5 = 1 ms toggle (x30D40)
            clk_div_counter = 24'h0000000;
            CLK_DIV = ~CLK_DIV;         // toggle every 1 ms creates 500 Hz clock
        end
        if ( clk_div_counter == 20'h186A0) begin     // x186A0 = 1*10^5 = 1 ms toggle (x30D40)
            clk_div_counter = 20'h00000;
            CLK_DIV_DISP = ~CLK_DIV_DISP;         // toggle every 1 ms creates 500 Hz clock
        end
    end  
      
endmodule
