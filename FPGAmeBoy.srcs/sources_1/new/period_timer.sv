`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/29/2021 11:05:59 PM
// Design Name: 
// Module Name: period_timer
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


module period_timer(
    input clk,
    input [6:0] period,
    output logic clk_out

    );
    
    logic [6:0] count = 0;
    
    always_ff @(posedge clk)
    begin
        clk_out <= (count == period) ? 1 : 0;
        count <= (count < period) ? count + 1 : 7'b0; 
    end
    
    always @(negedge clk)
    begin
        clk_out = 0;
    end
    
endmodule
