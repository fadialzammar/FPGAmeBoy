`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2021 04:44:05 PM
// Design Name: 
// Module Name: freq_timer
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
// Timer for all 4 channels
//////////////////////////////////////////////////////////////////////////////////


module freq_timer(
    input clk,
    input en,
    input [19:0] period,
    output logic clk_out = 0
    );
    
    logic [19:0] count = 0;
    
    
    always_ff @ (posedge clk)
    begin
        if(en == 0)
        begin
            clk_out <= 0;
        end
        if(en == 1)
        begin
        
            count = count + 1;
            if (count >= period)
            begin
                count <= 0;
                clk_out <= ~clk_out;
            end
        end
    end
endmodule
