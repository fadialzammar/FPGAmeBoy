`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/16/2021 09:49:17 PM
// Design Name: 
// Module Name: Memory
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


module ProgCount(
    input PC_CLK,
    input PC_RST,
    input PC_LD,
    input PC_INC,
    input logic [15:0] PC_DIN,
    output logic [15:0] PC_COUNT//= 'h100 // start at 0x100
    );
    
    always_ff @(posedge PC_CLK)
    begin
        if (PC_RST == 1'b1)
            PC_COUNT <= 16'h100;
        else if (PC_LD == 1'b1)
            PC_COUNT <= PC_DIN;
        else if (PC_INC == 1'b1) // Probably just do a feedback accumulator outside the module
            PC_COUNT <= PC_COUNT + 1;
        else
            PC_COUNT <= PC_COUNT;
    end
    
endmodule
