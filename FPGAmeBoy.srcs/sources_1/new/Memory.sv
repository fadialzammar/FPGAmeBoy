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

//synchronous memory

module Memory(
    input logic CLK, 
    input logic WE, 
    input logic RE, 
    input logic ADDR, 
    input logic DIN,
    output logic DOUT,
    logic [7:0] mem [15:0]

    );
    
    always_ff@(posedge CLK)
    begin
    if(WE)
    mem[ADDR] <= DIN;
    DOUT <= mem[ADDR];
    end
endmodule
