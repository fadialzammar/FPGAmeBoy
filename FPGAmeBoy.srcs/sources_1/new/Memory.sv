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
    input logic [15:0] ADDR, 
    input logic [7:0] DIN,
    output logic [7:0] DOUT,
    logic [7:0] mem [0:65535]

    );
initial begin
    for (int i = 0; i < (1<<16); i++) begin
        mem[i] = 0;
    end
    mem[517] = 8'h2D; // REG A
end
    always_ff@(posedge CLK)
    begin
        if(WE)
            mem[ADDR] <= DIN;
        if(RE)    
            DOUT <= mem[ADDR];
    end
endmodule
