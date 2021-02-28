`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/16/2021 10:33:06 PM
// Design Name: 
// Module Name: RegFile
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

//REGISTER ADDRESSES:
//000: B
//001: C
//002: D 
//003: E;
//004: H
//005: L 
//007: A

module RegFile #(parameter ADDR_SIZE = 5, DATA_SIZE = 8)(
    input [ADDR_SIZE-1:0] ADRX, ADRY,
    input [DATA_SIZE-1:0] DIN,
    input CLK, WE,
    output logic [DATA_SIZE-1:0] DX_OUT, DY_OUT
);

logic [DATA_SIZE-1:0] mem [0:(1<<ADDR_SIZE)-1];

//initialize all memory to zero
initial begin
    for (int i = 0; i < (1<<ADDR_SIZE); i++) begin
        mem[i] = 0;
    end
    mem[0] = 8'h18;
    mem[1] = 8'h69;
    mem[7] = 8'h2D;
end

//create synchronous write to port X
always_ff @ (posedge CLK)
begin
    if (WE == 1)
        mem[ADRX] <= DIN;
end

//asynchronous read to ports x and y
assign DX_OUT = mem[ADRX];
assign DY_OUT = mem[ADRY]; 
endmodule

