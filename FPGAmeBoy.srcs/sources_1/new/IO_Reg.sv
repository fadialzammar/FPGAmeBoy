`timescale 1ns / 1ps
`default_nettype wire
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/02/2021 10:07:43 PM
// Design Name: 
// Module Name: IO_Reg
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


module IO_Reg(
    input [7:0] ADR,
    input [7:0] D_IN, INT_IN,
    input [2:0] INT_ID,
    input IME, INT_CLR,
    input CLK, WE,
    output logic [7:0] D_OUT, D_IE, D_IF
);

logic [7:0] mem [0:255];

//initialize all memory to zero
initial begin
    for (int i = 0; i < 256; i++) begin
        mem[i] = 0;
    end
end

//create synchronous write to port X
always_ff @ (posedge D_IN)
begin
    if (WE == 1)
        mem[ADR] <= D_IN;
end

// Write Interrupt flags on incoming interrupt signal
always_ff @ (posedge INT_IN)
begin
    mem[15] <= INT_IN;
end
always_ff @ (posedge INT_CLR)
begin
    mem[15][INT_ID] <= 0;
end

//asynchronous read to ports x and y
assign D_OUT = mem[ADR];
// Always outputting interrupt data
assign D_IE  = mem[255];
assign D_IF  = mem[15];
endmodule