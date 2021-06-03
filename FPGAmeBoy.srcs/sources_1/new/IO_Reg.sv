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
    input INT_CLR,
    input CLK, WE,

    output logic [7:0] D_OUT, D_IE, D_IF, 
    output logic BROM_DI
);

logic [7:0] mem [0:255];

//initialize all memory to zero
initial begin
    for (int i = 0; i < 256; i++) begin
        mem[i] = 0;
    end
    mem['h05] = 'h00;
    mem['h06] = 'h00;
    mem['h07] = 'h00;
    mem['h10] = 'h80;
    mem['h11] = 'hBF;
    mem['h12] = 'hF3;
    mem['h14] = 'hBF;
    mem['h16] = 'h3F;
    mem['h17] = 'h00;
    mem['h19] = 'hBF;
    mem['h1A] = 'h7F;
    mem['h1B] = 'hFF;
    mem['h1C] = 'h9F;
    mem['h1E] = 'hBF;
    mem['h20] = 'hFF;
    mem['h21] = 'h00;
    mem['h22] = 'h00;
    mem['h23] = 'hBF;
    mem['h24] = 'h77;
    mem['h25] = 'hF3;
    mem['h26] = 'hF1;
    mem['h40] = 'h91;
    mem['hFF] = 'h00;
end

//create synchronous write to port X
always_ff @ (posedge CLK)
begin
    if (WE == 1)
        mem[ADR] <= D_IN;
    if(INT_CLR)
        mem[15][INT_ID] <= 0;
    else
        mem[15] <= INT_IN | mem[15];
end


//always_ff @ (posedge INT_CLR)
//begin
//    mem[15][INT_ID] <= 0;
//end

//asynchronous read to ports x and y
assign D_OUT = mem[ADR];

// Always outputting data
assign D_IE  = mem[255];
assign D_IF  = mem[15];

assign BROM_DI = mem[80][0];

endmodule