`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/17/2021 09:49:31 PM
// Design Name: 
// Module Name: ProgRom
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


module ProgRom(
    input PROG_CLK,
    input [9:0] PROG_ADDR,
    output logic [17:0] PROG_IR
    );
      
    (* rom_style="{distributed | block}" *) // force the ROM to be block memory
     logic [17:0] rom[0:1023];
     
    // initalize the ROM with the prog_rom.mem file
    initial begin
        $readmemh("prog_rom.mem", rom, 0, 1023);
    end 
    
    always_ff @(posedge PROG_CLK)
    begin
        PROG_IR <= rom[PROG_ADDR];
    end
endmodule
