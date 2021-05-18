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
    input [15:0] PROG_ADDR,
    output logic [7:0] PROG_IR
    );
      
    (* rom_style="{distributed | block}" *) // force the ROM to be block memory
     logic [7:0] rom[0:65535];
     
    // initalize the ROM with the prog_rom.mem file
    initial begin

        $readmemh("Tetris_Time.mem", rom);
    end 
    
    always_ff @(posedge PROG_CLK)
    begin
        //if (RE)
            PROG_IR <= rom[PROG_ADDR];
    end
endmodule
