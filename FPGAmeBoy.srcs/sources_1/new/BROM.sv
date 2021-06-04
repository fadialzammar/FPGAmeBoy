`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2021 05:15:55 PM
// Design Name: 
// Module Name: BROM
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


module BROM(
    input CLK,
    input [15:0] BROM_ADDR,
    input [15:0] PROG_COUNT,
    
    output logic [7:0] BROM_IR,
    output logic [7:0] BROM_DATA
    );
      
    (* rom_style="{distributed | block}" *) // force the ROM to be block memory
     logic [7:0] rom[0:255];
     
    // initalize the ROM with the BOOTROM.mem file
    initial begin
        $readmemh("BOOTROM.mem", rom);
    end 
    
    always_ff @(posedge CLK)
        begin
            BROM_IR <= rom[PROG_COUNT];
        end
        
     always_ff @(negedge CLK)
        begin
            BROM_DATA <= rom[BROM_ADDR];
        end
endmodule
