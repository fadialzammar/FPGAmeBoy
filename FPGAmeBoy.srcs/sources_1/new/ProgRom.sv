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
    input RE,
    input [15:0] PROG_ADDR,
    input [20:0] MBC_ADDR,
    output logic [7:0] PROG_IR,
    
    input [15:0] CART_ADDR,
    input CART_RE,
    output logic [7:0] CART_DATA,
    
    output logic [2:0] RAM_BANKS,
    output logic [7:0] ROM_BANKS
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
            //PROG_IR <= rom[PROG_ADDR];
            PROG_IR <= rom[MBC_ADDR];
            CART_DATA <= rom[CART_ADDR];

    end
    
    always_comb 
    begin
    //check ROM size    
        if ( PROG_ADDR == 328 ) begin // 148 in hex
            if( rom[PROG_ADDR] == 0 )
                ROM_BANKS = 0; // 32kB, no ROM banking
            else if ( rom[PROG_ADDR] == 1 )
                ROM_BANKS = 4; // 64kB, 4 banks
            else if ( rom[PROG_ADDR] == 2 )
                ROM_BANKS = 8; // 128kB, 8 banks
            else if ( rom[PROG_ADDR] == 3 )
                ROM_BANKS = 16; // 256kB, 16 banks
            else if ( rom[PROG_ADDR] == 4 )
                ROM_BANKS = 32; // 512kB, 32 banks
            else if ( rom[PROG_ADDR] == 5 )
                ROM_BANKS = 64; // 1MB, 64 banks
            else if ( rom[PROG_ADDR] == 6 )
                ROM_BANKS = 128; // ~2MB, 128 banks
        end
        //check RAM size
        if ( PROG_ADDR == 329 ) begin // 149 in hex
            if( rom[PROG_ADDR] == 0 )
                RAM_BANKS = 0; // no RAM
            else if ( rom[PROG_ADDR] == 1 )
                RAM_BANKS = 0; // 2kB - unofficial
            else if ( rom[PROG_ADDR] == 2 )
                RAM_BANKS = 1; // 8 kB, 1 bank
            else if ( rom[PROG_ADDR] == 3 )
                RAM_BANKS = 4; // 32kB, 4 banks of 8kB each
        end
    end
endmodule
