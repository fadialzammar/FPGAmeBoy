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

///synchronous memory




module Memory #(parameter ADDR_SIZE = 16, DATA_SIZE = 8)(
// main RAM 8KByte
//video RAM 8KByte
    input CLK, 
    input WE,
    input RE,
    input [ADDR_SIZE-1:0] ADDR, 
    input [DATA_SIZE-1:0] DIN,
    output logic [DATA_SIZE-1:0] DOUT);
    
    logic [DATA_SIZE-1:0] mem [0:(1<<ADDR_SIZE)-1];

//    (* rom_style="{distributed | block}" *) 
//    initial begin
//        $readmemh("nintendographic.mem", memory, )
     
    initial begin
        for (int i = 0; i < (1<<16); i++) begin
            mem[i] = 0;
         end
//      mem[17733] = 8'h1A; // REG A
    //   mem[515] = 8'hEE;
      mem[258] = 8'hBB;     //0x0102
      mem[515] = 8'hCC;     //0x0203
      mem[772] = 8'hDD;     //0x0304
      mem[1029] = 8'hEE;    //0x0405
      mem[1286] = 8'hAB;    //0x0506
      mem[43783] = 8'hCD;   //0xAB07
      mem[61389] = 8'hAA;   //0xEFCD
    end
  
    always_ff@(posedge CLK)
    begin
    if(WE==1)
    mem[ADDR] <= DIN;
    end
    
    always_ff@(negedge CLK)
    begin
    if(RE == 1)
    DOUT <= mem[ADDR];
    end
    
endmodule