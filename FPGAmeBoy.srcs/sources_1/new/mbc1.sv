`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2021 07:24:00 PM
// Design Name: 
// Module Name: mbc1
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


module mbc1(
    input CLK,
    input [15:0] ADDR,
    input [7:0] DATA,
    input WR,
    input RD,
    input RST,
    input [7:0] ROM_BANKS,
    input [2:0] RAM_BANKS,
    
    output logic [20:0] MBC_ADDR,
    output logic [7:0] MBC_DATA
    );
    
    
    //logic m_CurrentROMBank = 1;
    logic [6:0] bank_num; //selected Rom bank = bank_num[6:5] + bank_num[4:0]
    logic [1:0] ram_bank;
    logic banking_mode = 0; // default 0 ROM banking, set to 1 for RAM banking
    logic ram_en ;
    //reg [ram 
    
    always_ff @(posedge CLK)
    begin
        if(RST) begin
            ram_en <= 0;
            bank_num <= 7'b0;
            ram_bank <= 2'b0;
            banking_mode <= 0; 
        end 
       
    end
    
    always_comb
    begin
        MBC_DATA = DATA;
        // access to bank 00, 20, 40 and 60 is not allowed
        // automatically increment bank_num by 1 
        if ( bank_num == 7'h00 || bank_num == 7'h20 || bank_num == 7'h40 || bank_num == 7'h60 )
            bank_num += 1;
            
        if( RD ) begin
            if( ADDR < 16'h4000 ) begin //READ ROM Bank 0 or 20,40 & 60 when baking mode is set to 1
                if(banking_mode)
                    MBC_ADDR = ADDR + (bank_num[6:5]<<19);//Bank 0, 20, 40, 60
                    // 
                else 
                    MBC_ADDR = {5'b0,ADDR}; // Bank 0 only
            end
            else if ( ADDR >= 16'h4000 && ADDR < 16'h8000 ) begin //Read ROM bank from 01-7F, each has 16kB
                MBC_ADDR = (ADDR-16'h4000)  + (bank_num[6:0] << 14)  ;
            end
            else if ( ADDR >= 16'hA000 && ADDR < 16'hC000 && ram_en ) begin //Read Ram bank 00-03 if any
                MBC_ADDR = ram_bank[1:0]<<16 + ADDR;
                
            end
        end
        
        if( WR ) begin
            if( ADDR < 16'h2000 ) // enable RAM
                ram_en = (DATA[3:0] == 4'hA) ? 1'b1 : 1'b0;
            else if( ADDR >= 16'h2000 && ADDR < 16'h4000) //select ROM bank number from $01-$1F
                bank_num[4:0] = DATA[4:0];
            else if( ADDR >= 16'h4000 && ADDR < 16'h6000) begin
                if(banking_mode && ram_en) // select RAM bank
                    ram_bank[1:0] = DATA[1:0];
                else    //select 2 upper bits of ROM bank
                    bank_num[6:5] = DATA[1:0];
            end
            else if( ADDR >= 16'h6000 && ADDR < 16'h8000) begin
                if( ROM_BANKS > 8'd16 || RAM_BANKS > 3'd1) 
                    banking_mode = 1;
                //else if ( )
                    //banking_mode=1;
                else banking_mode = 0;  
            end
        end
    end
        
endmodule
