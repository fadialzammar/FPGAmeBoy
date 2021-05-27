`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/19/2021 09:18:47 PM
// Design Name: 
// Module Name: len_counter
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

//needs testing
module len_counter #(parameter DATA_SIZE = 6)(
    input clk, //clocked at 256Hz
    input [DATA_SIZE - 1:0] len_load, //256 for wave channel, 64 for others. Can be reloaded at any time
    input trigger,
    input en, //len enable: bit 6 of NRX4
    input is_wave,
    output logic out
    
    );
    logic [8:0] count_256 = 0;
    logic [6:0] count_64 = 0;
    
    logic en_flag = 0;
    
    //trigger reloads counter to 256 or 64 if counter is 0
    always_ff @(posedge trigger)
    begin
        if(is_wave)
        begin
            if(count_256 == 0)
            count_256 = 9'b100000000;
        end
        else
        begin
            if(count_64 == 0)
            count_64 = 7'b1000000;
        end
    end
    
    //turns on internal enable flag, loads counter with 256 or 64 minus data
    always_ff @(posedge en)
    begin
    en_flag = 1;
        if(is_wave)
        begin
            if(count_256 == 0)
            begin
                count_256 <= 9'b10000000 - len_load;
            end
            else
            begin
                count_64 = 7'b1000000 - len_load;
            end
        end
    end
    
    always_ff @ (posedge clk)
    begin
        if(is_wave)
        begin
            en_flag <= (count_256 == 0)? 1'b0 : 1'b1;
            if(en_flag && (count_256 > 0))
            begin
                out <= 1;
                count_256--;
            end
        end
        
        else
        begin
            en_flag <= (count_64 == 0)? 1'b0 : 1'b1;
            if(en_flag && (count_64 > 0))
            begin
                out <= 1;
                count_64--;
            end
        end
        
    end
endmodule
