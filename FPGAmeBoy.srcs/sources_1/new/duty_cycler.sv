`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2021 08:35:09 PM
// Design Name: 
// Module Name: duty_cycler
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

//tested and verified, 2 cycles to start
module duty_cycler(
    input clk,
    input [2:0] dc,
    output logic out = 0
    );
    
    logic [7:0] waveform;
    logic [7:0] counter = 0;
 
    
    always_comb 
    begin
        case (dc)
        2'b00:
            begin
            waveform = 8'b00000001;
            end
        2'b01:
            begin
            waveform = 8'b10000001;
            end
        2'b10:
            begin
            waveform = 8'b10000111;
            end
        2'b11:
            begin
            waveform = 8'b01111110;
            end
        endcase
    end
   
    logic [7:0] temp = 0; 
    
    
    always_ff @ (posedge clk)
    begin
        if(counter >= 7)
        begin
            counter <= 0;
            temp <= waveform;
        end
        
        temp <= (waveform >> counter);
        out <= temp[0];
        counter = counter + 1;
    end
    
    
endmodule
