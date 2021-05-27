`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2021 07:47:52 PM
// Design Name: 
// Module Name: sweep
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
// The frequency sweep unit is controlled by NR10 register
// periodically adjusts square 1's freq up or down
//////////////////////////////////////////////////////////////////////////////////

//needs testing
module sweep(
    input clk,
    input [10:0] freq,
    input [2:0] period,
    input trigger,
    input dir,
    input [2:0] shift,
    
    output logic [10:0] freq_out
    );
    
    logic sweep_en = 0;
    logic [11:0] shad_freq = 0;
    logic [11:0] temp_freq = 0;
    logic [3:0] sweep_counter = 0;
    
    
    always_ff @(posedge trigger)
    begin
        shad_freq <= {1'b0, freq};
        sweep_counter <= 0;
        sweep_en <= (period != 0 || shift != 0) ? 1'b1 : 1'b0;
        //might not be finished
    end
    
    always_ff @(posedge clk)
    begin
    
        if(sweep_counter > 0)
        begin
            sweep_counter <= sweep_counter - 1;
        end
    
        if (sweep_counter  == 0)
        begin
            sweep_counter <= (period == 0) ? 4'b1000 : {1'b0, period};
        end
        
        if(sweep_en != 0 && period != 0)
        //calculate new freq
        begin
            temp_freq <= shad_freq >> shift;
            temp_freq <= dir ? (shad_freq - temp_freq) : (shad_freq + temp_freq);
            if(temp_freq[12] == 1) //overflow check
            begin
                sweep_en <= 0;
            end
            freq_out <= temp_freq[11:0];

        end
    
    
    end
    
    
    
endmodule
