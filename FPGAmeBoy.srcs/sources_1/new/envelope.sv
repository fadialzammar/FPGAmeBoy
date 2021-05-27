`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/26/2021 04:20:04 PM
// Design Name: 
// Module Name: envelope
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


module envelope(
    input clk, //64Hz
    input [3:0] vol_init, //initial volume
    input vol_dir, //increase = 1 decrease = 0
    input trigger, 
    input [2:0] period, //frame seq clocks
    
    output logic [3:0] volume
    );
    
    
    
    logic [3:0] period_timer;
    logic [3:0] curr_vol;
    logic trig_ind = 0;
    
    always @(posedge trigger)
    begin
        //load period timer on trigger
        period_timer = period;
        //current volume value saved
        curr_vol = vol_init;
        trig_ind = 1'b1;
    end
    
    always @(posedge clk)
    begin
        //period (input) must not be zero for decrementing/incrementing vol
        if(period != 0 && trig_ind != 0)
        begin
            if(period_timer > 0)
            begin
                period_timer = period_timer - 1;
            end
            if(period_timer == 0)
            begin
                period_timer = period;
                if((curr_vol < 15 && vol_dir == 1) || (curr_vol > 0 && vol_dir == 0))
                begin
                    volume = vol_dir ? curr_vol + 1 : curr_vol - 1;
                end
                else //when vol value out of range, turn off envelope
                begin
                    trig_ind = 1'b0;
                end
            end
        end
    end
    
endmodule
