`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2021 04:31:18 PM
// Design Name: 
// Module Name: frame_seq
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

//tested and verified
module frame_seq(
    input clk, //512 Hz timer input
    output logic len_clk = 0, //256 length counter
    output logic env_clk = 0, //64 volume envelope
    output logic sweep_clk = 0 //128 sweep
    );
    
    
    logic [2:0] count = 0;
   
    //clock for 256 Hz
    always_ff @(posedge clk)
    begin
    case (count)
        0:
        begin
            len_clk = 1;
        end
        2:
        begin
            len_clk = 1;
            sweep_clk = 1;
        end
        4:
        begin
            len_clk = 1;
        end
        6:
        begin
            len_clk = 1;
            sweep_clk = 1;
        end
        7:
        begin
            env_clk = 1;
        end
        default:
        begin
            len_clk = 0;
            env_clk = 0;
            sweep_clk = 0;
        end
    endcase
    
    count = count+1;
    
    end
    always_ff @(negedge clk)
    begin
        len_clk = 0;
        env_clk = 0;
        sweep_clk = 0;
    end 
    
endmodule
