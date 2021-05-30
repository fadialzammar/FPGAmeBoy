`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/26/2021 06:38:17 PM
// Design Name: 
// Module Name: joypad
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


module joypad(
    input [3:0] btn_in,  // Button Inputs
    input [7:0] data_in, // Write Data
    input WE, CLK,       // Write Enable + CLK
    
    output [7:0] reg_out,
    output logic int_ctrl
    );
    
    logic [1:0] col; 
    logic [3:0] rows;
    logic and_rows;
    
    assign reg_out = {2'b11, col, rows};
    assign and_rows = rows[3]&rows[2]&rows[1]&rows[0];
    
    always_ff @(posedge CLK)
    begin
        if(WE == 1)
        begin
            col <= data_in[5:4];
        end
        rows = btn_in;
        if (and_rows == 0)
            int_ctrl <= 1; // Yuh
        else
            int_ctrl <= 0;
    end
 
endmodule
