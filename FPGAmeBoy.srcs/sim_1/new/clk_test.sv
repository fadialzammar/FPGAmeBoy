`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2021 04:12:04 AM
// Design Name: 
// Module Name: clk_test
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


module clk_test(    );
    logic CLK = 0;
    initial forever  #10  CLK =  ! CLK;
    // CLK DIV
    clk_div CLK_DIVIDER(
        .CLK(CLK),
        .CLK_DIV(CLK_DIV),
        .CLK_DIV_DISP(CLK_DIV_DISP)
    );
endmodule
