`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/14/2021 09:10:18 PM
// Design Name: 
// Module Name: dc_test
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


module dc_test();
    logic clk = 0;
    logic [2:0] dc;
    logic out;
    
    duty_cycler duty (.*);
    
    always
    begin
        clk = 0; #5; clk = 1; #5;
    end
     initial 
     begin 
         dc  = 2'b00;
         #100
         dc = 2'b01;
         #100
         dc = 2'b10;
         #100
         dc = 2'b11;
     end
endmodule
