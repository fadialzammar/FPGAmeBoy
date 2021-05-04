`timescale 1ns / 1ps
`default_nettype wire
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2021 02:48:46 PM
// Design Name: 
// Module Name: interrupt_handler
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


module interrupt_handler(
    input [7:0] D_IE, D_IF,
    input IME,
    output logic INTR,
    output logic [2:0] INTR_ID
    );
    logic [7:0] interrupts;
    assign interrupts = D_IE && D_IF;
    always@(*)
    begin
    
    if(IME == 1)
    begin
        if(interrupts[0] == 1)
        begin
            INTR = 1;
            INTR_ID = 0;
        end
        else if(interrupts[1])      // V-Blank
        begin
            INTR = 1;
            INTR_ID = 1;
        end
        else if(interrupts[2])      // Timer
        begin
            INTR = 1;
            INTR_ID = 2;
        end
        else if(interrupts[3])       // Serial IO Transfer
        begin
            INTR = 1;
            INTR_ID = 3;
        end
       else if(interrupts[4])    // Controller Input
        begin
            INTR = 1;
            INTR_ID = 4;
        end
    end  
    else
        INTR = 0; 
    end 
endmodule
