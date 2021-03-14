`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/01/2021 12:10:01 AM
// Design Name: 
// Module Name: Stack_Pointer
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


module Stack_Pointer(
    input SP_LD,
    input SP_INCR,
    input SP_DECR,
    input RST,
    input CLK,
    input [15:0] DIN,
    output logic [15:0] DOUT = 16'hFFFE
    );
        
        //Synchronous register logic
        always_ff @(posedge CLK) begin           
            
            if (RST == 1)			//Reset to 0xFFFE (top of memory)
                DOUT <= 16'hFFFE;                
            else if(SP_LD == 1)		//Loads input data               
                DOUT <= DIN;
            else if(SP_INCR == 1) 	//Increments output by one byte
                DOUT <= DOUT + 16'h0001;
            else if(SP_DECR == 1) 	//decrements output by one byte
                DOUT <= DOUT - 16'h0001;
            else
                DOUT <= DOUT; 		//Hold output otherwise
            end  
    endmodule

