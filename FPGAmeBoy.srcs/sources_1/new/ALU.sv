`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Izak Walker, Hao Nguyen
// 
// Create Date: 01/23/2021 03:52:38 PM
// Design Name: 
// Module Name: ALU
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

module ALU(ALU_FUN, A, B, FLAGS_IN, ALU_OUT, FLAGS_OUT);
        input [4:0] ALU_FUN;  //func7[5],func3
        input [7:0] A,B;
        input [3:0] FLAGS_IN;
        output logic [7:0] ALU_OUT; 
        output logic [3:0] FLAGS_OUT;
       
       // Local Arithmetic Op-code parameters
       localparam ADD = 5'b00000;
       localparam ADC = 5'b00001;
       
       // Flag Register Bits
       // [Z,N,H,C,0,0,0]
       localparam Z_FLAG = 2'd3;
       localparam N_FLAG = 2'd2;
       localparam H_FLAG = 2'd1;
       localparam C_FLAG = 2'd0;
       
       
       // Upper and lower 4 bit results + 1 bit for Half and Carry Flags
       logic [4:0] HIGH_RESULT;
       logic [4:0] LOW_RESULT;
       
       
        always_comb
        begin //reevaluate If these change
            case(ALU_FUN)
            
                // Concatenation station
                ADD: 
                    begin
                        // Concatenation  of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, A[3:0]} + {1'b0, B[3:0]};
                        // Sets Half-carry flag is there was overflow ito the fifth bit of LOW_RESULT
                        FLAGS_OUT[H_FLAG] = LOW_RESULT[4];
                        // Concatenation  of upper 4 bits of input with 
                        // additional bit for proper LOW_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} + {1'b0, B[7:4]};
                        // Sets Carry flag is there was overflow ito the fifth bit of HIGH_RESULT
                        FLAGS_OUT[C_FLAG] = HIGH_RESULT[4];
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0] + LOW_RESULT[3:0]};
                        
                    end
                    
                // Concatenation station + C
                ADC: 
                    begin
                    // Need to change to same form as ADD but also add carry flag
                        ALU_OUT = A + B + FLAGS_IN[C_FLAG];
                    end
              
                default: 
                    begin
                        ALU_OUT = 0;
                        FLAGS_OUT = FLAGS_IN;
                    end
            endcase
        end
    endmodule