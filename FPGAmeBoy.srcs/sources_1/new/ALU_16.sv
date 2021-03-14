`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/01/2021 08:52:56 PM
// Design Name: 
// Module Name: ALU_16
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


module ALU_16(ALU_FUN, A, B, FLAGS_IN, ALU_OUT, FLAGS_OUT);
        input [1:0] ALU_FUN;
        input [15:0] A,B;
        input [3:0] FLAGS_IN;
        output logic [15:0] ALU_OUT; 
        output logic [3:0] FLAGS_OUT;
       
       // Local Arithmetic Op-code parameters
       localparam ADD  = 2'b00;
       localparam INC  = 2'b01;
       localparam DEC  = 2'b10;  

       
       // Flag Register Bits
       // [Z,N,H,C,0,0,0]
       localparam Z_FLAG = 2'd3;
       localparam N_FLAG = 2'd2;
       localparam H_FLAG = 2'd1;
       localparam C_FLAG = 2'd0;
       
       
       // Upper 4 bit and lower 12 bit results + 1 bit for Half and Carry Flags
       logic [4:0] HIGH_RESULT;
       logic [12:0] LOW_RESULT;
       logic [8:0] TEMP_RESULT; 
       
        always_comb
        begin //reevaluate If these change
            case(ALU_FUN)
            
                // Concatenation station
                ADD: 
                    begin
                        // Concatenation  of lower 12 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, A[11:0]} + {1'b0, B[11:0]};
                        // Sets Half-carry flag if there was overflow in to the MSB of LOW_RESULT
                        FLAGS_OUT[H_FLAG] = LOW_RESULT[12];
                        // Concatenation  of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[15:12]} + {1'b0, B[15:12]} + {4'b0, LOW_RESULT[12]};
                        // Sets Carry flag if there was overflow in to the MSB of HIGH_RESULT
                        FLAGS_OUT[C_FLAG] = HIGH_RESULT[4];
                        // The output is the concatenation of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[11:0]};                        
                        // Subtract Flag is reset
                        FLAGS_OUT[N_FLAG] = 1'b0;                        
                        // Z Flag not affected
                        FLAGS_OUT[Z_FLAG]= FLAGS_IN[Z_FLAG];
                                             
                    end
                    
                // ADD SP,n
                //control unit change z to reset for addSP

                  //increment input A
                  INC:
                    begin
                      ALU_OUT = A+1;                        
                      //flags are not affected
                      FLAGS_OUT = FLAGS_IN;   
                  end
                  //decrement input A
                  DEC:
                    begin
                        ALU_OUT = A-1;
                        //flags are not affected
                        FLAGS_OUT = FLAGS_IN;     
                    end 
                    
                                                     
                default: 
                    begin
                        ALU_OUT = 16'b0;
                        FLAGS_OUT = FLAGS_IN;
                    end
            endcase
        end
    endmodule