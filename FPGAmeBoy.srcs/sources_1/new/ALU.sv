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
        input [4:0] ALU_FUN;
        input [7:0] A,B;
        input [3:0] FLAGS_IN;
        output logic [7:0] ALU_OUT; 
        output logic [3:0] FLAGS_OUT;
       
       // Local Arithmetic Op-code parameters
       localparam ADD  = 5'b00000;
       localparam ADC  = 5'b00001;
       localparam SUB  = 5'b00010;
       localparam SBC  = 5'b00011;
       localparam AND  = 5'b00100;
       localparam OR   = 5'b00101;
       localparam XOR  = 5'b00110;
       localparam CP   = 5'b00111;
       localparam INC  = 5'b01000;
       localparam DEC  = 5'b01001;  
       localparam SWAP = 5'b01010;
       localparam DAA  = 5'b01011;
       localparam CPL  = 5'b01100;
       localparam CCF  = 5'b01101;
       localparam SCF  = 5'b01110;
       localparam RLC  = 5'b01111;
       localparam RL   = 5'b10000;
       localparam RRC  = 5'b10001;
       localparam RR   = 5'b10010;
       localparam SLA  = 5'b10011;
       localparam SRA  = 5'b10100;
       localparam SRL  = 5'b10101;
       localparam BIT  = 5'b10110;
       localparam SET  = 5'b10111;
       localparam RES  = 5'b11000;

       
       // Flag Register Bits
       // [Z,N,H,C]
       localparam Z_FLAG = 2'd3;
       localparam N_FLAG = 2'd2;
       localparam H_FLAG = 2'd1;
       localparam C_FLAG = 2'd0;
       
       
       // Upper and lower 4 bit results + 1 bit for Half and Carry Flags
       logic [4:0] HIGH_RESULT;
       logic [4:0] LOW_RESULT;
       logic [8:0] TEMP_RESULT; 
       
        always_comb
        begin //reevaluate If these change
            case(ALU_FUN)
            
                // Concatenation station
                ADD: 
                    begin
                        // Concatenation  of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, A[3:0]} + {1'b0, B[3:0]};
                        // Sets Half-carry flag if there was overflow in to the MSB of LOW_RESULT
                        FLAGS_OUT[H_FLAG] = LOW_RESULT[4];
                        // Concatenation  of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} + {1'b0, B[7:4]} + {4'b0, LOW_RESULT[4]};
                        // Sets Carry flag if there was overflow in to the MSB of HIGH_RESULT
                        FLAGS_OUT[C_FLAG] = HIGH_RESULT[4];
                        // The output is the concatenation of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};                        
                        // Subtract Flag is reset
                        FLAGS_OUT[N_FLAG] = 1'b0;                        
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;            
                    end
                    
                // Concatenation station + C
                ADC: 
                    begin
                        // Concatenation  of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width and carry flag concatenation
                        LOW_RESULT = {1'b0, A[3:0]} + {1'b0, B[3:0]} + {4'b0, FLAGS_IN[C_FLAG]};
                        // Sets Half-carry flag if there was overflow in to the MSB of LOW_RESULT
                        FLAGS_OUT[H_FLAG] = LOW_RESULT[4];
                        // Concatenation  of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} + {1'b0, B[7:4]} + {4'b0, LOW_RESULT[4]};
                        // Sets Carry flag if there was overflow in to the MSB of HIGH_RESULT
                        FLAGS_OUT[C_FLAG] = HIGH_RESULT[4];
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};                        
                        // Subtract Flag is reset
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end
                // (-) Concatenation station
                SUB:
                    begin
                        // Sets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b1;
                        // Concatenation of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, A[3:0]} - {1'b0, B[3:0]};
                        // Sets Half-carry flag if there was no borrow from the MSB
                        FLAGS_OUT[H_FLAG] = ~LOW_RESULT[4];
                        // Concatenation of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} - {1'b0, B[7:4]} - {4'b0, LOW_RESULT[4]};
                        // Sets Carry flag if there was no borrow from the MSB
                        FLAGS_OUT[C_FLAG] = ~HIGH_RESULT[4];
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end 
                // (-) Concatenation station + C               
                SBC:
                    begin
                        // Sets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b1;
                        // Concatenation  of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width and carry flag concatenation
                        LOW_RESULT = {1'b0, A[3:0]} - ({1'b0, B[3:0]} + {4'b0, FLAGS_IN[C_FLAG]});
                        // Sets Half-carry flag if there was no borrow from the MSB
                        FLAGS_OUT[H_FLAG] = ~LOW_RESULT[4];
                        // Concatenation of upper 4 bits of input with 
                        // additional bit for proper LOW_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} - ({1'b0, B[7:4]} - {4'b0, LOW_RESULT[4]});
                        // Sets Carry flag if there was no borrow from the MSB
                        FLAGS_OUT[C_FLAG] = ~HIGH_RESULT[4];
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end 
                  //Logical AND 2 inputs A B, result in A              
                  AND:
                    begin
                        FLAGS_OUT[H_FLAG] = 1'b1;
                        ALU_OUT  = A & B;
                        //Set Z flag to 1 if result is zero
                        FLAGS_OUT[Z_FLAG] = (ALU_OUT   == 8'b0) ? 1'b1 : 1'b0;
                        // N & C flags are set to 0
                        FLAGS_OUT[C_FLAG] = 1'b0;
                        FLAGS_OUT[N_FLAG] = 1'b0;
                                              
                    end
                  // logical OR 2 inputs A B, result in A
                  OR:
                    begin
                        ALU_OUT = A | B;
                        //Set Z flag to 1 if result is zero
                        FLAGS_OUT[Z_FLAG] = (ALU_OUT   == 8'b0) ? 1'b1 : 1'b0;
                        //Set N,H,C flags to zero
                        FLAGS_OUT[H_FLAG] = 1'b0; 
                        FLAGS_OUT[N_FLAG] = 1'b0; 
                        FLAGS_OUT[C_FLAG] = 1'b0; 
                    end
                    
                  //logical XOR 2inputs A B, result in A
                  XOR:
                    begin
                        ALU_OUT = A ^ B;
                        //Set Z flag to 1 if result is zero
                        FLAGS_OUT[Z_FLAG] = (ALU_OUT   == 8'b0) ? 1'b1 : 1'b0;
                        //Set N,H,C flags to zero
                        FLAGS_OUT[H_FLAG] = 1'b0; 
                        FLAGS_OUT[N_FLAG] = 1'b0; 
                        FLAGS_OUT[C_FLAG] = 1'b0; 
                    end
                   
                  //compare input A & B, set ALU_OUT = A-B
                  CP:
                    begin
                        // Sets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b1;
                        // Concatenation of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, A[3:0]} - {1'b0, B[3:0]};
                        // Sets Half-carry flag if there was no borrow from the MSB
                        FLAGS_OUT[H_FLAG] = ~LOW_RESULT[4];
                        // Concatenation of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, A[7:4]} - {1'b0, B[7:4]} - {4'b0, LOW_RESULT[4]};
                        // Sets Carry flag if there was no borrow from the MSB
                        FLAGS_OUT[C_FLAG] = ~HIGH_RESULT[4];
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                     end 
                  //increment input B
                  INC:
                    begin
                      // Concatenation  of lower 4 bits of input B with 
                      // additional bit 
                      LOW_RESULT = {1'b0, B[3:0]} + {4'b0, 1'b1};
                      // Sets Half-carry flag is there was overflow ito the fifth bit of LOW_RESULT
                      FLAGS_OUT[H_FLAG] = LOW_RESULT[4];
                      // Concatenation  of upper 4 bits of input with 
                      // additional bit for proper HIGH_RESULT bit-width
                      HIGH_RESULT =  {1'b0, B[7:4]} + {4'b0, LOW_RESULT[4]};
                                           
                      // The output is the addition of the upper and lower 4 bits
                      ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};                        
                      // Subtract Flag is reset
                      FLAGS_OUT[N_FLAG] = 1'b0;                        
                      // Z Flag conditional
                      if (ALU_OUT == 8'b0)
                          FLAGS_OUT[Z_FLAG] = 1'b1;
                      else           
                          FLAGS_OUT[Z_FLAG] = 1'b0;     
                      //C flag is not affected
                      FLAGS_OUT[C_FLAG] = FLAGS_IN[C_FLAG];       
                  end
                  //decrement input B
                  DEC:
                    begin
                        //C flag is not affected
                        FLAGS_OUT[C_FLAG] = FLAGS_IN[C_FLAG]; 
                        // Sets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b1;
                        // Concatenation of lower 4 bits of the inputs with 
                        // additional bit for proper LOW_RESULT bit-width
                        LOW_RESULT = {1'b0, B[3:0]} - {4'b0, 1'b1};
                        // Sets Half-carry flag if there was no borrow from the MSB
                        FLAGS_OUT[H_FLAG] = ~LOW_RESULT[4];
                        // Concatenation of upper 4 bits of input with 
                        // additional bit for proper HIGH_RESULT bit-width
                        HIGH_RESULT = {1'b0, B[7:4]} - {4'b0, LOW_RESULT[4]};
                        // The output is the addition of the upper and lower 4 bits
                        ALU_OUT = {HIGH_RESULT[3:0], LOW_RESULT[3:0]};
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end 
                    
                  //Swap upper and lower nibbles of input A
                  SWAP:
                    begin
                        //Set Z flag to 1 if result is zero
                        FLAGS_OUT[Z_FLAG] = (ALU_OUT   == 8'b0) ? 1'b1 : 1'b0;
                        //Set N,H,C flags to zero
                        FLAGS_OUT[H_FLAG] = 1'b0; 
                        FLAGS_OUT[N_FLAG] = 1'b0; 
                        FLAGS_OUT[C_FLAG] = 1'b0; 
                        ALU_OUT = {A[3:0], A[7:4]};
                    end 
                    
                  // DAA - decimal adjust after addition
                  // Used to correct presentation of ALU_OUT to BCD
                  DAA:
                    begin
                        // There was an ADD previously
                        if (FLAGS_IN[N_FLAG] == 1'b0)
                        begin
                            //if there was a borrow (Half-carry flag equals 1) or the lower nibble was greater than 9
                            //add 6 to the result
                            if (FLAGS_IN[H_FLAG] || ((A & 8'h0f) > 8'h09))
                                begin
                                    TEMP_RESULT = {1'b0, A} + 9'h6;    
                                end
                             else
                                begin
                                    TEMP_RESULT = {1'b0, A};   
                                end    
                            //if Carry-flag IN equals 1 or the upper nibble was greater than 9
                            //add 6 to the upper nibble 
                            if (FLAGS_IN[C_FLAG] || (TEMP_RESULT > 9'h99))
                                begin
                                    TEMP_RESULT = TEMP_RESULT + 9'h60;    
                                end
                        end    
                        // There was a SUB previously
                        else
                            begin
                                if (FLAGS_IN[H_FLAG] == 1'b1)
                                    begin
                                        TEMP_RESULT = {1'b0, A - 8'h6}; 
                                    end
                                else
                                    begin
                                        TEMP_RESULT = {1'b0, A};
                                    end 
                                if (FLAGS_IN[C_FLAG] == 1'b1)
                                    begin
                                        TEMP_RESULT = TEMP_RESULT - 9'h60;
                                    end                               
                            end                            
                        ALU_OUT = TEMP_RESULT[7:0];
                        //C flag condition
                        FLAGS_OUT[C_FLAG]= TEMP_RESULT[8] ? 1'b1 : 1'b0;
                        //Set Z flag to 1 if result is zero
                        FLAGS_OUT[Z_FLAG] = (ALU_OUT   == 8'b0) ? 1'b1 : 1'b0;
                        //N flag is not affected
                        FLAGS_OUT[N_FLAG] = FLAGS_IN[N_FLAG];
                        // H flag is set to 0
                        FLAGS_OUT[H_FLAG] = 1'b0;

                    end
                  // CPL - complement input register  
                  CPL:
                    begin 
                        //Z & C flags are not affected
                        FLAGS_OUT[Z_FLAG] = FLAGS_IN[Z_FLAG];
                        FLAGS_OUT[C_FLAG] = FLAGS_IN[C_FLAG];
                        //flip all bits of input register
                        ALU_OUT = ~A;
                        // N & H flags are set to 1
                        FLAGS_OUT[H_FLAG] = 1'b1;
                        FLAGS_OUT[N_FLAG] = 1'b1;
                    end
                    
                  // CCF - Complement carry flag
                  CCF:
                    begin
                        //Z flag is not affected
                        FLAGS_OUT[Z_FLAG] = FLAGS_IN[Z_FLAG];
                        // N & H flags are set to 0
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        //C flag is complemented
                        FLAGS_OUT[C_FLAG] = ~FLAGS_IN[C_FLAG];
                        ALU_OUT = A; //doesnt matter
                    end
                    
                  // SCF - Set carry flag
                  SCF:
                    begin
                        //Z flag is not affected
                        FLAGS_OUT[Z_FLAG] = FLAGS_IN[Z_FLAG];
                        // N & H flags are set to 0
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        //C flag is set
                        FLAGS_OUT[C_FLAG] = 1'b1;
                        ALU_OUT = A; //doesnt matter
                    end
                
                    
                // Used for the C and CB Prefix commands  
                // Rotates A input left, MSB = Carry Flag 
                // LSB = A input MSB      
                RLC: 
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Rotates the bits in A to the left by 1    
                        ALU_OUT[7:1] = A[6:0];
                        ALU_OUT[0] = A[7];
                        // Sets the carry flag equal to the MSB of A
                        FLAGS_OUT[C_FLAG] = A[7];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end
                // Used for the RLA and CB Prefix commands                                 
                // Rotates A input left through the Carry Flag
                // LSB = input Carry Flag  
                RL:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Rotates the bits in A to the left by 1    
                        ALU_OUT[7:1] = A[6:0];
                        // Sets LSB of output to the current Carry Flag
                        ALU_OUT[0] = FLAGS_IN[C_FLAG];
                        // Sets the carry flag equal to the MSB of A
                        FLAGS_OUT[C_FLAG] = A[7];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end
                // Used for the RRCA and CB Prefix commands  
                // Rotates A input right
                // MSB = A input MSB    
                RRC:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Rotates the bits in A to the right by 1    
                        ALU_OUT[6:0] = A[7:1];
                        ALU_OUT[7] = A[0];
                        // Sets the carry flag equal to the LSB of A
                        FLAGS_OUT[C_FLAG] = A[0];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end   
                // Used for the RLA and CB Prefix commands  
                // Rotates A input right through the Carry Flag
                // MSB = input Carry Flag     
                RR:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Rotates the bits in A to the right by 1    
                        ALU_OUT[6:0] = A[7:1];
                        // Sets LSB of output to the current Carry Flag
                        ALU_OUT[7] = FLAGS_IN[C_FLAG];
                        // Sets the carry flag equal to the LSB of A
                        FLAGS_OUT[C_FLAG] = A[0];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end

                // Shifts A input by 1 to the left into the Carry Flag
                // LSB = 0  
                SLA:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Shifts the bits in A to the left by 1
                        ALU_OUT[7:1] = A[6:0];
                        ALU_OUT[0] = 0;
                        // Sets the Carry Flag equal to the MSB of A
                        FLAGS_OUT[C_FLAG] = A[7];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end
                // Shifts A input by 1 to the right into the Carry Flag
                // MSB is not changed
                SRA:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Shifts the bits in A to the right by 1, except MSB
                        ALU_OUT[6:0] = A[7:1];
                        ALU_OUT[7] = A[7];
                        // Sets the Carry Flag equal to the LSB of A
                        FLAGS_OUT[C_FLAG] = A[0];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end
                  // Shifts A input by 1 to the left into the Carry Flag
                  // MSB = 0
                  SRL:
                    begin
                        // Resets the Subtract FLag
                        FLAGS_OUT[N_FLAG] = 1'b0;
                        // Resets the Half Carry FLag
                        FLAGS_OUT[H_FLAG] = 1'b0;
                        // Shifts the bits in A to the right by 1, MSB = 0
                        ALU_OUT[6:0] = A[7:1];
                        ALU_OUT[7] = 0;
                        // Sets the Carry Flag equal to the LSB of A
                        FLAGS_OUT[C_FLAG] = A[0];
                        // Z Flag conditional
                        if (ALU_OUT == 8'b0)
                            FLAGS_OUT[Z_FLAG] = 1'b1;
                        else           
                            FLAGS_OUT[Z_FLAG] = 1'b0;   
                    end 
                    // Test bit [input B: 0-7] of input register [input A]
                    BIT:
                        begin
                            // Resets the Subtract FLag
                            FLAGS_OUT[N_FLAG] = 1'b0;
                            // Sets the Half Carry FLag
                            FLAGS_OUT[H_FLAG] = 1'b1;
                            // No Change in the Carry Flag
                            FLAGS_OUT[C_FLAG] = FLAGS_IN[C_FLAG];
                            // Z Flag set if bit is 0, it is cleared otherwise                            
                            FLAGS_OUT[Z_FLAG] = ~A[B]; 
                            // Outputs index bit 
                            ALU_OUT = B;                                               
                        end
                    // Set bit [input B: 0-7] of input register [input A]
                    SET:
                        begin
                           // No change in Flags 
                           FLAGS_OUT = FLAGS_IN;
                           // Sets the output equal to A
                           ALU_OUT = A;
                           // Sets the specified bit
                           ALU_OUT[B] = 1'b1; 
                        end
                    // Clears bit [input B: 0-7] of input register [input A]
                    RES:
                        begin
                           // No change in Flags 
                           FLAGS_OUT = FLAGS_IN;
                           // Sets the output equal to A
                           ALU_OUT = A;
                           // Clears the specified bit
                           ALU_OUT[B] = 1'b0; 
                        end                                 
                                                      
                default: 
                    begin
                        ALU_OUT = 8'b0;
                        FLAGS_OUT = FLAGS_IN;
                    end
            endcase
        end
    endmodule