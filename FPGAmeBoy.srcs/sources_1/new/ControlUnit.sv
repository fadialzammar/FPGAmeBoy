`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/07/2021 03:35:48 PM
// Design Name: 
// Module Name: ControlUnit
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


module ControlUnit(
        input CLK, C, Z, N, H, INTR, RESET,
        input [7:0] OPCODE,
        output logic PC_LD, PC_INC,             // program counter
        output logic [1:0] PC_MUX_SEL,
        output logic RF_WR,                     // register file
        output logic [1:0] RF_WR_SEL,
        output logic [4:0] RF_ADRX, RF_ADRY,
        output logic [4:0] ALU_SEL,             // ALU
        output logic ALU_OPX_SEL,
        output logic [1:0] ALU_OPY_SEL,
        output logic SCR_DATA_SEL, SCR_WE,      // scratch pad
        output logic [1:0] SCR_ADDR_SEL,
        output logic SP_LD, SP_INCR, SP_DECR,   // stack pointer
        output logic C_FLAG_LD, C_FLAG_SET, C_FLAG_CLR, // C Flag control
        output logic Z_FLAG_LD, Z_FLAG_SET, Z_FLAG_CLR, // Z Flag control
        output logic N_FLAG_LD, N_FLAG_SET, N_FLAG_CLR, // N Flag control
        output logic H_FLAG_LD, H_FLAG_SET, H_FLAG_CLR, // H Flag control
        output logic I_CLR, I_SET, FLG_LD_SEL, // interrupts
        output logic RST,       // reset
        output logic IO_STRB    // IO
    );

    parameter RF_MUX_ALU = 0;   // ALU output
    parameter RF_MUX_SCR = 1;   // scratch RAM output
    parameter RF_MUX_SP = 2;    // stack pointer output
    parameter RF_MUX_IN = 3;    // external input
    parameter RF_MUX_IMM = 4;   // immediate value from instruction
    parameter RF_MUX_DY = 5;    // DY output of reg file

    parameter SCR_ADDR_DY = 0;  // DY output of reg file
    parameter SCR_ADDR_ADRY = 1;    // ADRY of reg file
    parameter SCR_ADDR_SP = 2;  // stack pointer output
    parameter SCR_ADDR_SP_SUB = 3;  // stack pointer output minus 1?

    parameter REG_B = 3'b000;
    parameter REG_C = 3'b001;
    parameter REG_D = 3'b010;
    parameter REG_E = 3'b011;
    parameter REG_H = 3'b100;
    parameter REG_L = 3'b101;
    parameter REG_HL = 3'b110;
    parameter REG_A = 3'b111;
    
    // ALU Operations
    localparam ADD_ALU  = 5'b00000;
    localparam ADC_ALU  = 5'b00001;
    localparam SUB_ALU  = 5'b00010;
    localparam SBC_ALU  = 5'b00011;
    localparam AND_ALU  = 5'b00100;
    localparam OR_ALU   = 5'b00101;
    localparam XOR_ALU  = 5'b00110;
    localparam CP_ALU   = 5'b00111;
    localparam INC_ALU  = 5'b01000;
    localparam DEC_ALU  = 5'b01001;  
    localparam SWAP_ALU = 5'b01010;
    localparam DAA_ALU  = 5'b01011;
    localparam CPL_ALU  = 5'b01100;
    localparam CCF_ALU  = 5'b01101;
    localparam SCF_ALU  = 5'b01110;
    localparam RLC_ALU  = 5'b01111;
    localparam RL_ALU   = 5'b10000;
    localparam RRC_ALU  = 5'b10001;
    localparam RR_ALU   = 5'b10010;
    localparam SLA_ALU  = 5'b10011;
    localparam SRA_ALU  = 5'b10100;
    localparam SRL_ALU  = 5'b10101;
    localparam BIT_ALU  = 5'b10110;
    localparam SET_ALU  = 5'b10111;
    localparam RES_ALU  = 5'b11000;

    
    typedef enum int {INIT, FETCH, EXEC, INTERRUPT} STATE;

    STATE NS, PS = INIT;

    logic mcycle = 0;

    always_ff @(posedge CLK) begin
        if (RESET)
            PS <= INIT;
        else
            PS <= NS;
    end

    always_comb begin
        I_SET = 0; I_CLR =0; PC_LD=0; PC_INC=0; ALU_OPY_SEL=0; ALU_OPX_SEL = 0; RF_WR=0; RF_ADRX = 0; RF_ADRY = 0;
        SP_LD=0; SP_INCR=0; SP_DECR=0;
        SCR_WE=0; SCR_DATA_SEL=0; RST=0; PC_MUX_SEL=0; RF_WR_SEL=0; SCR_ADDR_SEL=0; ALU_SEL=0; IO_STRB = 0;
        C_FLAG_LD = 0; C_FLAG_SET = 0; C_FLAG_CLR = 0; 
        Z_FLAG_LD = 0; Z_FLAG_SET = 0; Z_FLAG_CLR = 0; 
        N_FLAG_LD = 0; N_FLAG_SET = 0; N_FLAG_CLR = 0; 
        H_FLAG_LD = 0; H_FLAG_SET = 0; H_FLAG_CLR = 0; FLG_LD_SEL=0;
        
        

        case (PS)
            INIT: begin
                RST = 1;
                NS = FETCH;
            end

            FETCH: begin
                PC_INC = 1;
                NS = EXEC;
            end

            EXEC:
            begin
                if (INTR)
                    NS = INTERRUPT;
                else
                    NS = FETCH;

                case (OPCODE) inside
                    
                    8'b00000000:  // NOP
                    begin 
                        // Control signal later TM                              
                        // No Reg Write
                        RF_WR = 0;  
                                                      
                        // Flags
                        C_FLAG_LD = 0;
                        Z_FLAG_LD = 0;
                        N_FLAG_LD = 0;
                        H_FLAG_LD = 0;
                       
                    end
                    
                    //
                    // 8-bit loads
                    //

                    8'b00???110: begin  // LD r, n
                        if (OPCODE[5:3] == 3'b110) begin    // LD (HL), n
                            
                        end
                        else begin  // normal LD r8, n8
                            RF_WR = 1;
                            RF_WR_SEL = RF_MUX_IMM;
                            RF_ADRX = OPCODE[5:3];
                            // SCR_ADDR_SEL = 1;
                        end
                    end

                    8'b01??????: begin  // LD r, r
                        if (OPCODE == 8'b01110110) begin    // HALT
                            
                        end

                        else if (OPCODE[5:3] == 3'b110) begin   // LD (HL), r
                            if (mcycle == 0) begin
                                
                            end
                            if (mcycle == 1) begin
                                
                            end
                        end

                        else if (OPCODE[2:0] == 3'b110) begin   // LD r, (HL)
                            if (mcycle == 0) begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            if (mcycle == 1) begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end
                        end

                        else begin  // normal LD r8, r8
                            RF_WR = 1;
                            RF_WR_SEL = RF_MUX_DY;
                            RF_ADRX = OPCODE[5:3];  // copies from Y into X
                            RF_ADRY = OPCODE[2:0];
                            // SCR_ADDR_SEL = 0;
                        end
                    end

                    8'b00001010: begin // LD A, (BC)
                    
                    end

                    8'b00011010: begin // LD A, (DE)
                    
                    end

                    8'b00000010: begin // LD (BC), A
                    
                    end

                    8'b00010010: begin // LD (DE), A
                    
                    end

                    8'b11111010: begin // LD A, (nn), (nn) = 16-bit immediate, LSB first
                    
                    end

                    8'b11101010: begin // LD (nn), A, (nn) = 16-bit immediate, LSB first
                    
                    end
                    
                    // ALU Time
                    8'b10000???:  // ADD A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = ADD_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b10001???:  // ADC A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = ADC_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b10100???:  // AND A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = AND_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b10111???:  // CP A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = CP_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                     8'b10110???:  // OR A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = OR_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // OR A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b10011???:  // SBC A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SBC_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];

                        // SBC A, (HL)  /// FIX Later
                        if (OPCODE[2:0] == 3'b110)
                        begin                         
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end
                        end                                                         
                    end
                    
                    
                    8'b10010???:  // SUB A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SUB_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0];  
                                                        
                        // SUB A, (HL)  /// FIX Later  
                        if (OPCODE[2:0] == 3'b110)
                        begin                        
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end
                        end                                                  
                    end
                    
                    8'b10101???:  // XOR A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = XOR_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;                            
                        // Register File Addresses
                        RF_ADRX = REG_A;
                        RF_ADRY = OPCODE[2:0]; 
                           
                        // XOR A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                  
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                SCR_ADDR_SEL =  SCR_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_SCR;
                                RF_ADRX = OPCODE[5:3]; // r
                            end
                        end
                    end                        
                    
                    default: begin
                        // literally crashes on a real game boy
                    end


                endcase // OPCODE

                if (INTR == 1)
                    NS = INTERRUPT;
                NS = FETCH;
            mcycle++;
            end // EXEC
        endcase // PS
    end


    
endmodule
