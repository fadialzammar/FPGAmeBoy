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
        output logic PC_LD, PC_INC,                     // program counter
        output logic [1:0] PC_MUX_SEL,
        output logic RF_WR,                             // register file
        output logic [2:0] RF_WR_SEL,
        output logic [4:0] RF_ADRX, RF_ADRY,
        output logic [4:0] ALU_SEL,                     // ALU
        output logic ALU_OPX_SEL,
        output logic [1:0] ALU_OPY_SEL,
        output logic MEM_WE, MEM_RE,                    // memory
        output logic [1:0] MEM_ADDR_SEL, MEM_DATA_SEL,
        output logic SP_LD, SP_INCR, SP_DECR,           // stack pointer
        output logic C_FLAG_LD, C_FLAG_SET, C_FLAG_CLR, // C Flag control
        output logic Z_FLAG_LD, Z_FLAG_SET, Z_FLAG_CLR, // Z Flag control
        output logic N_FLAG_LD, N_FLAG_SET, N_FLAG_CLR, // N Flag control
        output logic H_FLAG_LD, H_FLAG_SET, H_FLAG_CLR, // H Flag control
        output logic FLAGS_DATA_SEL,
        output logic I_CLR, I_SET, FLG_LD_SEL,          // interrupts
        output logic RST,                               // reset
        output logic IO_STRB                            // IO
    );

    parameter RF_MUX_ALU = 0;   // ALU output
    parameter RF_MUX_MEM = 1;   // scratch RAM output
    parameter RF_MUX_SP = 2;    // stack pointer output
    parameter RF_MUX_IN = 3;    // external input
    parameter RF_MUX_IMM = 4;   // immediate value from instruction
    parameter RF_MUX_DY = 5;    // DY output of reg file

    parameter MEM_ADDR_DY = 0;      // DY output of reg file
    parameter MEM_ADDR_ADRY = 1;    // ADRY of reg file
    parameter MEM_ADDR_SP = 2;      // stack pointer output
    parameter MEM_ADDR_SP_SUB = 3;  // stack pointer output minus 1?
    
    parameter MEM_DATA_DX = 0;      // DX output of the Reg File
    parameter MEM_DATA_PC = 1;      // PC value output 
    parameter MEM_DATA_FLAGS = 2;   // Flags Register values
    
    parameter FLAGS_DATA_ALU = 0;   // ALU Flags Output
    parameter FLAGS_DATA_MEM = 1;   // Memory Flags Output 
    
    parameter ALU_B_MUX_DY = 0;     // DY output of the Reg File
    parameter ALU_B_MUX_MEM = 1;    // Memory output

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

    
    typedef enum int {INIT, FETCH, EXEC, INTERRUPT, CB_EXEC, SP, IMMED} STATE;

    STATE NS, PS = INIT;

     logic mcycle = 0;
     // Flag used for identifying that NS after EXEC is not FETCH
     logic special_state = 1'b0;
     // Flag for CB prefixes
     logic CB_FLAG = 1'b0;
     // Flags for PUSH and POP
     logic POP_FLAG = 1'b0;
     logic POP_LB = 1'b0;
     logic POP_HB = 1'b0;
     logic PUSH_FLAG = 1'b0;
     logic PUSH_LB = 1'b0;
     logic PUSH_HB = 1'b0;
     
     // Immediate Value Select and defines
     logic [3:0] IMMED_SEL = 4'b0000;
     localparam ADD_IMMED   = 4'b0000;
     localparam SUB_IMMED   = 4'b0001;
     localparam AND_IMMED   = 4'b0010;
     localparam OR_IMMED    = 4'b0011;
     localparam ADC_IMMED   = 4'b0100;
     localparam SBC_IMMED   = 4'b0101;
     localparam XOR_IMMED   = 4'b0110;
     localparam CP_IMMED    = 4'b0111;
     
     logic [7:0] FLAGS;
     // Flag format for the Gameboy
     assign FLAGS = {Z,N,H,C,4'b0000};
     
    always_ff @(posedge CLK) begin
        if (RESET)
            PS <= INIT;
        else
            PS <= NS;
    end

    
    always_comb begin
        I_SET = 0; I_CLR =0; RST=0; IO_STRB = 0;
        PC_LD=0; PC_INC=0; PC_MUX_SEL=0;
        RF_WR=0; RF_ADRX = 0; RF_ADRY = 0;  RF_WR_SEL=0;
        SP_LD=0; SP_INCR=0; SP_DECR=0;
        MEM_WE=0; MEM_DATA_SEL=0; MEM_ADDR_SEL=0; 
        ALU_OPY_SEL=0; ALU_OPX_SEL = 0; ALU_SEL=0;
        C_FLAG_LD = 0; C_FLAG_SET = 0; C_FLAG_CLR = 0; 
        Z_FLAG_LD = 0; Z_FLAG_SET = 0; Z_FLAG_CLR = 0; 
        N_FLAG_LD = 0; N_FLAG_SET = 0; N_FLAG_CLR = 0; 
        H_FLAG_LD = 0; H_FLAG_SET = 0; H_FLAG_CLR = 0; FLG_LD_SEL = 0;  

        case (PS)
            INIT: 
            begin
                RST = 1;
                NS = FETCH;
            end

            FETCH:
            begin
                PC_INC = 1;
                if (CB_FLAG == 1)
                    NS = CB_EXEC;
                else
                    NS = EXEC;
            end

            EXEC:
            begin
                if (INTR)
                    NS = INTERRUPT;
                else
                    if(~special_state)
                        NS = FETCH;

                case (OPCODE) inside
                    
                    8'b00000000:  // NOP
                    begin 
                        // Control signal later TM                              
                        // No Reg Write
                        RF_WR = 0;  
                        // No Memory read or write
                        MEM_WE = 0;
                        MEM_RE = 0;            
                        // Flags
                        C_FLAG_LD = 0;
                        Z_FLAG_LD = 0;
                        N_FLAG_LD = 0;
                        H_FLAG_LD = 0;
                       
                    end
                    
                    //
                    // 8-bit loads
                    //
                    
                    8'b00110111: // SCF
                    begin
                        // Flags
                        C_FLAG_SET = 1;
                        Z_FLAG_LD = 0;
                        N_FLAG_CLR = 1;
                        H_FLAG_CLR = 1;
                    end

                    8'b00110111: // CCF
                    begin
                        // Flags                        
                        C_FLAG_SET = C == 0 ? 1'b1 : 1'b0;
                        C_FLAG_CLR = C == 1 ? 1'b1 : 1'b0;
                        Z_FLAG_LD = 0;
                        N_FLAG_CLR = 1;
                        H_FLAG_CLR = 1;
                    end
                    
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            if (mcycle == 1) begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
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
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;                       
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
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end
                        end
                    end  
                    
                    8'b11??0001: // POP
                    begin
                        POP_FLAG = 1'b1;
                        PUSH_LB = 1'b0;
                        PUSH_HB = 1'b1;
                        special_state = 1'b1;
                        NS = SP;                       
                    end       
                    
                    8'b11??0101: // PUSH
                    begin
                        PUSH_FLAG = 1'b1;
                        PUSH_LB = 1'b1;
                        PUSH_HB = 1'b0;
                        SP_DECR = 1'b1; 
                        special_state = 1'b1;
                        NS = SP;         // ======================== Might need to send to wait state for consistent timing ========================                
                    end             
                    
                    8'b11??0110: // ADD, SUB, AND, OR with Immediate values 
                    begin
                        case(OPCODE)
                            2'b00: // ADD A, immed
                                begin
                                    IMMED_SEL = ADD_IMMED;
                                end
                                
                             2'b01:  // SUB A, immed
                                begin
                                    IMMED_SEL = SUB_IMMED;
                                end
                                
                             2'b10:  // AND A, immed
                                begin
                                    IMMED_SEL = AND_IMMED;
                                end
                                
                             2'b11:  // OR A, immed
                                begin
                                    IMMED_SEL = OR_IMMED;
                                end                           
                        endcase
                        special_state = 1'b1;
                        NS = IMMED;
                    end   
                    
                    8'b11??1110: // ADC, SBC, XOR, CP with Immediate values 
                    begin
                        case(OPCODE)
                            2'b00: // ADC A, immed
                                begin
                                    IMMED_SEL = ADC_IMMED;
                                end
                                
                             2'b01:  // SBC A, immed
                                begin
                                    IMMED_SEL = SBC_IMMED;
                                end
                                
                             2'b10:  // XOR A, immed
                                begin
                                    IMMED_SEL = XOR_IMMED;
                                end
                                
                             2'b11:  // CP A, immed
                                begin
                                    IMMED_SEL = CP_IMMED;
                                end                           
                        endcase
                        special_state = 1'b1;
                        NS = IMMED;
                    end
                    
                    8'b11001011: // CB Prefix command
                    begin
                        CB_FLAG = 1'b1;
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
            
            IMMED: begin  // Immediate Value Instrutions
                // Same for each case
                // ALU A input mux select                                
                ALU_OPX_SEL = 1'b0;
                // ALU B input mux select
                ALU_OPY_SEL = ALU_B_MUX_MEM;                                
                // Input to the Reg File is the ALU output
                RF_WR_SEL = RF_MUX_ALU;                                
                // Write operation back into Register A
                RF_WR = 1;                                
                // Flags
                C_FLAG_LD = 1;
                Z_FLAG_LD = 1;
                N_FLAG_LD = 1;
                H_FLAG_LD = 1;
                // Flag register data select
                FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                // Register File Addresses
                RF_ADRX = REG_A;
                
                case(IMMED_SEL)
                    ADD_IMMED: // ADD A, immed
                        begin                                                      
                            // ALU Operation Select
                            ALU_SEL = ADD_ALU;                             
                        end
                        
                    SUB_IMMED: // SUB A, immed
                        begin                                             
                            // ALU Operation Select
                            ALU_SEL = SUB_ALU;                                                        
                        end
                        
                    AND_IMMED: // AND A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = AND_ALU;                                                           
                        end
                        
                    OR_IMMED: // OR A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = OR_ALU;
                        end
                        
                    ADC_IMMED: // ADC A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = ADC_ALU;
                        end
                        
                    SUB_IMMED: // SBC A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = SUB_ALU;
                        end
                    
                    XOR_IMMED: // XOR A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = XOR_ALU;
                        end
                    
                    CP_IMMED: // CP A, immed
                        begin
                            // ALU Operation Select
                            ALU_SEL = CP_ALU;
                        end                                      
                endcase
                special_state = 1'b0;
                NS = FETCH;
            end
            
            
            SP: begin   // Stack Pointer instructions
                    begin
                        if (PUSH_FLAG) // Pushes the Low Byte and then the High Byte
                            begin
                            case (OPCODE[5:4])
                                    2'b00: // BC
                                    begin
                                        // Register File does not write on a PUSH
                                        RF_WR = 0;
                                        // SP decrements additionally if pushing the High Byte
                                        SP_DECR = PUSH_HB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRY depeding on HB Flag
                                        RF_ADRX = PUSH_HB ? REG_B : REG_C;
                                        // Write the pushed value to memory &(SP)
                                        MEM_WE = 1'b1;
                                    end
                                    
                                    2'b01: // DE
                                    begin
                                        // Register File does not write on a PUSH
                                        RF_WR = 0;
                                        // SP decrements additionally if pushing the High Byte
                                        SP_DECR = PUSH_HB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRY depeding on HB Flag
                                        RF_ADRX = PUSH_HB ? REG_D : REG_E;
                                        // Write the pushed value to memory &(SP)
                                        MEM_WE = 1'b1;
                                    end
                                    
                                    2'b10: // HL
                                    begin
                                        // Register File does not write on a PUSH
                                        RF_WR = 0;
                                        // SP decrements additionally if pushing the High Byte
                                        SP_DECR = PUSH_HB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRY depeding on HB Flag
                                        RF_ADRX = PUSH_HB ? REG_H : REG_L;
                                        // Write the pushed value to memory &(SP)
                                        MEM_WE = 1'b1;
                                    end
                                    
                                    2'b11: // AF 
                                    begin
                                        // Register File does not write on a PUSH
                                        RF_WR = 0;
                                        // SP decrements additionally if pushing the High Byte
                                        SP_DECR = PUSH_HB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File or Flag Register values
                                        MEM_DATA_SEL = PUSH_HB ? MEM_DATA_DX : MEM_DATA_FLAGS;
                                        // RF_ADRX set to REG_A 
                                        RF_ADRX = REG_A;                                        
                                        // Write the pushed value to memory &(SP)
                                        MEM_WE = 1'b1;
                                    end
                                endcase                                
                                // Set Low Byte flag low and High Byte flag high to signify that the Low Byte has been pushed                                
                                // Return to the SP state to push the high byte 
                                if(PUSH_LB && ~PUSH_HB)
                                    begin
                                        PUSH_HB = 1'b1;
                                        PUSH_LB = 1'b0;
                                        NS = SP;
                                    end                                                                           
                                // Transition to the fetch state once both bytes are pushed   
                                else 
                                    begin
                                        // Reset High Byte flag, Low Byte flag, PUSH flag, and special state flag
                                        PUSH_FLAG = 1'b0;
                                        POP_HB = 1'b0;
                                        POP_LB = 1'b0;
                                        special_state = 1'b0;
                                        NS = FETCH;
                                    end
                                    
                            end       
                            
                        else if (POP_FLAG)  // Pops the High Byte and then the Low Byte
                             begin
                            case (OPCODE[5:4])
                                    2'b00: // BC
                                    begin
                                        // Reg File select set to memory
                                        RF_WR_SEL = RF_MUX_MEM;
                                        // Register File writes on a POP
                                        RF_WR = 1;
                                        // SP increments if popping the Low Byte
                                        SP_INCR = POP_LB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRX depeding on LB Flag
                                        RF_ADRX = ~POP_LB ? REG_B : REG_C;
                                        // Read the popped value from memory &(SP)
                                        MEM_RE = 1'b1;
                                    end
                                    
                                    2'b01: // DE
                                    begin
                                        // Reg File select set to memory
                                        RF_WR_SEL = RF_MUX_MEM;
                                        // Register File writes on a POP
                                        RF_WR = 1;
                                        // SP increments if popping the Low Byte
                                        SP_INCR = POP_LB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRX depeding on LB Flag
                                        RF_ADRX = ~POP_LB ? REG_D : REG_E;
                                        // Read the popped value from memory &(SP)
                                        MEM_RE = 1'b1;
                                    end
                                    
                                    2'b10: // HL
                                    begin
                                       // Reg File select set to memory
                                        RF_WR_SEL = RF_MUX_MEM;
                                        // Register File writes on a POP
                                        RF_WR = 1;
                                        // SP increments if popping the Low Byte
                                        SP_INCR = POP_LB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File
                                        MEM_DATA_SEL = MEM_DATA_DX;
                                        // High or Low Byte used set by RF_ADRX depeding on HB Flag
                                        RF_ADRX = ~POP_LB ? REG_H : REG_L;
                                        // Read the popped value from memory &(SP)
                                        MEM_RE = 1'b1;
                                    end
                                    
                                    2'b11: // AF
                                    begin
                                        // Reg File select set to memory
                                        RF_WR_SEL = RF_MUX_MEM;
                                        // Register File writes on a POP if not popping the flag register values
                                        RF_WR = ~POP_LB ? 1'b1: 1'b0;
                                        // Load the popped Low Byte values from the stack into the flag register
                                        C_FLAG_LD = POP_LB ? 1'b1 : 1'b0;
                                        Z_FLAG_LD = POP_LB ? 1'b1 : 1'b0;
                                        N_FLAG_LD = POP_LB ? 1'b1 : 1'b0;
                                        H_FLAG_LD = POP_LB ? 1'b1 : 1'b0;   
                                        // Flag register data select
                                        FLAGS_DATA_SEL = POP_LB ? FLAGS_DATA_MEM : FLAGS_DATA_ALU;
                                        // SP increments if popping the low Byte
                                        SP_INCR = POP_LB ? 1'b1 : 1'b0;
                                        // Memory address select set to SP
                                        MEM_ADDR_SEL = MEM_ADDR_SP;
                                        // Memory Data select set to DX output of the Reg File or Flag Register values
                                        MEM_DATA_SEL = POP_HB ? MEM_DATA_DX : MEM_DATA_FLAGS;
                                        // RF_ADRX set to REG_A 
                                        RF_ADRX = REG_A;                                         
                                        // Read the popped value from memory &(SP)
                                        MEM_RE = 1'b1;
                                    end
                                endcase                                
                                // Set Low Byte flag high and High Byte flag low to signify that the High Byte has been popped
                                // Return to the SP state to POP the Low Byte  
                                if(~POP_LB && POP_HB)
                                    begin
                                        POP_HB = 1'b0;
                                        POP_LB = 1'b1;
                                        NS = SP;
                                    end                                                        
                                // Transition to the fetch state once both bytes are poped    
                                else 
                                    begin
                                        // Reset High Byte flag, Low Byte flag, POP flag, and special state flag
                                        POP_FLAG = 1'b0;
                                        POP_HB = 1'b0;
                                        POP_LB = 1'b0;
                                        special_state = 1'b0;
                                        NS = FETCH;
                                    end
                            end                            
                    end
                    
            end // SP
            
            CB_EXEC: //CB prefix opcodes
            begin   
                case (OPCODE) inside
                    // CB Time
                    8'b00000???:  // RLC n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RLC_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;                        
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00001???:  // RRC n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RRC_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00010???:  // RL n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RL_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00011???:  // RR n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RR_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00100???:  // SLA n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SLA_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00101???:  // SRA n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SRA_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00110???:  // SWAP n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SWAP_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    8'b00111???:  // SRL n, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = SRL_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register n
                        RF_WR = 1;                                
                        // Flags
                        C_FLAG_LD = 1;
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        // Writes to the opcode defined address
                        RF_ADRX = OPCODE[2:0];
                        RF_ADRY = REG_A;

                        // CP A, (HL)  /// FIX Later 
                        if (OPCODE[2:0] == 3'b110)
                        begin                      
                            if (mcycle == 0)
                            begin
                                RF_WR = 0;
                                RF_ADRY = REG_HL;
                                MEM_ADDR_SEL =  MEM_ADDR_DY;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    
                    default: begin
                        
                    end
                endcase
                
                if (INTR == 1)
                    NS = INTERRUPT;
                    
                // Reset the CB Flag
                CB_FLAG = 1'b0;
                NS = FETCH;
                mcycle++;
                
            end // CB_EXEC
           
            
        endcase // PS
    end


    
endmodule
