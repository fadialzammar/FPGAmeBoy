`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/07/2021 03:35:48 
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
        output logic [3:0] RF_WR_SEL,
        output logic [2:0] RF_ADRX, RF_ADRY,
        output logic [4:0] ALU_SEL,                     // ALU
        output logic [1:0] ALU_16_SEL,
        output logic ALU_OPX_SEL, ALU_16_A_SEL,
        output logic [2:0] ALU_OPY_SEL, ALU_16_B_SEL,
        output logic MEM_WE, MEM_RE,                    // memory
        output logic [1:0] MEM_ADDR_SEL, 
        output logic [2:0] MEM_DATA_SEL,
        output logic [7:0] IMMED_ADDR_LOW, IMMED_ADDR_HIGH,  //16-bit Immediates
        output logic [7:0] IMMED_DATA_LOW,  IMMED_DATA_HIGH,
        output logic SP_LD, SP_INCR, SP_DECR,           // stack pointer
        output logic SP_DIN_SEL,
        output logic C_FLAG_LD, C_FLAG_SET, C_FLAG_CLR, // C Flag control
        output logic Z_FLAG_LD, Z_FLAG_SET, Z_FLAG_CLR, // Z Flag control
        output logic N_FLAG_LD, N_FLAG_SET, N_FLAG_CLR, // N Flag control
        output logic H_FLAG_LD, H_FLAG_SET, H_FLAG_CLR, // H Flag control
        output logic FLAGS_DATA_SEL,
        output logic I_CLR, I_SET, FLG_LD_SEL,          // interrupts
        output logic RST,                               // reset
        output logic IO_STRB,                           // IO
        output logic [2:0] BIT_SEL                      // BIT select signal 
    ); 
    // RF Data Mux
    parameter RF_MUX_ALU             = 0; // ALU output
    parameter RF_MUX_MEM             = 1; // Memory output
    parameter RF_MUX_SP_LOW          = 2; // Stack Pointer Low Byte
    parameter RF_MUX_SP_HIGH         = 3; // Stack Pointer High Byte
    parameter RF_MUX_SP_IMMED_LOW    = 4; // Stack Pointer + Immediate value Low Byte
    parameter RF_MUX_SP_IMMED_HIGH   = 5; // Stack Pointer + Immediate value High Byte
    parameter RF_MUX_IMMED_LOW       = 6; // Immediate value Low Byte
    parameter RF_MUX_IMMED_HIGH      = 7; // Immediate value High Byte
    parameter RF_MUX_DY              = 8; // DY output of the Reg File
    
    // 
    parameter MEM_ADDR_SP      = 0; // stack pointer output
    parameter MEM_ADDR_IMMED   = 1; // Immediate Value address input
    parameter MEM_ADDR_IMMED_1 = 2; // Immediate Value address input + 1  
    parameter MEM_ADDR_16_RF   = 3; // 16 bit output of the Reg File
    
    parameter MEM_DATA_DX      = 0; // DX output of the Reg File
    parameter MEM_DATA_PC      = 1; // PC value output 
    parameter MEM_DATA_FLAGS   = 2; // Flags Register values
    parameter MEM_DATA_SP_LOW  = 3; // Stack Pointer Low Byte output
    parameter MEM_DATA_SP_HIGH = 4; // Stack Pointer High Byte output
    
    parameter SP_DIN_RF_16 = 0; // 16 bit output of Reg File 
    parameter SP_DIN_IMMED = 1; // Immediate value input
    
    parameter FLAGS_DATA_ALU = 0; // ALU Flags Output
    parameter FLAGS_DATA_MEM = 1;  // Memory Flags Output 
    
    parameter ALU_B_MUX_DY   = 0; // DY output of the Reg File
    parameter ALU_B_MUX_MEM  = 1; // Memory output
    parameter ALU_B_MUX_PROG = 2; // PROGOM output

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

    localparam ADD_16_ALU = 2'b00;
    localparam INC_16_ALU = 2'b01;
    localparam DEC_16_ALU = 2'b10;

    
    typedef enum int {INIT, FETCH, EXEC, INTERRUPT, CB_EXEC, HL_EXEC, HL_FETCH, SP_LOW, SP_HIGH, IMMED, ALU16} STATE;

    STATE NS, PS = INIT;

     logic mcycle = 0;     
     // Flag used for identifying that NS after EXEC is SP_LOW
     logic SP_LOW_FLAG = 1'b0;
     // Flag used for identifying that NS after EXEC is SP_HIGH
     logic SP_HIGH_FLAG = 1'b0;
     // Used for saving the opcode into the SP state
     logic [7:0] SP_OPCODE = 8'h00;

     // Flag for CB prefixes
     logic CB_FLAG = 1'b0;
     // Flag for Immediate value usage
     logic IMMED_FLAG = 1'b0;
     // Flag for HL pointer state
     logic HL_FLAG = 1'b0;
  
     // FLag for 16 bit ALU state
     logic ALU_16=1'b0;
     logic [7:0]PS_OPCODE = 8'b00000000;
     
     // Store ALU function for HL States
     logic [4:0] HL_ALU_FUN = 5'b00000;
    
     logic IMMED_16_FLAG = 1'b0;
     // Flag for Immediate Low Byte Load
     logic LOW_IMMED = 1'b0;
     //Used for saving the last immediate address values
     logic [7:0] LAST_IMMED_ADDR_LOW, LAST_IMMED_ADDR_HIGH;
     //Used for saving the last immediate address values
     logic [7:0] LAST_IMMED_DATA_LOW, LAST_IMMED_DATA_HIGH;
     // Flags for PUSH and POP
     logic POP_FLAG = 1'b0;
     logic PUSH_FLAG = 1'b0;       
     // Immediate Value Select 
     logic [7:0] IMMED_SEL = 8'h00;       
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
        SP_LD=0; SP_INCR=0; SP_DECR=0; SP_DIN_SEL = 0;
        MEM_WE=0; MEM_RE = 0; MEM_DATA_SEL=0; MEM_ADDR_SEL=0; 
        IMMED_ADDR_LOW = 0; IMMED_ADDR_HIGH = 0;
        IMMED_DATA_LOW = 0; IMMED_DATA_LOW = 0;
        ALU_OPY_SEL=0; ALU_OPX_SEL = 0; ALU_SEL=0;
        ALU_16_A_SEL=0; ALU_16_B_SEL=0; 
        C_FLAG_LD = 0; C_FLAG_SET = 0; C_FLAG_CLR = 0; 
        Z_FLAG_LD = 0; Z_FLAG_SET = 0; Z_FLAG_CLR = 0; 
        N_FLAG_LD = 0; N_FLAG_SET = 0; N_FLAG_CLR = 0; 
        H_FLAG_LD = 0; H_FLAG_SET = 0; H_FLAG_CLR = 0; FLG_LD_SEL = 0;  
        HL_FLAG = 0; BIT_SEL = 0;

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
                else if (IMMED_FLAG == 1)
                    begin
                        NS = IMMED;
                    end               
                else
                    NS = EXEC;
            end

            EXEC:
            begin              
                        
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
                    
                    // ============== Takes 8 cycles ============== //
                    // Load Stack Pointer Value into 16 bit immediate address location
                    8'b00001000: // LD (a16), SP
                    begin                               
                        // Set the Immediate flag high to transition to the Immediate state after the next fetch
                        IMMED_FLAG = 1'b1;                        
                        // Set the Immediate Select to the OPCODE
                        IMMED_SEL = OPCODE;
                    end
                    
                    // 16 bit Immediate Loads: nn , 16-immediate = d16; SP, 16-immediate = d16
                    8'b00??0001:
                    begin
                        // Set the Immediate Select to the OPCODE
                        IMMED_SEL = OPCODE; 
                        // Set the Immediate flag high to transition to the Immediate state after the next fetch
                        IMMED_FLAG = 1'b1;      
                    end
                    
                    //
                    // 8-bit loads
                    //
                    
                    8'b00??0100: // INC B, D, H, (HL)
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 3'b000;                                
                        // ALU Operation Select
                        ALU_SEL = INC_ALU;                                
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
                        case (OPCODE[5:4])
                            2'b00: // INC B
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_B;
                            end
                            
                             2'b01: // INC D
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_D;
                            end
                            
                             2'b10: // INC H
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_H;
                            end
                            
                             2'b11: // INC (HL) ////////////=========== Update with (HL) code
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_HL;
                            end
                        endcase
                    end
                    
                    8'b00??0101: // DEC B, D, H, (HL)
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 3'b000;                                
                        // ALU Operation Select
                        ALU_SEL = DEC_ALU;                                
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
                        case (OPCODE[5:4])
                            2'b00: // DEC B
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_B;
                            end
                            
                             2'b01: // DEC D
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_D;
                            end
                            
                             2'b10: // DEC H
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_H;
                            end
                            
                             2'b11: // DEC (HL) ////////////=========== Update with (HL) code
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_HL;
                            end
                        endcase
                    end

                    8'b00??1100: // INC C, E, L, A
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 3'b00;                                
                        // ALU Operation Select
                        ALU_SEL = INC_ALU;                                
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
                        case (OPCODE[5:4])
                            2'b00: // INC C
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_C;
                            end
                            
                             2'b01: // INC E
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_E;
                            end
                            
                             2'b10: // INC L
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_L;
                            end
                            
                             2'b11: // INC A 
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_A;
                            end
                        endcase
                    end
                    
                     8'b00??1101: // DEC C, E, L, A
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
                        // ALU Operation Select
                        ALU_SEL = DEC_ALU;                                
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
                        case (OPCODE[5:4])
                            2'b00: // DEC C
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_C;
                            end
                            
                             2'b01: // DEC E
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_E;
                            end
                            
                             2'b10: // DEC L
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_L;
                            end
                            
                             2'b11: // DEC A 
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_A;
                            end
                        endcase
                    end
                    
                    8'b00000111: // RLCA
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RLC_ALU;                                
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
                    end
                    
                    8'b00010111: // RLA
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RL_ALU;                                
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
                    end
                    
                    8'b00001111: // RRCA
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RRC_ALU;                                
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
                    end
                    
                    8'b00011111: // RRA
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = RR_ALU;                                
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
                    end
                    
                    
                    8'b00101111: // CPL
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
                        // ALU Operation Select
                        ALU_SEL = CPL_ALU;                                
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
                    end
                    
                    8'b00100111: // DAA
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b00;                                
                        // ALU Operation Select
                        ALU_SEL = DAA_ALU;                                
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
                    end
                    
                    8'b00110111: // SCF
                    begin
                        // Flags
                        C_FLAG_SET = 1;
                        Z_FLAG_LD = 0;
                        N_FLAG_CLR = 1;
                        H_FLAG_CLR = 1;
                    end

                    8'b00111111: // CCF
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
                            // RF_WR = 0;
                            IMMED_FLAG = 1;
                            IMMED_SEL = OPCODE;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                        ALU_OPY_SEL = 0;                                
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

                        // ADD A, (HL) 
                        if (OPCODE[2:0] == 3'b110)
                        begin
                            HL_ALU_FUN = ADD_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                          
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            // Reset control lines
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                         
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                       
                    end
                    
                    8'b10001???:  // ADC A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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

                        // ADC A, (HL)
                        if (OPCODE[2:0] == 3'b110)
                        begin
                            HL_ALU_FUN = ADC_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                       
                    end
                    
                    8'b10100???:  // AND A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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

                        // AND A, (HL) 
                        if (OPCODE[2:0] == 3'b110)

                        begin
                            HL_ALU_FUN = AND_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                     
                    end
                    
                    8'b10111???:  // CP A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
                        // ALU Operation Select
                        ALU_SEL = CP_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Does not write operation back into Register A
                        RF_WR = 0;                                
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
                            HL_ALU_FUN = CP_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                         
                    end
                    
                     8'b10110???:  // OR A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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
                            HL_ALU_FUN = 5'b00101;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 3'b001; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                        
                    end
                    
                    8'b10011???:  // SBC A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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

                        // SBC A, (HL)
                        if (OPCODE[2:0] == 3'b110)
                        begin
                            HL_ALU_FUN = SBC_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                            
                    end
                    
                    
                    8'b10010???:  // SUB A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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
                            HL_ALU_FUN = SUB_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end                                                    
                    end
                    
                    8'b10101???:  // XOR A, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
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
                           
                        // XOR A, (HL)
                        if (OPCODE[2:0] == 3'b110)
                        begin
                            HL_ALU_FUN = XOR_ALU;                      
                            RF_ADRX = REG_H;
                            RF_ADRY = REG_L;
                            
                            MEM_ADDR_SEL = 3'b011;
                            MEM_RE = 1;
                            // ALU B input mux select
                            ALU_OPY_SEL = 2'b01; // Select data from memory
                            RF_WR = 0;
                            C_FLAG_LD = 0;
                            Z_FLAG_LD = 0;
                            N_FLAG_LD = 0;
                            H_FLAG_LD = 0;                            
                            
                            HL_FLAG = 1;
                            NS = HL_FETCH;
                        end   
                    end  
                    
                    8'b11??0001: // POP
                    begin
                        POP_FLAG = 1'b1;
                        SP_OPCODE = OPCODE;
                        SP_HIGH_FLAG = 1'b1;                    
                    end       
                    
                    8'b11??0101: // PUSH // ======================== Might need to send to wait state for consistent timing ======================== //
                    begin
                        PUSH_FLAG = 1'b1;                        
                        SP_OPCODE = OPCODE;
                        SP_LOW_FLAG = 1'b1;
                        // Decrement before pushing
                        SP_DECR = 1'b1;
                                                       
                    end             
                    
                    8'b11??0110: // ADD, SUB, AND, OR with Immediate values 
                    begin
                        // Save opcode for the next cycle 
                        IMMED_SEL = OPCODE;  
                        // Set the Immediate flag high to transition to the Immediate state after the next fetch
                        IMMED_FLAG = 1'b1;                                                             
                    end   
                    
                    8'b11??1110: // ADC, SBC, XOR, CP with Immediate values 
                    begin
                        // Save opcode for the next cycle
                        IMMED_SEL = OPCODE;
                        // Set the Immediate flag high to transition to the Immediate state after the next fetch
                        IMMED_FLAG = 1'b1;
                    end
                    
                    8'b11001011: // CB Prefix command
                    begin
                        CB_FLAG = 1'b1;
                    end                    
                    
                    // =========== Only takes 4 cycle instead of 6 ============ //
                    8'b11111000: // LD HL, SP + r8
                    begin                               
                        // Save opcode for the next cycle
                        IMMED_SEL = OPCODE;
                        // Set the Immediate flag high to transition to the Immediate state after the next fetch
                        IMMED_FLAG = 1'b1;
                    end
                    
                    // =========== Only takes 1 cycle instead of 2 ============ //
                    8'b11111001: // LD SP, HL
                    begin                               
                        // Does not write operation back into Reg File
                        RF_WR = 1'b0;                                                      
                        // Register File Addresses
                        RF_ADRX = REG_H;
                        RF_ADRY = REG_L;                        
                        // Stack Pointer Data Select set to the 16-bit Reg File Output
                        SP_DIN_SEL = SP_DIN_RF_16;
                        // Load the HL values into the Sack Pointer
                        SP_LD = 1'b1;
                    end
                    
                    //16 bit ALU
                    8'b00??0011: // INC BC, DE, HL, SP
                    begin
                        ALU_16_SEL = INC_16_ALU;
                         case (OPCODE[5:4])
                            2'b00: // INC BC
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_B;
                                RF_ADRY = REG_C;
                            end
                            
                             2'b01: // INC DE
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_D;
                                RF_ADRY = REG_E;
                            end
                            
                             2'b10: // INC HL
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_H;
                                RF_ADRY = REG_L;
                            end
                            
                             2'b11: // INC SP??
                            begin
                                // Register File Addresses
                                
                            end
                        endcase
                        RF_WR_SEL = 4'b0011; //ALU16_OUT[15:8]                                
                        // Write operation back into Register 
                        RF_WR = 1; 
                        PS_OPCODE= OPCODE;
                        ALU_16=1'b1;
                    end
                    
                    8'b00??1011: // DEC BC, DE, HL, SP
                    begin
                                   
                       ALU_16_SEL = DEC_16_ALU;
                         case (OPCODE[5:4])
                            2'b00: // INC BC
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_B;
                                RF_ADRY = REG_C;
                            end
                            
                             2'b01: // INC DE
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_D;
                                RF_ADRY = REG_E;
                            end
                            
                             2'b10: // INC HL
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_H;
                                RF_ADRY = REG_L;
                            end
                            
                             2'b11: // INC SP??
                            begin
                                // Register File Addresses
                                
                            end
                        endcase
                        
                        RF_WR_SEL = 4'b0011; //ALU16_OUT[15:8]                                
                        // Write operation back into Register 
                        RF_WR = 1; 
                        
                        PS_OPCODE= OPCODE;
                        ALU_16=1'b1;
                    end
                    
                    8'b00??1001: // ADD HL to BC, DE, HL, and SP
                    begin
                        C_FLAG_LD = 1;
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 0;                                
                        // ALU Operation Select
                        ALU_SEL = ADD_ALU;  // ADD Lower byte first                               
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register L
                        RF_WR = 1;                                
                        // Flag register data select
                        FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                        // Register File Addresses
                        RF_ADRX = REG_L;
                        case (OPCODE[5:4])
                            2'b00: // ADD C to L
                            begin
                                // Register File Addresses
                                RF_ADRY = REG_C;
                            end
                            
                             2'b01: // ADD E to L
                            begin
                                // Register File Addresses
                                RF_ADRY = REG_E;
                            end
                            
                             2'b10: // ADD L to L
                            begin
                                // Register File Addresses
                                RF_ADRY = REG_L;
                            end
                            
                             2'b11: // ADD SPam ??? do i need this
                            begin
                                // Register File Addresses
                                RF_ADRX = REG_A;
                            end
                          endcase
                        
                        ALU_16=1'b1;
                        PS_OPCODE= OPCODE;
                    end
                    
                    default: begin
                        // literally crashes on a real game boy
                    end
                    8'b11000011: begin //JP (nn), jump to address nn = two byte imeediate value (opcode C3) 
                        PC_LD = 1; 
                    end
                    
                    8'b11000010: begin // JP cc, nn cc = NZ jump if Z flag is reset. (opcode C2)
                        if(Z == 0) PC_LD = 1;                        
                    end
                    8'b11001010: begin //JP if Z flag is set (opcode CA)
                        if(Z == 1) PC_LD = 1;
                    end
                    8'b11010010: begin //JP if C flag is reset (opcode D2)
                        if(C == 1) PC_LD = 1;
                    end
                    8'b11011010: begin // JP if C flat is set (opcode DA)
                        if(C == 0) PC_LD = 1;
                    end
                    8'b11101001: begin //JP to address contained in HL (opcode E9)
                    end
                    8'b00011000: begin //JR: add n to current address and jump to it (opcode 18)
                    end
                    8'b00100000: begin //JR cc, n: if Z flag is reset, add n to current address and jump to it (opcode 20)
                    end
                    8'b00101000: begin //JR : if Z flag is set, add n to current address and jump to it (opcode 28)
                    end
                    8'b00110000: begin //JR : if C flag is reset, add n to current address and jump to it (opcode 30)
                    end
                    8'b00111000: begin //JR : if C flag is set, add n to current address and jump to it (opcode 38)
                    end
                    8'b11001101: begin //CALL : push addr of next instruction onto stack and then jump to address nn (opcode CD)
                    end
                    8'b11000100: begin //CALL cc, nn : call address n if Z flag is reset (opcode C4)
                    end
                    8'b11001100: begin //CALL cc, nn : call address n if Z flag is set (opcode CC)
                    end
                    8'b11010100: begin //CALL cc, nn : call address n if C flag is reset(opcode D4)
                    end
                    8'b11011100: begin //CALL cc, nn: call address n if C flag is set (opcode DC)
                    end
                

                endcase // OPCODE
                    
                if (INTR)
                    NS = INTERRUPT;       
                else // Maybe add begin and end to this
                    if (SP_LOW_FLAG) // Transition to the SP sate is the Next State Stack Pointer flag is high
                        NS = SP_LOW;
                    else if(HL_FLAG) // Transition to HL state
                        NS = HL_FETCH;
                    else if (SP_HIGH_FLAG) // Transition to the SP sate is the Next State Stack Pointer flag is high
                        NS = SP_HIGH;
                    else if (ALU_16)
                        NS= ALU16;
                    else 
                        NS = FETCH;
            end // EXEC
            
            ALU16: begin // 16 bit ALU
            
            case (PS_OPCODE) inside
            
                8'b00??0011: // INC
                begin
                
                // ALU Operation Select
                ALU_OPY_SEL = 3'b001;  
                ALU_SEL = INC_ALU;                                
                // Input to the Reg File is the ALU output
                RF_WR_SEL = RF_MUX_ALU;                                
                // Write operation back into Register A
                RF_WR = 1;  
                
                // Flag register data select
                FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                case (PS_OPCODE[5:4])
                    2'b00: // INC C
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_C;
                    end
                    
                     2'b01: // INC E
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_E;
                    end
                    
                     2'b10: // INC L
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_L;
                    end
                    
                     2'b11: // INC SP??
                    begin
                        // Register File Addresses
                        
                    end
                endcase
                        
                
                RF_WR = 1;        
                end
                
                8'b00??1011: // DEC BC, DE, HL, SP
                begin
                    
                // ALU Operation Select
                ALU_OPY_SEL = 3'b001;  
                ALU_SEL = DEC_ALU;                                
                // Input to the Reg File is the ALU output
                RF_WR_SEL = RF_MUX_ALU;                                
                // Write operation back into Register A
                RF_WR = 1;  
                
                // Flag register data select
                FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                case (PS_OPCODE[5:4])
                    2'b00: // INC C
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_C;
                    end
                    
                     2'b01: // INC E
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_E;
                    end
                    
                     2'b10: // INC L
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_L;
                    end
                    
                     2'b11: // INC SP??
                    begin
                        // Register File Addresses
                        
                    end
                endcase
                        
                RF_WR = 1;      
                end
                    
                8'b00??1001: // ADD HL to BC, DE, HL, and SP
                begin
                // ALU A input mux select                                
                ALU_OPX_SEL = 1'b0;
                // ALU B input mux select
                ALU_OPY_SEL = 2'b00;                                
                // ALU Operation Select
                ALU_SEL = ADC_ALU;      //ADC the upper byte                          
                // Input to the Reg File is the ALU output
                RF_WR_SEL = RF_MUX_ALU;                                
                // Write operation back into Register A
                RF_WR = 1;          // write to H                      
                // Flags
                C_FLAG_LD = 1;
                Z_FLAG_LD = 1;
                N_FLAG_LD = 1;
                H_FLAG_LD = 1;
                // Flag register data select
                FLAGS_DATA_SEL = FLAGS_DATA_ALU;
                // Register File Addresses
                RF_ADRX = REG_H;
                                                              
                case (PS_OPCODE[5:4]) 
                    2'b00: // ADD B to H
                    begin 
                        // Register File Addresses
                        RF_ADRY = REG_B;
                    end
                    
                     2'b01: // ADD D to H
                    begin
                        // Register File Addresses
                        RF_ADRY = REG_D;
                    end
                    
                     2'b10: // ADD H to H
                    begin
                        // Register File Addresses
                        RF_ADRY = REG_H;
                    end
                    
                     2'b11: // ADD SPam ??? do i need this
                    begin
                        // Register File Addresses
                        RF_ADRX = REG_A;
                    end
                    endcase
                end
            endcase
            ALU_16=1'b0;
            NS= FETCH;
            end // ALU16
            
            IMMED: begin  // Immediate Value Instrutions
                // Same for each ALU case, LD cases will overwrite when necessary
                // ALU A input mux select                                
                ALU_OPX_SEL = 1'b0;
                // ALU B input mux select
                ALU_OPY_SEL = ALU_B_MUX_PROG;                                
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
                
                case(IMMED_SEL) inside

                    8'b00001000: // LD (a16), SP
                    begin
                        // Reg File does not write
                        RF_WR = 0; 
                        //HIGH_IMMED = 
                        // Flag for Immediate Low Byte Load
                        LOW_IMMED = ~LOW_IMMED ? 1'b1 : 1'b0;
                        // Set the IMMED_ADDR_LOW output value to the immediate value (OPCODE) if the LOW_IMMED flag is high
                        IMMED_ADDR_LOW = LOW_IMMED ? OPCODE : LAST_IMMED_ADDR_LOW;
                        // Saves the new Immediate Value Address Low Byte for writing
                        LAST_IMMED_ADDR_LOW = IMMED_ADDR_LOW;                       
                        // Set the IMMED_ADDR_HIGH output value to the immediate value (OPCODE) if the LOW_IMMED flag is low
                        IMMED_ADDR_HIGH = ~LOW_IMMED ?  OPCODE : LAST_IMMED_ADDR_HIGH;
                        // Saves the new Immediate Value Address High Byte for writing
                        LAST_IMMED_ADDR_HIGH = IMMED_ADDR_HIGH;
                        // Set the Stack Pointer Low Flag to Write the Low byte of the Stack Pointer Data
                        SP_LOW_FLAG = ~LOW_IMMED ? 1'b1 : 1'b0;
                    end
                    8'b00???110: begin  // LD r, n
                        RF_WR = 1;
                        RF_WR_SEL = RF_MUX_IMMED_LOW;
                        IMMED_DATA_LOW = OPCODE;
                        RF_ADRX = IMMED_SEL[5:3];
                    end
                    
                    8'b00??0001: // 16 bit Immediate Loads: nn , 16-immediate = d16; SP, 16-immediate = d16
                    begin
                        // Flag for Immediate Low Byte Load
                        LOW_IMMED = ~LOW_IMMED ? 1'b1 : 1'b0;
                        // Flag for 16 bit Immediates
                        IMMED_16_FLAG = LOW_IMMED ? 1'b1 : 1'b0;
                        case(IMMED_SEL[5:4])                            
                            2'b00: // LD BC, d16
                            begin
                                // Reg File writes either the High or Low Byte
                                RF_WR = 1'b1; 
                                // Set the IMMED_DATA_LOW output value to the immediate value (OPCODE) if the LOW_IMMED flag is high
                                IMMED_DATA_LOW = LOW_IMMED ? OPCODE : LAST_IMMED_DATA_LOW;
                                // Saves the new Immediate Value Data Low Byte for writing
                                LAST_IMMED_DATA_LOW = IMMED_DATA_LOW;                       
                                // Set the IMMED_DATA_HIGH output value to the immediate value (OPCODE) if the LOW_IMMED flag is low
                                IMMED_DATA_HIGH = ~LOW_IMMED ?  OPCODE : LAST_IMMED_DATA_HIGH;
                                // Saves the new Immediate Value Data High Byte for writing
                                LAST_IMMED_DATA_HIGH = IMMED_DATA_HIGH;
                                // Write the High Byte to B and the Low Byte to C
                                RF_ADRX = ~LOW_IMMED ? REG_B : REG_C;
                                // Load the Stack Pointer with the High and Low Byte immediate values
                                RF_WR_SEL = LOW_IMMED ? RF_MUX_IMMED_LOW : RF_MUX_IMMED_HIGH;
                            end
                            
                            2'b01: // LD DE, d16
                            begin
                                // Reg File writes either the High or Low Byte
                                RF_WR = 1'b1; 
                                // Set the IMMED_DATA_LOW output value to the immediate value (OPCODE) if the LOW_IMMED flag is high
                                IMMED_DATA_LOW = LOW_IMMED ? OPCODE : LAST_IMMED_DATA_LOW;
                                // Saves the new Immediate Value Data Low Byte for writing
                                LAST_IMMED_DATA_LOW = IMMED_DATA_LOW;                       
                                // Set the IMMED_DATA_HIGH output value to the immediate value (OPCODE) if the LOW_IMMED flag is low
                                IMMED_DATA_HIGH = ~LOW_IMMED ?  OPCODE : LAST_IMMED_DATA_HIGH;
                                // Saves the new Immediate Value Data High Byte for writing
                                LAST_IMMED_DATA_HIGH = IMMED_DATA_HIGH;
                                // Write the High Byte to D and the Low Byte to E
                                RF_ADRX = ~LOW_IMMED ? REG_D : REG_E;
                                // Load the Stack Pointer with the High and Low Byte immediate values
                                RF_WR_SEL = LOW_IMMED ? RF_MUX_IMMED_LOW : RF_MUX_IMMED_HIGH; 
                            end
                            
                            2'b10: // LD HL, d16
                            begin
                                // Reg File writes either the High or Low Byte
                                RF_WR = 1'b1; 
                                // Set the IMMED_DATA_LOW output value to the immediate value (OPCODE) if the LOW_IMMED flag is high
                                IMMED_DATA_LOW = LOW_IMMED ? OPCODE : LAST_IMMED_DATA_LOW;
                                // Saves the new Immediate Value Data Low Byte for writing
                                LAST_IMMED_DATA_LOW = IMMED_DATA_LOW;                       
                                // Set the IMMED_DATA_HIGH output value to the immediate value (OPCODE) if the LOW_IMMED flag is low
                                IMMED_DATA_HIGH = ~LOW_IMMED ?  OPCODE : LAST_IMMED_DATA_HIGH;
                                // Saves the new Immediate Value Data High Byte for writing
                                LAST_IMMED_DATA_HIGH = IMMED_DATA_HIGH;
                                // Write the High Byte to H and the Low Byte to L
                                RF_ADRX = ~LOW_IMMED ? REG_H : REG_L;
                                // Load the Stack Pointer with the High and Low Byte immediate values
                                RF_WR_SEL = LOW_IMMED ? RF_MUX_IMMED_LOW : RF_MUX_IMMED_HIGH; 
                            end  
                              
                            2'b11: // LD SP, d16
                            begin
                                // Reg File does not write
                                RF_WR = 1'b0; 
                                // Set the IMMED_DATA_LOW output value to the immediate value (OPCODE) if the LOW_IMMED flag is high
                                IMMED_DATA_LOW = LOW_IMMED ? OPCODE : LAST_IMMED_DATA_LOW;
                                // Saves the new Immediate Value Data Low Byte for writing
                                LAST_IMMED_DATA_LOW = IMMED_DATA_LOW;                       
                                // Set the IMMED_DATA_HIGH output value to the immediate value (OPCODE) if the LOW_IMMED flag is low
                                IMMED_DATA_HIGH = ~LOW_IMMED ?  OPCODE : LAST_IMMED_DATA_HIGH;
                                // Saves the new Immediate Value Data High Byte for writing
                                LAST_IMMED_DATA_HIGH = IMMED_DATA_HIGH;
                                // Load the Stack Pointer with the High and Low Byte immediate values
                                SP_LD = ~LOW_IMMED ? 1'b1 : 1'b0;
                                // Set the Stack Pointer input 
                                SP_DIN_SEL = SP_DIN_IMMED;
                            end                        
                        endcase
                    end
                    
                    8'b11111000: // LD HL, SP + r8
                    begin
                        // Reg File writes either the High or Low Byte
                        RF_WR = 1'b1; 
                        // Set the IMMED_DATA_LOW output value to the immediate value (OPCODE) 
                        IMMED_DATA_LOW = OPCODE;
                        // Saves the new Immediate Value Data Low Byte for writing to the H register
                        LAST_IMMED_DATA_LOW = IMMED_DATA_LOW;
                        // Set Reg File DIN select to the Stack Pointer + immediate value
                        RF_WR_SEL = RF_MUX_SP_IMMED_LOW; 
                        // Write the Low Byte to register L
                        RF_ADRX = REG_L;   
                        // Set the Stack Pointer High Flag to Write the High byte of the Stack Pointer + Immediate Data
                        SP_HIGH_FLAG = 1'b1;                                           
                    end
                    
                    8'b11??0110: // ADD, SUB, AND, OR with Immediate values
                    begin 
                        case(IMMED_SEL[5:4])
                        
                            2'b00: // ADD A, immed
                            begin
                                // ALU Operation Select
                                ALU_SEL = ADD_ALU; 
                            end
                            
                            2'b01: // SUB A, immed
                            begin
                                // ALU Operation Select
                                ALU_SEL = SUB_ALU;    
                            end
                            
                            2'b10: // AND A, immed
                            begin
                                //ALU Operation Select
                                 ALU_SEL = AND_ALU;
                            end
                            
                            2'b11:  // OR A, immed
                            begin
                                // ALU Operation Select
                                 ALU_SEL = OR_ALU;
                            end                                
                        endcase
                    end
                    
                    8'b11??1110: // ADC, SBC, XOR, CP with Immediate values 
                    begin 
                        case(IMMED_SEL[5:4])
                            2'b00: // ADC A, immed
                            begin
                                // ALU Operation Select
                                ALU_SEL = ADC_ALU; 
                            end
                            
                            2'b01: // SBC A, immed
                            begin
                                // ALU Operation Select
                                ALU_SEL = SBC_ALU;    
                            end
                            
                            2'b10: // XOR A, immed
                            begin
                                //ALU Operation Select
                                 ALU_SEL = XOR_ALU;
                            end
                            
                            2'b11:  // CP A, immed
                            begin
                                // ALU Operation Select
                                ALU_SEL = CP_ALU;
                                // Does not write operation back into Register A
                                RF_WR = 0; 
                            end  
                        endcase               
                    end                                     
                endcase
                // Reset Immediate Flag if the value is not LD (a16), SP and transition back to the fetch state
                IMMED_FLAG = IMMED_SEL == (8'b00001000) || IMMED_16_FLAG && ~SP_LOW_FLAG ? 1'b1 : 1'b0;
                // Transition to the SP_LOW state if the Stack Pointer Low Flag is High or the SP_HIGH state if the Stack Pointer High Flag is High
                if (SP_LOW_FLAG)
                    NS = SP_LOW;
                else if (SP_HIGH_FLAG)
                    NS = SP_HIGH;
                else
                    NS = FETCH;
            end // IMMED
            
            
            SP_LOW: begin   // Stack Pointer Low Byte state
                if (PUSH_FLAG) // Pushes the Low Byte and then the High Byte
                    begin
                        // Register File does not write on a PUSH
                        RF_WR = 0;
                        // The Stack Pointer  is decremented when pushing the Low Byte
                        SP_DECR = 1'b1; 
                        // Memory address select set to SP
                        MEM_ADDR_SEL = MEM_ADDR_SP;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_DX;
                        // Write the pushed value to memory &(SP)
                        MEM_WE = 1'b1;
                        // Low Byte used set by RF_ADRX except for the Flag Register Values
                        case (SP_OPCODE[5:4])
                            2'b00: // BC
                            begin                                                          
                                RF_ADRX = REG_C;                                
                            end
                            
                            2'b01: // DE
                            begin
                                RF_ADRX = REG_E;                        
                            end
                            
                            2'b10: // HL
                            begin
                                RF_ADRX = REG_L;
                            end
                            
                            2'b11: // AF 
                            begin
                                // Memory Data select set to the Flag Register values (Low Byte)
                                MEM_DATA_SEL = MEM_DATA_FLAGS;
                            end
                        endcase                                
                        // Reset the Stack Pointer Low Byte Flag
                        SP_LOW_FLAG = 1'b0;
                        // Transition to the Stack Pointer High Byte state to push the High Byte
                        NS = SP_HIGH;                                                                                              
                    end       
                        
                else if (POP_FLAG)  // Pops the High Byte and then the Low Byte
                    begin
                        // Reg File select set to memory
                        RF_WR_SEL = RF_MUX_MEM;
                        // Register File writes on a POP
                        RF_WR = 1'b1; 
                        // SP is incremented after popping the Low Byte
                        SP_INCR = 1'b1;                       
                        // Memory address select set to SP
                        MEM_ADDR_SEL = MEM_ADDR_SP;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_DX;
                        // Read the popped value from memory &(SP)
                        MEM_RE = 1'b1;
                        // Low Byte used set by RF_ADRX except for the Flag Register Values
                        case (SP_OPCODE[5:4])
                                2'b00: // BC
                                begin                                                                   
                                    RF_ADRX = REG_C;                                    
                                end
                                
                                2'b01: // DE
                                begin                                    
                                    RF_ADRX = REG_E;
                                end
                                
                                2'b10: // HL
                                begin
                                    RF_ADRX = REG_L;
                                end
                                
                                2'b11: // AF
                                begin
                                    // Register File does not write when popping the Flag Register values
                                    RF_WR = 1'b0;                                   
                                    // Load the popped Low Byte values from the stack into the flag register
                                    C_FLAG_LD = 1'b1;
                                    Z_FLAG_LD = 1'b1;
                                    N_FLAG_LD = 1'b1;
                                    H_FLAG_LD = 1'b1; 
                                    // Flag register data select
                                    FLAGS_DATA_SEL =  FLAGS_DATA_MEM;                                   
                                    // Memory Data select set to Flag Register values
                                    MEM_DATA_SEL = MEM_DATA_FLAGS;
                                end
                            endcase                                               
                        // Reset the Stack Pointer Low flag and the POP flag
                        
                        SP_LOW_FLAG = 1'b0;
                        POP_FLAG = 1'b0;
                        // Transition to the FETCH state once both bytes are popped
                        NS = FETCH;             
                end
                // LD (a16), SP ( Write Low Byte)
                else                
                    begin
                        // Set the Immediate address values to the proper outouts
                        IMMED_ADDR_LOW = LAST_IMMED_ADDR_LOW;
                        IMMED_ADDR_HIGH = LAST_IMMED_ADDR_HIGH;
                        // Memory address select set to IMMED
                        MEM_ADDR_SEL = MEM_ADDR_IMMED;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_SP_LOW;
                        // Read the popped value from memory &(SP)
                        MEM_WE = 1'b1;
                        // Reset the Stack Pointer Low Byte Flag
                        SP_LOW_FLAG = 1'b0;
                        // Reset the Immediate Flag
                        IMMED_FLAG =1'b0;
                        // Transition to the SP_HIGH state to write the High byte of the Stack Pointer
                        NS = SP_HIGH;
                    end                            
            end // SP_LOW               
                
            SP_HIGH: begin   // Stack Pointer High Byte state
                if (PUSH_FLAG) // Pushes the Low Byte and then the High Byte
                    begin
                        // Register File does not write on a PUSH
                        RF_WR = 0;
                        // Memory address select set to SP
                        MEM_ADDR_SEL = MEM_ADDR_SP;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_DX;
                        // Write the pushed value to memory &(SP)
                        MEM_WE = 1'b1;
                        // High Byte used set by RF_ADRX
                        case (SP_OPCODE[5:4])
                            2'b00: // BC
                            begin                                                          
                                RF_ADRX = REG_B;                                
                            end
                            
                            2'b01: // DE
                            begin
                                RF_ADRX = REG_D;                        
                            end
                            
                            2'b10: // HL
                            begin
                                RF_ADRX = REG_H;
                            end
                            
                            2'b11: // AF 
                            begin
                                RF_ADRX = REG_A;
                            end
                        endcase                                
                        // Reset the Stack Pointer High Byte Flag and the PUSH Flag
                        SP_HIGH_FLAG = 1'b0;
                        PUSH_FLAG = 1'b0;
                        // Transition to the FETCH state once both bytes are pushed
                        NS = FETCH;                                                                                              
                    end       
                        
                else if (POP_FLAG)  // Pops the High Byte and then the Low Byte
                    begin
                        // Reg File select set to memory
                        RF_WR_SEL = RF_MUX_MEM;
                        // Register File writes on a POP
                        RF_WR = 1'b1;
                        // SP is incremented before popping the Low Byte
                        SP_INCR = 1'b1;
                        // Memory address select set to SP
                        MEM_ADDR_SEL = MEM_ADDR_SP;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_DX;
                        // Read the popped value from memory &(SP)
                        MEM_RE = 1'b1;
                        // High Byte used set by RF_ADRX
                        case (SP_OPCODE[5:4])
                                2'b00: // BC
                                begin                                                                   
                                    RF_ADRX = REG_B;                                    
                                end
                                
                                2'b01: // DE
                                begin                                    
                                    RF_ADRX = REG_D;
                                end
                                
                                2'b10: // HL
                                begin
                                    RF_ADRX = REG_H;
                                end
                                
                                2'b11: // AF
                                begin
                                    RF_ADRX = REG_A;
                                end
                            endcase                                                
                        // Reset the Stack Pointer High Byte Flag
                        SP_HIGH_FLAG = 1'b0;
                        // Transition to the Stack Pointer Low Byte state to pop the Low Byte
                        NS = SP_LOW;             
                    end 
                // LD HL, SP + r8    
                else if (IMMED_SEL == 8'b11111000) 
                    begin 
                        // Reg File writes either the High or Low Byte
                        RF_WR = 1'b1; 
                        // Set the IMMED_DATA_LOW output value to the Immediate value from the previous state 
                        IMMED_DATA_LOW = LAST_IMMED_DATA_LOW;                        
                        // Set Reg File DIN select to the Stack Pointer + immediate value High Byte
                        RF_WR_SEL = RF_MUX_SP_IMMED_HIGH; 
                        // Write the High Byte to register H 
                        RF_ADRX = REG_H; 
                        // Reset the Stack Pointer High Byte Flag
                        SP_HIGH_FLAG = 1'b0;
                        // Reset the Immediate Flag
                        IMMED_FLAG =1'b0;
                        // Transition to the FETCH once both bytes have been written
                        NS = FETCH;
                    end               
                // LD (a16), SP ( Write High Byte)
                else                
                    begin
                        // Set the Immediate address values to the proper outouts
                        IMMED_ADDR_LOW = LAST_IMMED_ADDR_LOW;
                        IMMED_ADDR_HIGH = LAST_IMMED_ADDR_HIGH;
                        // Memory address select set to IMMED
                        MEM_ADDR_SEL = MEM_ADDR_IMMED_1;
                        // Memory Data select set to DX output of the Reg File
                        MEM_DATA_SEL = MEM_DATA_SP_HIGH;
                        // Read the popped value from memory &(SP)
                        MEM_WE = 1'b1;
                        // Reset the Stack Pointer High Byte Flag
                        SP_HIGH_FLAG = 1'b0;
                        // Reset the Immediate Flag
                        IMMED_FLAG =1'b0;
                        // Transition to the FETCH state once both bytes have been written
                        NS = FETCH;
                    end                                     
             end // SP_HIGH
                
            
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
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
                                MEM_ADDR_SEL =  MEM_ADDR_16_RF;
                            end
                            
                            if (mcycle == 1) 
                            begin
                                RF_WR = 1;
                                RF_WR_SEL = RF_MUX_MEM;
                                RF_ADRX = OPCODE[5:3]; // r
                            end 
                        end                                                        
                    end
                    8'b01??????:  // BIT K, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0; // PLACEHOLDER
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b11; // PLACEHOLDER
                        BIT_SEL = OPCODE[5:3];                                 
                        // ALU Operation Select
                        ALU_SEL = BIT_ALU;   
                        // Flags
                        Z_FLAG_LD = 1;
                        N_FLAG_LD = 1;
                        H_FLAG_LD = 1;                            
                        // Register File Addresses
                        RF_ADRX = OPCODE[2:0]; 
                        // ADD HL CASE
                    end
                    8'b11??????:  // SET K, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b11;
                        BIT_SEL = OPCODE[5:3];                                
                        // ALU Operation Select
                        ALU_SEL = SET_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;                            
                        // Register File Addresses
                        RF_ADRX = OPCODE[2:0];
                        // ADD HL CASE
                    end
                    8'b10??????:  // RES K, n
                    begin
                        // ALU A input mux select                                
                        ALU_OPX_SEL = 1'b0;
                        // ALU B input mux select
                        ALU_OPY_SEL = 2'b11;                               
                        // ALU Operation Select
                        ALU_SEL = RES_ALU;                                
                        // Input to the Reg File is the ALU output
                        RF_WR_SEL = RF_MUX_ALU;                                
                        // Write operation back into Register A
                        RF_WR = 1;         
                        // Set bit selector
                        BIT_SEL = OPCODE[5:3];                   
                        // Register File Addresses
                        RF_ADRX = OPCODE[2:0];
                        // ADD HL CASE
                    end
                  
                    default: begin
                        // CRASHES
                    end
                endcase
                
                if (INTR == 1)
                    NS = INTERRUPT;
                    
                // Reset the CB Flag
                CB_FLAG = 1'b0;
                NS = FETCH;
                mcycle++;
            end // CB_EXEC
          
            HL_FETCH: begin
                NS = HL_EXEC;
                RF_ADRX = REG_H;
                RF_ADRY = REG_L;
                
                MEM_ADDR_SEL = 'b011;
                ALU_SEL = HL_ALU_FUN;
                ALU_OPY_SEL = 'b01;
                
            end
            HL_EXEC: begin
                NS = FETCH;
                ALU_SEL = HL_ALU_FUN;
                RF_ADRX = REG_A;
                ALU_OPY_SEL = 'b01;
                RF_WR = 1;
                C_FLAG_LD = 1;
                Z_FLAG_LD = 1;
                N_FLAG_LD = 1;
                H_FLAG_LD = 1;  
                
            end
        endcase // PS
    end
endmodule