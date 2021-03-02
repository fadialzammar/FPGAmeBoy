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
        output logic PC_LD, PC_INC,     // program counter
        output logic [1:0] PC_MUX_SEL,
        output logic RF_WR,             // register file
        output logic [1:0] RF_WR_SEL,
        output logic [2:0] RF_ADRX, RF_ADRY,
        output logic [3:0] ALU_SEL,     // ALU
        output logic ALU_OPY_SEL,
        output logic SCR_DATA_SEL, SCR_WE,  // scratch pad
        output logic [1:0] SCR_ADDR_SEL,
        output logic SP_LD, SP_INCR, SP_DECR,   // stack pointer
        output logic FLG_C_LD, FLG_C_SET, FLG_C_CLR, FLG_Z_LD,  // flags
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
    // parameter REG_HL = 3'b110;
    parameter REG_A = 3'b111;

    
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
        I_SET = 0; I_CLR =0; PC_LD=0; PC_INC=0; ALU_OPY_SEL=0; RF_WR=0; SP_LD=0; SP_INCR=0; SP_DECR=0;
        SCR_WE=0; SCR_DATA_SEL=0; FLG_C_SET=0; FLG_C_CLR=0; FLG_C_LD=0; FLG_Z_LD=0; FLG_LD_SEL=0;
        RST=0; PC_MUX_SEL=0; RF_WR_SEL=0; SCR_ADDR_SEL=0; ALU_SEL=0; IO_STRB = 0;

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

                if (INTR == 1)
                    NS = INTERRUPT;
                NS = FETCH;
            mcycle++;
            end // EXEC
        endcase // PS
    end


    
endmodule
