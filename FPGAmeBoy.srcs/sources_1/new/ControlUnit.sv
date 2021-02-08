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
    
    typedef enum int {INIT, FETCH, EXEC, INTERRUPT} STATE;

    STATE NS, PS = INIT;

    always_ff @(posedge CLK)
    begin
        if (RESET)
            PS <= INIT;
        else
            PS <= NS;
    end

    always_comb
    begin
        I_SET = 0; I_CLR =0; PC_LD=0; PC_INC=0; ALU_OPY_SEL=0; RF_WR=0; SP_LD=0; SP_INCR=0; SP_DECR=0;
        SCR_WE=0; SCR_DATA_SEL=0; FLG_C_SET=0; FLG_C_CLR=0; FLG_C_LD=0; FLG_Z_LD=0; FLG_LD_SEL=0;
        RST=0; PC_MUX_SEL=0; RF_WR_SEL=0; SCR_ADDR_SEL=0; ALU_SEL=0; IO_STRB = 0;

        case (PS)
            INIT:
            begin
                RST = 1;
                NS = FETCH;
            end

            FETCH:
            begin
                PC_INC = 1;
                NS = EXEC;
            end

            EXEC:
            begin
                if (INTR)
                    NS = INTERRUPT;
                else
                    NS = FETCH;

                
                if (OPCODE[7:6] == 2'b01) // LD r, r'
                begin
                    //filler
                end

                if (INTR == 1)
                    NS = INTERRUPT;
                NS = FETCH;
            end
        endcase
    end


    
endmodule
