`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/17/2021 09:06:22 PM
// Design Name: 
// Module Name: Wrapper
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


module Wrapper(
    input CLK,
    input RST
    );
    
    // Program Counter Signals
    logic PC_LD;
    logic [15:0] PC_DIN;
    logic [15:0] PC;
    
    // ProgRom Signals
    
    // ALU Signals
    logic [4:0] ALU_FUN;
    logic [7:0] ALU_A,ALU_B;
    logic [3:0] ALU_FLAGS_IN;

    logic [7:0] ALU_OUT; 
    logic [3:0] ALU_FLAGS_OUT;
    
    // RegFile Signals
    logic [4:0] RF_ADRX, RF_ADRY;
    logic [7:0] RF_DIN, RF_DX_OUT, RF_DY_OUT;
    logic RF_WR;

    // FlagReg Signals
    logic Z_IN,Z_FLAG_LD,Z_FLAG_SET,Z_FLAG_CLR,Z_FLAG;
    logic N_IN,N_FLAG_LD,N_FLAG_SET,N_FLAG_CLR,N_FLAG;
    logic H_IN,H_FLAG_LD, H_FLAG_SET,H_FLAG_CLR,H_FLAG;
    logic C_IN, C_FLAG_LD, C_FLAG_SET, C_FLAG_CLR, C_FLAG;
    
    // Control signals
        logic [7:0] OPCODE;
        logic PC_INC;            // program counter
        logic [1:0] PC_MUX_SEL;
        logic RF_WR;                // register file
        logic [1:0] RF_WR_SEL;
        logic [2:0] RF_ADRX, RF_ADRY;
        logic [4:0] ALU_SEL;          // ALU
        logic ALU_OPY_SEL;
        logic SCR_DATA_SEL, SCR_WE;     // scratch pad
        logic [1:0] SCR_ADDR_SEL;
        logic SP_LD, SP_INCR, SP_DECR;   // stack pointer
        
    ProgCount ProgCount( 
        .PC_CLK(CLK),
        .PC_RST(RST),
        .PC_LD(PC_LD),
        .PC_INC(PC_INC),
        .PC_DIN(PC_DIN),
        .PC_COUNT(PC)
    );
    
    ALU ALU(
        .ALU_FUN(ALU_FUN), .A(ALU_A), .B(ALU_B), .FLAGS_IN(ALU_FLAGS_IN),
        .ALU_OUT(ALU_OUT), .FLAGS_OUT(ALU_FLAGS_OUT)
    );
    
    Flag_Reg Flag_Reg(
        .CLK(CLK), .RST(RST),
        .Z(Z_IN), .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), .Z_FLAG(Z_FLAG),
        .N(N_IN), .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), .N_FLAG(N_FLAG),
        .H(H_IN), .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), .H_FLAG(H_FLAG),
        .C(C_IN), .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), .C_FLAG(C_FLAG)
    );
    RegFile RegFile(
        .ADRX(RF_ADRX), .ADRY(RF_ADRY), .DIN(RF_DIN),
        .DX_OUT(RF_DX_OUT), .DY_OUT(RF_DY_OUT), 
        .WE(RF_WR), .CLK(CLK)
    );
    
    ProgRom ProgRom(
        .PROG_CLK(CLK),
        .PROG_ADDR(PC),
        .PROG_IR(OPCODE)
    );
    
    ControlUnit ControlUnit(
        // Inputs
        .CLK(CLK), .INTR(0), .RESET(RST),
        .C(C_FLAG), .Z(Z_FLAG), .N(N_FLAG), .H(H_FLAG), 
        .OPCODE(), // Memory Line
        // Outputs
        .PC_LD(PC_LD), .PC_INC(PC_INC),     // program counter
        .PC_MUX_SEL(0),                     // Unconnected
        .RF_WR(RF_WR),             // register file
        .RF_WR_SEL(0),                       //Unconnected
        .RF_ADRX(RF_ADRX), .RF_ADRY(RF_ADRY),
        .ALU_SEL(ALU_FUN),     // ALU
        .ALU_OPY_SEL(0),
        .SCR_DATA_SEL(0), .SCR_WE(0),  // scratch pad
        .SCR_ADDR_SEL(0),
        .SP_LD(0), .SP_INCR(0), .SP_DECR(0),   // stack pointer
        .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), // Flags 
        .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), // Z Flag control
        .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), // N Flag control
        .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), // H Flag control
        .I_CLR(0), .I_SET(0), .FLG_LD_SEL(0), // interrupts
        .RST(RST),       // reset
        .IO_STRB(0)    // IO
    );
   
endmodule
