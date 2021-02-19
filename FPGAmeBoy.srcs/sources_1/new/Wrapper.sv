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
    logic [1:0] RF_WR_SEL;
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
        .ALU_FUN(ALU_FUN), .A(RF_DX_OUT), .B(RF_DY_OUT), .FLAGS_IN(4'b0000),
        .ALU_OUT(RF_DIN), .FLAGS_OUT(ALU_FLAGS_OUT)
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
        .CLK(CLK), .INTR(), .RESET(RST),
        .C(C_FLAG), .Z(Z_FLAG), .N(N_FLAG), .H(H_FLAG), 
        .OPCODE(OPCODE), // Memory Line
        // Outputs
        .PC_LD(PC_LD), .PC_INC(PC_INC),     // program counter
        .PC_MUX_SEL(),                     // Unconnected
        .RF_WR(RF_WR),             // register file
        .RF_WR_SEL(),                       //Unconnected
        .RF_ADRX(RF_ADRX), .RF_ADRY(RF_ADRY),
        .ALU_SEL(ALU_FUN),     // ALU
        .ALU_OPY_SEL(),
        .SCR_DATA_SEL(), .SCR_WE(),  // scratch pad
        .SCR_ADDR_SEL(),
        .SP_LD(), .SP_INCR(), .SP_DECR(),   // stack pointer
        .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), // Flags 
        .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), // Z Flag control
        .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), // N Flag control
        .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), // H Flag control
        .I_CLR(), .I_SET(), .FLG_LD_SEL(), // interrupts
        .RST(RST),       // reset
        .IO_STRB()    // IO
    );
   
endmodule
