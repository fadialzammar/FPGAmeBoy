`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/12/2021 12:53:48 AM
// Design Name: 
// Module Name: OPCODES
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


// ===== AND ===== //
// 8 bit AND commands
`define AND_A    8'hA7;
`define AND_B    8'hA0;
`define AND_C    8'hA1;
`define AND_D    8'hA2;
`define AND_E    8'hA3;
`define AND_H    8'hA4;
`define AND_L    8'hA5;
// 8 bit AND with value @ HL address command  
`define AND_HL_ADDR    8'hA6;
// AND immediate command
`define AND_imm  8'hC6;

// ===== CALL ===== //
// 16 bit CALL commands
`define CALL_NZ   8'hC4;
`define CALL_Z    8'hCC;
`define CALL_NC   8'hD4;
`define CALL_C    8'hDC;
// AND immediate command
`define CALL_imm  8'hCD;

// ===== CCF ===== //
`define CCF     8'h3F;

// ===== CPL ===== //
`define CPL     8'h2F;

// ===== CP ===== //
// 8 bit CP commands
`define CP_A    8'hBF;
`define CP_B    8'hB8;
`define CP_C    8'hB9;
`define CP_D    8'hBA;
`define CP_E    8'hBB;
`define CP_H    8'hBC;
`define CP_L    8'hBD;
// CP immediate value
`define CP_imm  8'hFE;
// 8 bit CP of value @ HL address commands
`define CP_HL_ADDR   8'hBE;

// ===== DAA ===== //
`define DAA       8'h27;

// ===== DEC ===== //
// 8 bit DEC commands
`define DEC_A     8'h3D;
`define DEC_B     8'h05;
`define DEC_C     8'h0D;
`define DEC_D     8'h15;
`define DEC_E     8'h1D;
`define DEC_H     8'h25;
`define DEC_L     8'h2D;
// 8 bit DEC of value @ HL address 
`define DEC_HL_ADDR     8'h35
// 16 bit DEC commands
`define DEC_BC     8'h0B;
`define DEC_DE     8'h1B;
`define DEC_HL     8'h2B;
`define DEC_SP     8'h3B;

// ===== JP ===== //
`define JP_imm     8'hC3;
`define JP_NZ      8'hC2;
`define JP_Z       8'hCA;
`define JP_NC      8'hD2;
`define JP_C       8'hDA;
//  PC -> value @ HL address 
`define JP_HL_ADDR     8'h18;

// ===== JR ===== //
`define JR_NZ      8'h20;
`define JR_Z       8'h28;
`define JR_NC      8'h30;
`define JR_C       8'h38;
`define JR_imm     8'h18;

// ===== PUSH ===== //
`define PUSH_AF    8'hF5;
`define PUSH_BC    8'hC5;
`define PUSH_DE    8'hD5;
`define PUSH_HL    8'hE5;

//// Next 4 Roll ccmmands only for Register A  //// 

// ===== RLA ===== //
`define RLA     8'h17;

// ===== RLCA ===== //
`define RLCA    8'h07;

// ===== RRCA ===== //
`define RRCA    8'h0F;

// ===== RRA ===== //
`define RRA     8'h1F;

// ===== SBC ===== //
// 8 bit SBC commands
`define SBC_A   8'h9F;
`define SBC_B   8'h98;
`define SBC_C   8'h99;
`define SBC_D   8'h9A;
`define SBC_E   8'h9B;
`define SBC_H   8'h9C;
`define SBC_L   8'h9D;
// SBC immediate command
`define SBC_imm   8'hDE;
//  8 bit SBC of value @ HL address
`define SBC_HL_ADDR 8'h9E;

