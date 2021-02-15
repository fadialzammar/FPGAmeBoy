// ===== ADC ===== //
// 8 bit ADC commands
`define ADC_A 8'h8F;
`define ADC_B 8'h88;
`define ADC_C 8'h89;
`define ADC_D 8'h8A;
`define ADC_E 8'h8B;
`define ADC_H 8'h8C;
`define ADC_L 8'h8D;
// ADC commands
`define ADC_HL_ADDR 8'h8E;
// Immediate ADC commands
`define ADC_IMM 8'hCE;

// ===== ADD ===== //
// 8 bit ADD commands
`define ADD_A 8'h87;
`define ADD_B 8'h80;
`define ADD_C 8'h81;
`define ADD_D 8'h82;
`define ADD_E 8'h83;
`define ADD_H 8'h84;
`define ADD_L 8'h85;
// Address Add
`define ADD_HL_ADDR 8'h86;
// 16 bit ADD
`define ADD_BC 8'h09;
`define ADD_DC 8'h19;
`define ADD_HHL 8'h29;
`define ADD_SP 8'h39;
// Immediate ADD commands
`define ADD_IMM 8'hC6;

// ===== AND ===== //
// 8 bit AND commands
`define AND_A 8'hA7;
`define AND_B 8'hA0;
`define AND_C 8'hA1;
`define AND_D 8'hA2;
`define AND_E 8'hA3;
`define AND_H 8'hA4;
`define AND_L 8'hA5;
// 16 bit AND command  
`define AND_HL 8'hA6;
// AND immediate command
`define AND# 8'hC6;
// 16 bit HL AND commands
`define AND_BC 8'h09;
`define AND_DC 8'h19;
`define AND_HHL 8'h29;
`define AND_SP 8'h39;

// ===== CALL ===== //
// 16 bit CALL commands
`define CALL_NZ 8'hC4;
`define CALL_Z  8'hCC;
`define CALL_NC 8'hD4;
`define CALLC  8'hDC;
// AND immediate command
`define CALL_IMM  8'hCD;

// ===== INTERRUPT ===== //
`define DI 8'hF3;
`define EI 8'hFB;

// ===== HALT ===== //
`define HALT 8'h76;

// ===== INCREMENT ===== //
`define INC_A 8'h3C;
`define INC_B 8'h04;
`define INC_C 8'h0C;
`define INC_D 8'h14;
`define INC_E 8'h1C;
`define INC_H 8'h24;
`define INC_L 8'h2C;

`define INC_HL_ADDR 8'h34;
// 16-bit increment
`define INC_BC 8'h03;
`define INC_DE 8'h13;
`define INC_HL 8'h23;
`define INC_SP 8'h33;

// ===== NOP ===== //
`define NOP 8'h00;

// ===== OR ===== //
`define OR_A 8'hB7;
`define OR_B 8'hB0;
`define OR_C 8'hB1;
`define OR_D 8'hB2;
`define OR_E 8'hB3;
`define OR_H 8'hB4;
`define OR_L 8'hB5;
`define OR_HL_ADDR 8'hB6;
// Immediate OR
`define OR_IMM 8'hF6;

// ===== POP ===== //
`define POP_AF 8'hF1;
`define POP_BC 8'hC1;
`define POP_DE 8'hD1;
`define POP_HL 8'hE1;

// ===== RETURN ===== //
`define RET 8'hC9;
`define RET_NZ 8'hC0;
`define RET_Z 8'hC8;
`define RET_NC 8'hD0;
`define RET_C 8'hD8;

`define RETI 8'hD9;

// ===== SUBTRACT ===== //
`define SUB_A 8'h97;
`define SUB_B 8'h90;
`define SUB_C 8'h91;
`define SUB_D 8'h92;
`define SUB_E 8'h93;
`define SUB_H 8'h94;
`define SUB_L 8'h95;
`define SUB_HL_ADDR 8'h9;
// Immediate
`define SUB_IMM 8'hD6;

// ===== SCF ===== //
`define SCF 8'h37;

// ===== STOP ===== //
`define STOP 8'h10;

// ===== XOR ===== //
`define XOR_A 8'hAF;
`define XOR_B 8'hA8;
`define XOR_C 8'hA9;
`define XOR_D 8'hAA;
`define XOR_E 8'hAB;
`define XOR_H 8'hAC;
`define XOR_L 8'hAD;
`define XOR_HL_ADDR 8'hAE;
// Immediate
`define XOR_IMM 8'hEE;





