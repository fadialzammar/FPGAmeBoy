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
    input CPU_RESET
    );
    
    // Program Counter Signals
    logic PC_LD;
    logic [15:0] PC_DIN;
    logic [15:0] PC;
    logic [15:0] CU_PC_ADDR;
    logic [15:0] REG_PC_ADDR;
    
    // ProgRom Signals
    
    // ALU Signals
    logic [4:0] ALU_FUN;
    logic [1:0] ALU_16_FUN;
    logic [7:0] ALU_A, ALU_B;
    logic [3:0] ALU_FLAGS_IN;
    logic [7:0] ALU_OUT; 
    logic [3:0] ALU_FLAGS_OUT;
    logic ALU_A_SEL;
    logic [2:0] ALU_B_SEL;
    // ALU 16 bit signals
    logic [15:0] ALU_16_A,ALU_16_B,ALU_16_OUT;
    logic [1:0] ALU_16_B_SEL;
    // RegFile Signals
    logic [4:0] RF_ADRX, RF_ADRY;
    logic [7:0] RF_DIN, RF_DX_OUT, RF_DY_OUT;
    logic RF_WR;
    logic [3:0] RF_DIN_SEL;
    // 16 bit output from the Reg File
    logic [15:0] RF_16_OUT;
    assign RF_16_OUT = {RF_DX_OUT, RF_DY_OUT};

    // FlagReg Signals
    logic Z_IN,Z_FLAG_LD,Z_FLAG_SET,Z_FLAG_CLR,Z_FLAG;
    logic N_IN,N_FLAG_LD,N_FLAG_SET,N_FLAG_CLR,N_FLAG;
    logic H_IN,H_FLAG_LD, H_FLAG_SET,H_FLAG_CLR,H_FLAG;
    logic C_IN, C_FLAG_LD, C_FLAG_SET, C_FLAG_CLR, C_FLAG;
    
    localparam Z_IDX = 3'd7;
    localparam N_IDX = 3'd6;
    localparam H_IDX = 3'd5;
    localparam C_IDX = 3'd4;
    
    localparam eightbitzero = 8'h00;
    // [Z,N,H,C,0,0,0,0]
    logic [7:0] FLAG_REG_IN, FLAG_REG_OUT;
    
    logic [2:0] BIT_SEL;
    // Inputs to the flag register file from the Flag Reg MUX    
    assign Z_IN = FLAG_REG_IN[Z_IDX];
    assign N_IN = FLAG_REG_IN[N_IDX];
    assign H_IN = FLAG_REG_IN[H_IDX];
    assign C_IN = FLAG_REG_IN[C_IDX];
    
    // Outputs from the flag register file to the ALU or Memory
    assign FLAG_REG_OUT = {Z_FLAG,N_FLAG,H_FLAG,C_FLAG, 4'b0000};
        
    // Stack Pointer signals
    logic [15:0] SP_DIN;
    logic [15:0] SP_DOUT;
    
    // Memory signals    
    logic [7:0] MEM_DIN, MEM_DOUT;
    logic [15:0] MEM_ADDR_IN;
    logic [15:0] HL_PTR;
    
    // H is the X output of  Reg File and L is the Y output of the Reg File
    assign HL_PTR = {RF_DX_OUT, RF_DY_OUT};  

    logic [7:0] BIT_SEL_8;
    assign BIT_SEL_8 = {5'b00000, BIT_SEL};
    
    // Control signals
    logic [7:0] OPCODE;
    logic PC_INC;                   // program counter
    logic [1:0] PC_MUX_SEL;
    logic [1:0] RF_WR_SEL;
    logic ALU_OPY_SEL;
    logic FLAGS_DATA_SEL;
    logic SP_LD, SP_INCR, SP_DECR;   // stack pointer
    logic SP_DIN_SEL;
    logic MEM_WE, MEM_RE;            // memory
    logic [1:0] MEM_ADDR_SEL;
    logic [2:0] MEM_DATA_SEL;
    logic [7:0] IMMED_ADDR_LOW, IMMED_ADDR_HIGH;
    logic [7:0] IMMED_DATA_LOW, IMMED_DATA_HIGH;


    logic [15:0] IMMED_ADDR, IMMED_ADDR_1;
    // Concatenate the High and Low Bytes of the Immediate Address Values
    assign IMMED_ADDR = {IMMED_ADDR_HIGH,IMMED_ADDR_LOW};
    assign IMMED_ADDR_1 = IMMED_ADDR + 1;
    
    assign JP_PC = {IMMED_DATA_HIGH, IMMED_DATA_LOW}-1;
    
    logic [15:0] IMMED_DATA_16;
    // Concatenate the High and Low Bytes of the Immediate Data Values
    assign IMMED_DATA_16 = {IMMED_DATA_HIGH,IMMED_DATA_LOW};
    
    logic [15:0] SP_IMMED_VAL;
    // Set equal to the Stack Pointer DOUT + an Immediate Value
    assign SP_IMMED_VAL = SP_DOUT + IMMED_DATA_LOW;

    // Restart Address Values    
    logic [3:0] RST_MUX_SEL;
    logic [15:0] RST_ADDR;
    
    MUX8to1#(.DATA_SIZE(16)) RST_MUX(
        .In0(16'h0000), .In1(16'h0008), .In2(16'h0010), .In3(16'h0018), .In4(16'h0020), .In5(16'h0028), .In6(16'h0030), .In7(16'h0038),
        .Sel(RST_MUX_SEL), .Out(RST_ADDR)
    );
    MUX4to1#(.DATA_SIZE(16)) ProgCount_MUX(
        .In0(JP_PC), .In1(CU_PC_ADDR), .In2(RF_16_OUT), .In3(RST_ADDR),
        .Sel(PC_MUX_SEL), .Out(PC_DIN)
    );
    
    ProgCount ProgCount( 
        .PC_CLK(CLK),
        .PC_RST(RST),
        .PC_LD(PC_LD),
        .PC_INC(PC_INC),
        .PC_DIN(PC_DIN),
        .PC_COUNT(PC)
    );
    MUX2to1#(.DATA_SIZE(8)) ALU_A_MUX(
        .In0(RF_DX_OUT), .In1(MEM_DOUT),
        .Sel(ALU_A_SEL), .Out(ALU_A)
    );
    
    MUX4to1#(.DATA_SIZE(8)) ALU_B_MUX(
        .In0(RF_DY_OUT), .In1(MEM_DOUT), .In2(OPCODE), .In3(BIT_SEL_8),
        .Sel(ALU_B_SEL), .Out(ALU_B)
    );
    
    MUX4to1#(.DATA_SIZE(16)) ALU_16_B_MUX(
        .In0(ALU_16_OUT), .In1(16'h0000), .In2(), .In3(),
        .Sel(ALU_16_B_SEL), .Out(ALU_16_B)
    );
    
    ALU ALU(
        .ALU_FUN(ALU_FUN), .A(ALU_A), .B(ALU_B), .FLAGS_IN(FLAG_REG_OUT[7:4]),
        .ALU_OUT(ALU_OUT), .FLAGS_OUT(ALU_FLAGS_OUT)
    );
    ALU_16 ALU_16(
        .ALU_FUN(ALU_16_FUN), .A(HL_PTR), .B(), .FLAGS_IN(FLAG_REG_OUT[7:4]),
        .ALU_OUT(ALU_16_OUT), .FLAGS_OUT(ALU_FLAGS_OUT)
    );
    
    MUX2to1 Flag_Reg_MUX(
        .In0({ALU_FLAGS_OUT,4'b0000}), .In1(MEM_DOUT), 
        .Sel(FLAGS_DATA_SEL), .Out(FLAG_REG_IN)
    );
    
    Flag_Reg Flag_Reg(
        .CLK(CLK), .RST(RST),
        .Z_IN(Z_IN), .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), .Z_OUT(Z_FLAG),
        .N_IN(N_IN), .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), .N_OUT(N_FLAG),
        .H_IN(H_IN), .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), .H_OUT(H_FLAG),
        .C_IN(C_IN), .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), .C_OUT(C_FLAG)
    );
    
    // Change Control Unit
    MUX9to1 RegFile_MUX(
        .In0(ALU_OUT), .In1(MEM_DOUT), 
        .In2(SP_DOUT[7:0]), .In3(ALU_16_OUT[15:8]), .In4(SP_IMMED_VAL[7:0]), .In5(SP_IMMED_VAL[15:8]),
        .In6(IMMED_DATA_LOW), .In7(IMMED_DATA_HIGH), .In8(RF_DY_OUT),
        .Sel(RF_DIN_SEL),  .Out(RF_DIN)

 //   MUX6to1 RegFile_MUX(
 //       .In0(ALU_OUT), .In1(MEM_DOUT), .In2(SP_DOUT), 
 //       .In3(), .In4(OPCODE), .In5(RF_DY_OUT),
 //       .Sel(RF_DIN_SEL),  .Out(RF_DIN)

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

    MUX2to1#(.DATA_SIZE(16)) SP_MUX(
        .In0(RF_16_OUT), .In1(IMMED_DATA_16),
        .Sel(SP_DIN_SEL), .Out(SP_DIN)
    );

    Stack_Pointer Stack_Pointer(
        .SP_LD(SP_LD),
        .SP_INCR(SP_INCR),
        .SP_DECR(SP_DECR),
        .RST(RST),
        .CLK(CLK),
        .DIN(SP_DIN),
        .DOUT(SP_DOUT)
    );
    
    // Memory Address MUX
    MUX4to1#(.DATA_SIZE(16)) MEM_ADDR_MUX(
        .In0(SP_DOUT), .In1(IMMED_ADDR), .In2(IMMED_ADDR_1), .In3(RF_16_OUT),
        .Sel(MEM_ADDR_SEL), .Out(MEM_ADDR_IN)
    );
    
    // Memory Data MUX
    MUX10to1 MEM_DATA_MUX(
        .In0(RF_DX_OUT), .In1(PC[7:0]), .In2(PC[15:8]), .In3(FLAG_REG_OUT), 
        .In4(SP_DOUT[7:0]), .In5(SP_DOUT[15:8]), .In6(INTR_REG_DIN), 
      .In7(ALU_OUT), .In8(IMMED_DATA_LOW), .In9(RF_DY_OUT), .Sel(MEM_DATA_SEL), .Out(MEM_DIN)
    );
    
    Memory Memory(
        .CLK(CLK), 
        .WE(MEM_WE), 
        .RE(MEM_RE), 
        .ADDR(MEM_ADDR_IN), 
        .DIN(MEM_DIN),
        .DOUT(MEM_DOUT)
    );
    
    ControlUnit ControlUnit(
        // Inputs
        .CLK(CLK), .INTR(), .RESET(CPU_RESET),
        .C(C_FLAG), .Z(Z_FLAG), .N(N_FLAG), .H(H_FLAG), 
        .OPCODE(OPCODE), // Memory Line
        // Outputs
        .PC(PC),
        .PC_LD(PC_LD), .PC_INC(PC_INC),     // program counter
        .PC_HIGH_FLAG(PC_HIGH_FLAG), .PC_LOW_FLAG(PC_LOW_FLAG),
        .PC_MUX_SEL(PC_MUX_SEL), .CALL_MUX_SEL(CALL_MUX_SEL),                   
        .PC_ADDR_OUT(CU_PC_ADDR),                     
        .RF_WR(RF_WR),             // register file
        .RF_WR_SEL(RF_DIN_SEL), 
        .RF_ADRX(RF_ADRX), .RF_ADRY(RF_ADRY),
        .ALU_SEL(ALU_FUN),     // ALU
        .ALU_16_SEL(ALU_16_FUN),
        .ALU_OPX_SEL(ALU_A_SEL), 
        .ALU_OPY_SEL(ALU_B_SEL),
        .ALU_16_B_SEL(ALU_16_B_SEL),
        .MEM_WE(MEM_WE), .MEM_RE(MEM_RE), // memory
        .MEM_ADDR_SEL(MEM_ADDR_SEL), .MEM_DATA_SEL(MEM_DATA_SEL),
        .IMMED_ADDR_LOW(IMMED_ADDR_LOW), .IMMED_ADDR_HIGH(IMMED_ADDR_HIGH),
        .IMMED_DATA_LOW(IMMED_DATA_LOW), .IMMED_DATA_HIGH(IMMED_DATA_HIGH),
        .SP_LD(SP_LD), .SP_INCR(SP_INCR), .SP_DECR(SP_DECR), .SP_DIN_SEL(SP_DIN_SEL), // stack pointer
        .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), // Flags 
        .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), // Z Flag control
        .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), // N Flag control
        .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), // H Flag control
        .FLAGS_DATA_SEL(FLAGS_DATA_SEL),        
        .I_CLR(), .I_SET(), .FLG_LD_SEL(), // interrupts
        .RST(RST),       // reset
        .IO_STRB(),    // IO
        .BIT_SEL(BIT_SEL),
        .RST_MUX_SEL(RST_MUX_SEL)
    );
   
endmodule
