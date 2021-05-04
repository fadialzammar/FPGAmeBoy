`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/17/2021 09:06:22 PM
// Design Name: 
// Module Name: CPUWrapper
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


module CPU_Wrapper(
    input CLK,
    input RST,
    input [7:0] MEM_DOUT,     // from Memory to CPU
    input [7:0] OPCODE,
    input [7:0] INT_EN, INT_FLAG,
    
    output [7:0] MEM_DIN,     // from CPU to Memory
    output MEM_WE,
    output MEM_RE,
    output [15:0] MEM_ADDR_IN,  // from CPU to Memory
    output [15:0] PC    // TODO: change to 10 bits?
    );
    
    // Program Counter Signals
    logic PC_LD;
    logic [15:0] PC_DIN;
    // logic [15:0] PC;
    logic [15:0] RET_PC, CALL_PC, CALL_PC_FALSE, JP_PC, JR_PC, JR_PC_FALSE;
    logic [15:0] CALL_PC_IN;
    logic [15:0] CU_PC_ADDR;
    logic [15:0] REG_PC_ADDR;
    
    // ALU Signals
    logic [4:0] ALU_FUN;
    logic [1:0] ALU_16_FUN;
    logic [7:0] ALU_A, ALU_B;
    logic [3:0] ALU_FLAGS_IN;
    logic [7:0] ALU_OUT; 
    logic [3:0] ALU_FLAGS_OUT;
    logic [3:0] ALU_16_FLAGS_OUT;
    logic ALU_A_SEL;
    logic [2:0] ALU_B_SEL;
    
    // ALU 16 bit signals
    logic [15:0] ALU_16_A,ALU_16_B,ALU_16_OUT;
    logic [1:0] ALU_16_A_SEL;
    
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
    // [Z,N,H,C,0,0,0,0]
    localparam Z_IDX = 3'd7;
    localparam N_IDX = 3'd6;
    localparam H_IDX = 3'd5;
    localparam C_IDX = 3'd4;
    
    localparam eightbitzero = 8'h00;
    logic [2:0] BIT_SEL;
    logic [7:0] FLAG_REG_IN, FLAG_REG_OUT;
    
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
    
    // Interrupt Signals
    logic [2:0] INTR_ID;
    logic INTR;
    
    // Memory signals    
    // logic [7:0] MEM_DIN, MEM_DOUT;
    // logic [15:0] MEM_ADDR_IN;
    logic [15:0] HL_PTR;
    logic [15:0] MEM_ADDR_BUF_OUT;
    logic MEM_ADDR_BUF_WE;
    
    // H is the X output of  Reg File and L is the Y output of the Reg File
    assign HL_PTR = {RF_DX_OUT, RF_DY_OUT};  

    logic [7:0] BIT_SEL_8;
    assign BIT_SEL_8 = {5'b00000, BIT_SEL};
    
    // Control signals
    // logic [7:0] OPCODE;
    logic PC_INC;                   // program counter
    logic PC_HIGH_FLAG, PC_LOW_FLAG;
    logic [2:0] PC_MUX_SEL;
    logic CALL_MUX_SEL;
    logic [1:0] RF_WR_SEL;
    logic ALU_OPY_SEL;
    logic [1:0] FLAGS_DATA_SEL;
    logic SP_LD, SP_INCR, SP_DECR;   // stack pointer
    logic [1:0] SP_DIN_SEL;
    // logic MEM_WE;                    // memory
    logic MEM_HOLD;
    logic [3:0] MEM_ADDR_SEL;
    logic [3:0] MEM_DATA_SEL;
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
   
    // RET PC High and Low Bytes
    logic [7:0] RET_PC_LOW, RET_PC_HIGH;   
    // Set the Return PC low byte to the low byte popped off the stack
    assign RET_PC_LOW = PC_LOW_FLAG && !PC_HIGH_FLAG ? MEM_DOUT : RET_PC_LOW;
    // Set the Return PC high byte to the high byte popped off the stack
    assign RET_PC_HIGH = PC_HIGH_FLAG && !PC_LOW_FLAG ? MEM_DOUT : RET_PC_HIGH;
    // Concatenate the High and Low Bytes of the PC Address Values
    assign RET_PC = {RET_PC_HIGH,RET_PC_LOW} + 2; // + 2 to bypass the immediate values after CALL
    assign CALL_PC = {IMMED_ADDR_HIGH,IMMED_ADDR_LOW} - 1;
    assign CALL_PC_FALSE = PC + 2; // CALL not taken due to conditional (skip immediat values)
    
    assign MEM_RE = ~MEM_HOLD;

    // CALL Data MUX
     MUX2to1 #(.DATA_SIZE(16)) CALL_MUX(
        .In0(CALL_PC_FALSE), .In1(CALL_PC),
        .Sel(CALL_MUX_SEL), .Out(CALL_PC_IN)
    );    

    // Restart Address Values    
    logic [3:0] RST_MUX_SEL;
    logic [15:0] RST_ADDR;
    
    MUX8to1#(.DATA_SIZE(16)) RST_MUX(
        .In0(16'h0000), .In1(16'h0008), .In2(16'h0010), .In3(16'h0018), .In4(16'h0020), .In5(16'h0028), .In6(16'h0030), .In7(16'h0038),
        .Sel(RST_MUX_SEL), .Out(RST_ADDR)
    );
    
   // Interrupt Address Values    
    logic [3:0] INTR_MUX_SEL;
    logic [15:0] INTR_ADDR;
    
    MUX5to1#(.DATA_SIZE(16)) INTR_MUX(
        .In0(16'h0040), .In1(16'h0048), .In2(16'h0050), .In3(16'h0058), .In4(16'h0060),
        .Sel(INTR_ID), .Out(INTR_ADDR)
    );
  
    // PC Data MUX
    MUX7to1#(.DATA_SIZE(16)) PC_MUX(
        .In0(JP_PC), .In1(CU_PC_ADDR), .In2(RF_16_OUT), .In3(RST_ADDR),
        .In4(RET_PC), .In5(CALL_PC_IN), .In6(INTR_ADDR), .Sel(PC_MUX_SEL), .Out(PC_DIN)
    );
    
   // Program Counter Instantiation
    ProgCount ProgCount( 
        .PC_CLK(CLK),
        .PC_RST(RST),
        .PC_LD(PC_LD),
        .PC_INC(PC_INC),
        .PC_DIN(PC_DIN),
        .PC_COUNT(PC)
    );
    
    // 8-bit ALU A input MUX
    MUX2to1#(.DATA_SIZE(8)) ALU_A_MUX(
        .In0(RF_DX_OUT), .In1(MEM_DOUT), 
        .Sel(ALU_A_SEL), .Out(ALU_A)
    );
    
    // 8-bit ALU B input MUX
    MUX6to1#(.DATA_SIZE(8)) ALU_B_MUX(
        .In0(RF_DY_OUT), .In1(MEM_DOUT), .In2(OPCODE), .In3(BIT_SEL_8), .In4(SP_DOUT[15:8]),.In5(SP_DOUT[7:0]),
        .Sel(ALU_B_SEL), .Out(ALU_B)
    );
    
    // 16-bit ALU A input MUX
    MUX4to1#(.DATA_SIZE(16)) ALU_16_A_MUX(
        .In0(HL_PTR), .In1(SP_DOUT), .In2(), .In3(),
        .Sel(ALU_16_A_SEL), .Out(ALU_16_A)
    );
    
    // 8-bit ALU Instantiation
    ALU ALU(
        .ALU_FUN(ALU_FUN), .A(ALU_A), .B(ALU_B), .FLAGS_IN(FLAG_REG_OUT[7:4]),
        .ALU_OUT(ALU_OUT), .FLAGS_OUT(ALU_FLAGS_OUT)
    );
    
    // 16-bit ALU Instantiation
    ALU_16 ALU_16(
        .ALU_FUN(ALU_16_FUN), .A(ALU_16_A), .B({8'b0,IMMED_DATA_LOW}), .FLAGS_IN(FLAG_REG_OUT[7:4]),
        .ALU_OUT(ALU_16_OUT), .FLAGS_OUT(ALU_16_FLAGS_OUT)
    );
    
    // Flag Register MUX
    MUX3to1 Flag_Reg_MUX(
        .In0({ALU_FLAGS_OUT,4'b0000}), .In1(MEM_DOUT), .In2({ALU_16_FLAGS_OUT,4'b0000}),
        .Sel(FLAGS_DATA_SEL), .Out(FLAG_REG_IN)
    );
    
    // Flag Register Instantiation
    Flag_Reg Flag_Reg(
        .CLK(CLK), .RST(RST),
        .Z_IN(Z_IN), .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), .Z_OUT(Z_FLAG),
        .N_IN(N_IN), .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), .N_OUT(N_FLAG),
        .H_IN(H_IN), .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), .H_OUT(H_FLAG),
        .C_IN(C_IN), .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), .C_OUT(C_FLAG)
    );
    
    // Reg File MUX
    MUX10to1 RegFile_MUX(
        .In0(ALU_OUT), .In1(MEM_DOUT), 
        .In2(SP_DOUT[7:0]), .In3(SP_DOUT[15:8]), .In4(ALU_16_OUT[15:8]), .In5(SP_IMMED_VAL[7:0]), .In6(SP_IMMED_VAL[15:8]),
        .In7(IMMED_DATA_LOW), .In8(IMMED_DATA_HIGH), .In9(RF_DY_OUT),
        .Sel(RF_DIN_SEL), .Out(RF_DIN)
    );
    
    // Reg File Instantiation
    RegFile RegFile(
        .ADRX(RF_ADRX), .ADRY(RF_ADRY), .DIN(RF_DIN),
        .DX_OUT(RF_DX_OUT), .DY_OUT(RF_DY_OUT), 
        .WE(RF_WR), .CLK(CLK)
    );

    // Stack Pointer Data MUX
    MUX3to1#(.DATA_SIZE(16)) SP_MUX(
        .In0(RF_16_OUT), .In1(IMMED_DATA_16),.In2(ALU_16_OUT),
        .Sel(SP_DIN_SEL), .Out(SP_DIN)
    );

    // Stack Pointer Instantiation
    Stack_Pointer Stack_Pointer(
        .SP_LD(SP_LD),
        .SP_INCR(SP_INCR),
        .SP_DECR(SP_DECR),
        .RST(RST),
        .CLK(CLK),
        .DIN(SP_DIN),
        .DOUT(SP_DOUT)
    );

    // Memory Address Buffer
    always_ff @ (posedge CLK) begin
        if (MEM_ADDR_BUF_WE)
            MEM_ADDR_BUF_OUT = RF_16_OUT;
    end
    
    // This is not good
    logic[15:0] HL_ADDR;
    logic HL_HOLD;
    // HL_State Buffer
    always_ff@(posedge CLK)
    begin
        if(HL_HOLD==1)
            HL_ADDR <= RF_16_OUT;
    end
    
    
    // Memory Address MUX
    MUX9to1#(.DATA_SIZE(16)) MEM_ADDR_MUX(
        .In0(SP_DOUT), .In1(IMMED_ADDR), .In2(IMMED_ADDR_1), .In3(RF_16_OUT), .In4(MEM_ADDR_BUF_OUT),
        .In5({8'hFF, IMMED_ADDR_LOW}), .In6({8'hFF, RF_DY_OUT}), .In7(), .In8(HL_ADDR), .Sel(MEM_ADDR_SEL), .Out(MEM_ADDR_IN)
    );
    
    // Interrupt Enable/Disable Values MUX
    interrupt_handler intr(
        .IME(IME),
        .D_IE(INT_EN),
        .D_IF(INT_FLAG),
        .INTR(INTR),
        .INTR_ID(INTR_ID)
    );
    // Memory Data MUX
    MUX10to1 MEM_DATA_MUX(
        .In0(RF_DX_OUT), .In1(PC[7:0]), .In2(PC[15:8]), .In3(FLAG_REG_OUT), 
        .In4(SP_DOUT[7:0]), .In5(SP_DOUT[15:8]), .In6(INTR_REG_DIN), 
        .In7(ALU_OUT), .In8(IMMED_DATA_LOW), .In9(RF_DY_OUT), .Sel(MEM_DATA_SEL), .Out(MEM_DIN)
    );
    
    // Control Unit Instantiation
    ControlUnit ControlUnit(
        // Inputs
        .CLK(CLK), .INTR(INTR), .RESET(RST),
        .C(C_FLAG), .Z(Z_FLAG), .N(N_FLAG), .H(H_FLAG), 
        .OPCODE(OPCODE), .PC(PC), // Memory Line
        // Outputs
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
        .ALU_16_A_SEL(ALU_16_A_SEL),
        .MEM_WE(MEM_WE), .MEM_HOLD(MEM_HOLD), // memory
        // .MEM_DIN_BUF_WE(MEM_DIN_BUF_WE),
        .MEM_ADDR_BUF_WE(MEM_ADDR_BUF_WE),
        .MEM_ADDR_SEL(MEM_ADDR_SEL), .MEM_DATA_SEL(MEM_DATA_SEL), .INTR_REG_SEL(INTR_REG_SEL),
        .IMMED_ADDR_LOW(IMMED_ADDR_LOW), .IMMED_ADDR_HIGH(IMMED_ADDR_HIGH), // immediates
        .IMMED_DATA_LOW(IMMED_DATA_LOW), .IMMED_DATA_HIGH(IMMED_DATA_HIGH),
        .SP_LD(SP_LD), .SP_INCR(SP_INCR), .SP_DECR(SP_DECR), .SP_DIN_SEL(SP_DIN_SEL), // stack pointer
        .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), // Flags 
        .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), // Z Flag control
        .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), // N Flag control
        .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), // H Flag control
        .FLAGS_DATA_SEL(FLAGS_DATA_SEL),        
        .I_CLR(), .I_SET(), .FLG_LD_SEL(), // interrupts
        .RST(RST),       // FIXME: duplicate resets
        .IO_STRB(),    // IO
        .BIT_SEL(BIT_SEL),
        .RST_MUX_SEL(RST_MUX_SEL),
        .HL_HOLD(HL_HOLD),
        .IME(IME),
        .INTR_ID(INTR_ID)
    );
  
endmodule
