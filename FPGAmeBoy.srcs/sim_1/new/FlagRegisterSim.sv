`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/09/2021 04:43:54 PM
// Design Name: 
// Module Name: FLAG_REG_SIM
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


module FLAG_REG_SIM();

    logic CLK = 0;
    logic RST;
    
    // Zero FLag
    logic Z;
    logic Z_FLAG_LD;
    logic Z_FLAG_SET;
    logic Z_FLAG_CLR;
    logic Z_FLAG;
    
    // Subtract Flag
    logic N;
    logic N_FLAG_LD;
    logic N_FLAG_SET;
    logic N_FLAG_CLR;
    logic N_FLAG;
    
    // Half Carry Flag
    logic H;
    logic H_FLAG_LD;
    logic H_FLAG_SET;
    logic H_FLAG_CLR;    
    logic H_FLAG;
    
    // Carry Flag
    logic C;
    logic C_FLAG_LD;
    logic C_FLAG_SET;
    logic C_FLAG_CLR;
    logic C_FLAG;
    
    // Clock
    always #5 CLK = ~CLK;
    
    // Module Instantiation
    Flag_Reg Flag_Reg_SIM(
        .CLK(CLK), .RST(RST),
        .Z(Z), .Z_FLAG_LD(Z_FLAG_LD), .Z_FLAG_SET(Z_FLAG_SET), .Z_FLAG_CLR(Z_FLAG_CLR), .Z_FLAG(Z_FLAG),
        .N(N), .N_FLAG_LD(N_FLAG_LD), .N_FLAG_SET(N_FLAG_SET), .N_FLAG_CLR(N_FLAG_CLR), .N_FLAG(N_FLAG),
        .H(H), .H_FLAG_LD(H_FLAG_LD), .H_FLAG_SET(H_FLAG_SET), .H_FLAG_CLR(H_FLAG_CLR), .H_FLAG(H_FLAG),
        .C(C), .C_FLAG_LD(C_FLAG_LD), .C_FLAG_SET(C_FLAG_SET), .C_FLAG_CLR(C_FLAG_CLR), .C_FLAG(C_FLAG)
    );
    
    initial begin
    
    // Initial Values
    RST = 0;
    Z = 0;
    N = 0;
    H = 0;
    C = 0;  
    
    Z_FLAG = 0;
    N_FLAG = 0;
    H_FLAG = 0;
    C_FLAG = 0;
    
    Z_FLAG_LD = 0;
    N_FLAG_LD = 0;
    H_FLAG_LD = 0;
    C_FLAG_LD = 0;
    
    Z_FLAG_SET = 0;
    N_FLAG_SET = 0;
    H_FLAG_SET = 0;
    C_FLAG_SET = 0;
    
    Z_FLAG_CLR = 0;
    N_FLAG_CLR = 0;
    H_FLAG_CLR = 0;
    C_FLAG_CLR = 0;
    
    #5;
    
    // Load Test
    RST = 0;
    Z = 1;
    N = 1;
    H = 1;
    C = 1;  
    
    Z_FLAG_LD = 1;
    N_FLAG_LD = 1;
    H_FLAG_LD = 1;
    C_FLAG_LD = 1;
    
    #10;
    
    Z_FLAG_LD = 0;
    N_FLAG_LD = 0;
    H_FLAG_LD = 0;
    C_FLAG_LD = 0;
    
    // CLear Test
    RST = 0;
    Z = 1;
    N = 1;
    H = 1;
    C = 1;  
    
    Z_FLAG_CLR = 1;
    N_FLAG_CLR = 1;
    H_FLAG_CLR = 1;
    C_FLAG_CLR = 1;
    
    #10;
    
    Z_FLAG_CLR = 0;
    N_FLAG_CLR = 0;
    H_FLAG_CLR = 0;
    C_FLAG_CLR = 0;
    
    // SET Test
    RST = 0;
    Z = 0;
    N = 0;
    H = 0;
    C = 0;  
    
    Z_FLAG_SET = 1;
    N_FLAG_SET = 1;
    H_FLAG_SET = 1;
    C_FLAG_SET = 1;
    
    #10;
    
    Z_FLAG_SET = 0;
    N_FLAG_SET = 0;
    H_FLAG_SET = 0;
    C_FLAG_SET = 0;
    
    // RESET Test
    RST = 0;
    Z = 1;
    N = 1;
    H = 1;
    C = 1;  
    
    Z_FLAG_LD = 1;
    N_FLAG_SET = 1;
    H_FLAG_CLR = 1;
    C_FLAG_LD = 1;
    
    RST = 1;
    
    #10;
    
    RST = 0;
    Z = 0;
    N = 0;
    H = 0;
    C = 0;  
    // Set High
    Z_FLAG_SET = 1;
    N_FLAG_SET = 1;
    H_FLAG_SET = 1;
    C_FLAG_SET = 1;
    
    end
    
endmodule
