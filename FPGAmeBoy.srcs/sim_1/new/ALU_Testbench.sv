`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Izak Walker
// 
// Create Date: 01/30/2021 06:56:58 PM
// Design Name: 
// Module Name: ALU_Testbench
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


module ALU_Testbench();

    // Inputs
    logic [4:0] ALU_FUN;
    logic [7:0] A,B;
    logic[3:0] FLAGS_IN;
    
    // Outputs
    logic [7:0] ALU_OUT; 
    logic [3:0] FLAGS_OUT;
    
    // ALU Module Instantiation
    ALU ALU_TB(
        .ALU_FUN(ALU_FUN), .A(A), .B(B), .FLAGS_IN(FLAGS_IN),
        .ALU_OUT(ALU_OUT), .FLAGS_OUT(FLAGS_OUT)
    );
    
    // Local Arithmetic Op-code parameters
    localparam ADD  = 5'b00000;
    localparam ADC  = 5'b00001;
    localparam SUB  = 5'b00010;
    localparam SBC  = 5'b00011;
    localparam AND  = 5'b00100;
    localparam OR   = 5'b00101;
    localparam XOR  = 5'b00110;
    localparam CP   = 5'b00111;
    localparam INC  = 5'b01000;
    localparam DEC  = 5'b01001;  
    localparam SWAP = 5'b01010;
    localparam DAA  = 5'b01011;
    localparam CPL  = 5'b01100;
    localparam CCF  = 5'b01101;
    localparam SCF  = 5'b01110;
    localparam RLC  = 5'b01111;
    localparam RL   = 5'b10000;
    localparam RRC  = 5'b10001;
    localparam RR   = 5'b10010;
    localparam SLA  = 5'b10011;
    localparam SRA  = 5'b10100;
    localparam SRL  = 5'b10101;
    localparam BIT  = 5'b10110;
    localparam SET  = 5'b10111;
    localparam RES  = 5'b11000;
    
    initial begin
    
        // Initialize values to 0
        ALU_FUN = ADD;
        A = 8'h0;
        B = 8'h0;
        FLAGS_IN = 4'h0;
        FLAGS_OUT = 4'h0;
        ALU_OUT = 8'h0;
        
        #10;
        // ADD  Test
        A = 8'h45;
        B = 8'h69;
        FLAGS_IN = 4'b0;
        ALU_FUN = ADD;
        
        #10;
        // ADC  Test
        
        
        #10;
        // SUB  Test
        
        
        #10;
        // SBC  Test
        
        
        #10;
        // AND  Test
        
        
        #10;
        // OR   Test
        
        
        #10;
        // XOR  Test
        
        
        #10;
        // CP   Test
        
    
        #10;
        // INC  Test
            
        
        #10;
        // DEC  Test
        
        
        #10;
        // SWAP Test
        
        
        #10;
        // DAA  Test
        
            
        #10;    
        // PL   Test
        
    
        #10;    
        // CFF  Test
        
       
        #10; 
        // SCF  Test
        
        
        #10;
        // RLC  Test
        
        
        #10;
        // RL   Test
        
        
        #10;
        // RRC  Test
        
        
        #10;
        // RR   Test
        
        
        #10;
        // SLA  Test
        
        
        #10;
        // SRA  Test
        
        
        #10;
        // SRL  Test
        
        
        #10;
        // BIT  Test
        A = 8'b11110111;
        B = 8'h3;
        ALU_FUN = BIT;
        
        #10;
        // SET  Test
        A = 8'b00001111;
        B = 8'h6;
        ALU_FUN = SET;
        
        #10;
        // RES  Test
        A = 8'b11111111;
        B = 8'h7;
        ALU_FUN = RES;
        
        #10;
     
    end
 
endmodule
