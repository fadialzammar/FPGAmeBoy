`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: ME (Izak Walker)
// 
// Create Date: 01/09/2021 04:19:50 PM
// Design Name: 
// Module Name: Flag_Reg
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


module Flag_Reg(

    input CLK,
    input RST,
    
    // Zero FLag
    input Z_IN,
    input Z_FLAG_LD,
    input Z_FLAG_SET,
    input Z_FLAG_CLR,
    output logic Z_OUT = 1'b0,
    
    // Subtract Flag
    input N_IN,
    input N_FLAG_LD,
    input N_FLAG_SET,
    input N_FLAG_CLR,
    output logic N_OUT = 1'b0,
    
    // Half Carry Flag
    input H_IN,
    input H_FLAG_LD,
    input H_FLAG_SET,
    input H_FLAG_CLR,    
    output logic H_OUT = 1'b0,
    
    // Carry Flag
    input C_IN,
    input C_FLAG_LD,
    input C_FLAG_SET,
    input C_FLAG_CLR,
    output logic C_OUT = 1'b0
    
    );
    
    // Zero Flag Register
    always_ff @(posedge CLK) begin
        //RESET
        if (RST == 1) begin
            Z_OUT <= 0; 
        end
        // Clear Z Flag
        else if (Z_FLAG_CLR == 1) begin
            Z_OUT <= 0; 
        end
        // Set Z FLag High
        else if (Z_FLAG_SET == 1) begin
            Z_OUT <= 1;
        end
        // Load Z Flag
        else if (Z_FLAG_LD == 1) begin
            Z_OUT <= Z_IN;
        end
    end
    
    // Subtract Flag Register
    always_ff @(posedge CLK) begin
        //RESET
        if (N_FLAG_CLR == 1) begin
            N_OUT <= 0; 
        end
        // Clear N Flag
        else if (N_FLAG_CLR == 1) begin
            N_OUT <= 0; 
        end
        // Set N FLag High
        else if (N_FLAG_SET == 1) begin
            N_OUT <= 1;
        end
        // Load N Flag
        else if (N_FLAG_LD == 1) begin
            N_OUT <= N_IN;
        end
    end
    
    // Half Carry Flag Register
    always_ff @(posedge CLK) begin
        // RESET
        if (RST == 1) begin
            H_OUT <= 0; 
        end    
        // Clear H Flag
        else if (H_FLAG_CLR == 1) begin
            H_OUT <= 0; 
        end
        // Set H FLag High
        else if (H_FLAG_SET == 1) begin
            H_OUT <= 1;
        end
        // Load H Flag
        else if (H_FLAG_LD == 1) begin
            H_OUT <= H_IN;
        end

    end
    
    // Carry Flag Register
    always_ff @(posedge CLK) begin
        // RESET
        if (RST == 1) begin
            C_OUT <= 0; 
        end
        // Clear C Flag
        else if (C_FLAG_CLR == 1) begin
            C_OUT <= 0; 
        end
        // Set C FLag High
        else if (C_FLAG_SET == 1) begin
            C_OUT <= 1;
        end
        // Load C Flag
        else if (C_FLAG_LD == 1) begin
            C_OUT <= C_IN;
        end
    end
endmodule
