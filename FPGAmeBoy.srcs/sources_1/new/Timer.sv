`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/15/2021 03:23:21 PM
// Design Name: 
// Module Name: Timer
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
module timer(
    input clk,
    
    input RST,
    input [15:0] REG_ADDR,
    input  [7:0] DIN,
    input wr,
    input rd,
    input cs,
    
    output logic [7:0] DOUT,
    output logic TIMER_IRQ
    );
    
    logic [7:0] REG_DIV; // Divider Register
    logic [7:0] REG_TIMA; // Timer counter
    logic [7:0] REG_TMA; // Timer modulo
    logic [7:0] REG_TAC; // Timer control
    
    logic [15:0] div; // use for counting
    
    logic TIMER_EN = REG_TAC[2]; //enable timer if bit 2 of TImer Control is set
    logic [1:0] CLOCK_SEL = REG_TAC[1:0]; // Last 2 LSB determine the input clock select
    
    assign REG_DIV[7:0] = div[15:8];  // assign 8 MSB of div[15:0] to divider register
    
    logic clk_4khz = div[9];
    logic clk_256khz = div[3];
    logic clk_64khz = div[5];
    logic clk_16khz = div[7];
    logic clk_tim;
    //assign CLK_tim depending on timer_en and clock_sel
    assign clk_tim = (TIMER_EN) ? (
        (CLOCK_SEL == 2'b00) ? (clk_4khz) : (
        (CLOCK_SEL == 2'b01) ? (clk_256khz) : (
        (CLOCK_SEL == 2'b10) ? (clk_64khz) : 
                                   (clk_16khz)))) : (1'b0);

    logic last_clk_tim;
        
    // Bus RW
    // Combinational Read
    always @(*)
    begin
        DOUT = 8'hFF;
        if (REG_ADDR == 16'hFF04) DOUT = REG_DIV; else
        if (REG_ADDR == 16'hFF05) DOUT = REG_TIMA; else
        if (REG_ADDR == 16'hFF06) DOUT = REG_TMA; else
        if (REG_ADDR == 16'hFF07) DOUT = REG_TAC;
    end
    
    // Sequential Write
    always @(posedge clk) begin
        last_clk_tim <= clk_tim;
    end
    
    always @(posedge clk) begin
        if (RST) begin
            REG_TIMA <= 0;
            REG_TMA <= 0;
            REG_TAC <= 0;
            div <= 0;
            TIMER_IRQ <= 0;
            
        end
        else begin
            div <= div + 1'b1;
            if (wr && cs) begin
                if (REG_ADDR == 16'hFF04) div <= 16'b0;
                else if (REG_ADDR == 16'hFF06) begin
                    REG_TMA <= DIN;
                    
                end
                else if (REG_ADDR == 16'hFF07) REG_TAC <= DIN;
                //else if (REG_ADR == 16'hFF05) REG_TIMA <= DIN;
            end    
            else begin
                    if ((last_clk_tim == 1'b1)&&(clk_tim == 1'b0)&&(TIMER_EN)) begin
                        REG_TIMA <= REG_TIMA + 1'b1;
                        if (REG_TIMA == 8'hFF) TIMER_IRQ <= 1'b1; 
                    end
                    else begin 
                        if (TIMER_IRQ) begin
                            TIMER_IRQ <= 1'b0;
                        end
                        if (TIMER_EN) begin
                            if (REG_TIMA == 8'b0) REG_TIMA <= REG_TMA;                        
                        end
                    
                    end
            end
        end
    end


endmodule
