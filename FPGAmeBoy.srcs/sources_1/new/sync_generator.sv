`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/04/2021 06:15:15 PM
// Design Name: 
// Module Name: sync_generator
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


module sync_generator(
    input CLK, RST,
    
    output logic VSYNC,HSYNC
    );
    //    // HV Timing
//    // 10MHz 800 x 600 @ 60Hz
    localparam PPU_H_FRONT  = 16'd112;
    localparam PPU_H_SYNC   = 16'd240; // So front porch + sync = OAM search
    localparam PPU_H_TOTAL  = 16'd2080;
    localparam PPU_H_PIXEL  = 16'd1600;
    // 8 null pixels in the front for objects which have x < 8, 8 bit counter
    localparam PPU_H_OUTPUT = 16'd1616;
   
    localparam PPU_V_ACTIVE = 16'd600;
    localparam PPU_V_BACK   = 16'd23;
    localparam PPU_V_SYNC   = 16'd6;  
    localparam PPU_V_BLANK  = 16'd29;
    // Visible area + Front Porch
    localparam PPU_V_TOTAL  = 16'd666;
   
    // Raw timing counter
    reg [15:0] h_count;
    reg [15:0] v_count;
    
    // HV counter
    always @(posedge CLK)
    begin
        if (RST) begin
            h_count <= 0;
            HSYNC <= 0;
            v_count <= 0;
            VSYNC <= 0;
        end
        else begin
            if(h_count < PPU_H_TOTAL - 1)
                h_count <= h_count + 1'b1;
            else begin
                h_count <= 0;
                if(v_count < PPU_V_TOTAL - 1)
                    v_count <= v_count + 1'b1;
                else
                    v_count <= 0;
                if(v_count == PPU_V_ACTIVE + PPU_V_BACK - 1)
                    VSYNC <= 1;
                if(v_count == PPU_V_ACTIVE + PPU_V_BACK + PPU_V_SYNC - 1)
                    VSYNC <= 0;
            end
            if(h_count == PPU_H_FRONT - 1)
                HSYNC <= 1;
            if(h_count == PPU_H_FRONT + PPU_H_SYNC - 1)
                HSYNC <= 0;
        end 
    end
endmodule
