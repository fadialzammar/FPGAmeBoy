`timescale 1ns / 1ps
`default_nettype wire
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/03/2021 02:48:46 PM
// Design Name: 
// Module Name: interrupt_handler
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


module interrupt_handler(
    input [7:0] D_IE, D_IF,
    input IME, INT_CLR,
    output logic ppu_vblank_ack, ppu_lcdc_ack, timer_ack,
    output logic INTR = 0,
    output reg [2:0] INTR_ID
    );
    logic [7:0] interrupts;
    
    always@(*)
    begin
    interrupts = D_IE & D_IF;
    
    // Generate Interrupt signals
    if(IME == 1)
    begin
        if(interrupts[0] == 1)
        begin
            INTR = 1;
            INTR_ID = 0;
        end
        else if(interrupts[1] == 1)      // V-Blank
        begin
            INTR = 1;
            INTR_ID = 1;
        end
        else if(interrupts[2] == 1)      // Timer
        begin
            INTR = 1;
            INTR_ID = 2;
        end
        else if(interrupts[3] == 1)       // Serial IO Transfer
        begin
            INTR = 1;
            INTR_ID = 3;
        end
       else if(interrupts[4] == 1)    // Controller Input
        begin
            INTR = 1;
            INTR_ID = 4;
        end
        else
        begin
            INTR = 0;
            INTR_ID = 7;
        end
    end  
    else
        INTR = 0;
    
    // Clear interrupt flag after ISR is executed
    if(INT_CLR == 1)
    begin
        INTR = 0;
        // Generate PPU/TIM ack signals
        case(INTR_ID)
            // PPU VBLANK
            0:
            begin
                ppu_vblank_ack = 1;
            end
            // PPU LCDC
            1:
            begin
                ppu_lcdc_ack = 1;
            end
            // Timer
            2:
            begin
                timer_ack = 1;
            end
            default:
            begin
                ppu_vblank_ack = 0;
                ppu_lcdc_ack = 0;
                timer_ack = 0;
            end
        endcase
    end
    else
    begin
            ppu_vblank_ack = 0;
            ppu_lcdc_ack = 0;
            timer_ack = 0;
    end
    end 
    
endmodule
