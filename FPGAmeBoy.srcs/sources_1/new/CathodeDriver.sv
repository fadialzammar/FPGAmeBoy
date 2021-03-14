`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Paul Hummel
// 
// Create Date: 06/28/2018 11:50:35 PM
// Design Name: 
// Module Name: CathodeDriver
// Target Devices: Basys3 and RAT MCU
// Description: 
//              CATHODES = {dp,a,b,c,d,e,f,g}
//              ANODES = {d4, d3, d2, d1}
// 
// Revision:
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////


module CathodeDriver(
    input CLK,
    input [15:0] HEX,
    output logic [7:0] CATHODES,
    output logic [3:0] ANODES
    );
    
    logic s_clk_500 = 1'b0;             // 250Hz refresh clock
    logic [1:0] r_disp_digit = 2'b00;   // current digit being displayed
    logic [19:0] clk_div_counter = 20'h00000;

    // Clock Divider to create 500 Hz refresh from 100 MHz clock
	always_ff @(posedge CLK) begin
        clk_div_counter = clk_div_counter + 1;        
        if ( clk_div_counter == 20'h186A0) begin     // x186A0 = 1*10^5 = 1 ms toggle (x30D40)
            clk_div_counter = 20'h00000;
            s_clk_500 = ~s_clk_500;         // toggle every 1 ms creates 500 Hz clock
        end
    end    
    
    // Refresh Seven Segment Display every 240 Hz 
    always_ff @(posedge s_clk_500) begin    
        case (r_disp_digit)
            2'b00: begin
                ANODES= 4'b1110;
                case (HEX[3:0])
                    4'b0000: CATHODES = 8'b10000001; //0
                    4'b0001: CATHODES = 8'b11001111; //1
                    4'b0010: CATHODES = 8'b10010010; //2
                    4'b0011: CATHODES = 8'b10000110; //3
                    4'b0100: CATHODES = 8'b11001100; //4
                    4'b0101: CATHODES = 8'b10100100; //5
                    4'b0110: CATHODES = 8'b10100000; //6
                    4'b0111: CATHODES = 8'b10001111; //7
                    4'b1000: CATHODES = 8'b10000000; //8
                    4'b1001: CATHODES = 8'b10001100; //9
                    4'b1010: CATHODES = 8'b10001000; //a
                    4'b1011: CATHODES = 8'b11100000; //b
                    4'b1100: CATHODES = 8'b10110001; //c
                    4'b1101: CATHODES = 8'b11000010; //d
                    4'b1110: CATHODES = 8'b10110000; //e
                    4'b1111: CATHODES = 8'b10111000; //f
                    default: CATHODES = 8'b11111111; // failsafe turn off
                endcase
            end
            2'b01: begin
                ANODES= 4'b1101;
                case (HEX[7:4])
                    4'b0000: CATHODES = 8'b10000001;
                    4'b0001: CATHODES = 8'b11001111;
                    4'b0010: CATHODES = 8'b10010010;
                    4'b0011: CATHODES = 8'b10000110;
                    4'b0100: CATHODES = 8'b11001100;
                    4'b0101: CATHODES = 8'b10100100;
                    4'b0110: CATHODES = 8'b10100000;
                    4'b0111: CATHODES = 8'b10001111;
                    4'b1000: CATHODES = 8'b10000000;
                    4'b1001: CATHODES = 8'b10001100;
                    4'b1010: CATHODES = 8'b10001000; //a
                    4'b1011: CATHODES = 8'b11100000;
                    4'b1100: CATHODES = 8'b10110001;
                    4'b1101: CATHODES = 8'b11000010;
                    4'b1110: CATHODES = 8'b10110000;
                    4'b1111: CATHODES = 8'b10111000;
                    default: CATHODES = 8'b11111111; // all off on error
                endcase
            end
            2'b10: begin
                ANODES= 4'b1011;
                case (HEX[11:8])
                    4'b0000: CATHODES = 8'b10000001;
                    4'b0001: CATHODES = 8'b11001111;
                    4'b0010: CATHODES = 8'b10010010;
                    4'b0011: CATHODES = 8'b10000110;
                    4'b0100: CATHODES = 8'b11001100;
                    4'b0101: CATHODES = 8'b10100100;
                    4'b0110: CATHODES = 8'b10100000;
                    4'b0111: CATHODES = 8'b10001111;
                    4'b1000: CATHODES = 8'b10000000;
                    4'b1001: CATHODES = 8'b10001100;
                    4'b1010: CATHODES = 8'b10001000; //a
                    4'b1011: CATHODES = 8'b11100000;
                    4'b1100: CATHODES = 8'b10110001;
                    4'b1101: CATHODES = 8'b11000010;
                    4'b1110: CATHODES = 8'b10110000;
                    4'b1111: CATHODES = 8'b10111000;
                    default: CATHODES = 8'b11111111; // all off on error
                endcase
            end
            2'b11: begin
                ANODES= 4'b0111;
                case (HEX[15:12])
                    4'b0000: CATHODES = 8'b10000001;
                    4'b0001: CATHODES = 8'b11001111;
                    4'b0010: CATHODES = 8'b10010010;
                    4'b0011: CATHODES = 8'b10000110;
                    4'b0100: CATHODES = 8'b11001100;
                    4'b0101: CATHODES = 8'b10100100;
                    4'b0110: CATHODES = 8'b10100000;
                    4'b0111: CATHODES = 8'b10001111;
                    4'b1000: CATHODES = 8'b10000000;
                    4'b1001: CATHODES = 8'b10001100;
                    4'b1010: CATHODES = 8'b10001000; //a
                    4'b1011: CATHODES = 8'b11100000;
                    4'b1100: CATHODES = 8'b10110001;
                    4'b1101: CATHODES = 8'b11000010;
                    4'b1110: CATHODES = 8'b10110000;
                    4'b1111: CATHODES = 8'b10111000;
                    default: CATHODES = 8'b11111111; // all off on error
                endcase
            end
            default: begin      // digit error turn everything off
                ANODES = 4'hF;
                CATHODES = 8'hFF;
                r_disp_digit = 2'b00;
            end
        endcase
        
        r_disp_digit = r_disp_digit + 1; 
    end

    
    
endmodule
