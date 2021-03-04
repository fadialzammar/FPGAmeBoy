`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/03/2021 01:23:38 PM
// Design Name: 
// Module Name: Mux6to1
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


module Mux6to1(In0, In1, In2, In3, In4, In5, Sel, Out);
    input [7:0] In0, In1, In2, In3, In4, In5; //8-bit inputs 
    input [3:0] Sel; //selector signal
    output logic [8:0] Out; //8-bit output
    always_comb
        case (Sel) // 6->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            default: Out <= In0; 
        endcase
endmodule

module Mux5to1(In0, In1, In2, In3, In4, Sel, Out);
    input [7:0] In0, In1, In2, In3, In4; //8-bit inputs
    input [2:0] Sel; //selector signal
    output logic [7:0] Out; //8-bit output
    always_comb
        case (Sel) // 5->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            default: Out <= In0; 
        endcase
endmodule

module Mux4to1
    #(parameter DATA_SIZE = 8)
    (   input [DATA_SIZE - 1:0] In0, In1, In2, In3, //8-bit inputs 
        input [1:0] Sel, //selector signal
        output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 4->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            default: Out <= In0; 
        endcase
endmodule

module Mux3to1(In0, In1, In2, Sel, Out);
    input [7:0] In0, In1, In2; //8-bit inputs 
    input [1:0] Sel; //selector signal
    output logic [7:0] Out; //8-bit output
    always_comb
        case (Sel) // 3->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            default: Out <= In0; 
        endcase
endmodule

module Mux2to1(In0, In1, Sel, Out);
    input [7:0] In0, In1; //8-bit inputs 
    input Sel; //selector signal
    output logic [7:0] Out; //8-bit output
    always_comb
        case (Sel) // 2->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1;           
            default: Out <= In0; 
        endcase
endmodule
