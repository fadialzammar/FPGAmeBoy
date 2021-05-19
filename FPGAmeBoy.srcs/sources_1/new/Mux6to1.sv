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
module MUX12to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, In7, In8, In9, In10, In11, //8-bit inputs 
    input [3:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out 
    );
    always_comb
        case (Sel) // 11->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            7: Out <= In7;
            8: Out <= In8;
            9: Out <= In9;
            10: Out <= In10;
            11: Out <= In11;
            default: Out <= In0; 
        endcase
endmodule
module MUX11to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, In7, In8, In9, In10, //8-bit inputs 
    input [3:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 10->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            7: Out <= In7;
            8: Out <= In8;
            9: Out <= In9;
            10: Out <= In10;
            default: Out <= In0; 
        endcase
endmodule

module MUX10to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, In7, In8, In9, //8-bit inputs 
    input [3:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 10->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            7: Out <= In7;
            8: Out <= In8;
            9: Out <= In9;
            default: Out <= In0; 
        endcase
endmodule

module MUX9to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, In7, In8, //8-bit inputs 
    input [3:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 9->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            7: Out <= In7;
            8: Out <= In8;
            default: Out <= In0; 
        endcase
endmodule

module MUX8to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, In7, //8-bit inputs 
    input [2:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 8->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            7: Out <= In7;
            default: Out <= In0; 
        endcase
endmodule

module MUX7to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, In6, //8-bit inputs 
    input [2:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 7->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            3: Out <= In3;
            4: Out <= In4;
            5: Out <= In5;
            6: Out <= In6;
            default: Out <= In0; 
        endcase
endmodule

module MUX6to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, In5, //8-bit inputs 
    input [2:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
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

module MUX5to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, In3, In4, //8-bit inputs
    input [2:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
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

module MUX4to1
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

module MUX3to1
#(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, In2, //8-bit inputs 
    input [1:0] Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 3->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1; 
            2: Out <= In2;
            default: Out <= In0; 
        endcase
endmodule

module MUX2to1
    #(parameter DATA_SIZE = 8)(
    input [DATA_SIZE - 1:0] In0, In1, //8-bit inputs 
    input Sel, //selector signal
    output logic [DATA_SIZE - 1:0] Out //8-bit output
    );
    always_comb
        case (Sel) // 2->1 multiplexor
            0: Out <= In0; 
            1: Out <= In1;           
            default: Out <= In0; 
        endcase
endmodule
