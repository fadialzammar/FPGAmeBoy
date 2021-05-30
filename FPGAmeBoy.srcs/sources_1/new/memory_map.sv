`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/07/2021 09:30:26 PM
// Design Name: 
// Module Name: memory_map
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

module memory_map(

	//Cpu 0000-FFFF
	input 	[15:0] 	A_cpu,
	output 	[7:0] 	Di_cpu,
	output  [7:0]   CPU_OPCODE,
	input 	[7:0] 	Do_cpu,
	input					wr_cpu,
	input					rd_cpu,
	
	// DMA (VRAM 8000-H9FFF) || (RAM C000-DFFF) || (OAM FE00-FE9F)
	input  [15:0]      A_DMA,
	output logic [7:0] Di_DMA,
	output logic       cs_DMA,
	input  [7:0]       Do_DMA,
	input              wr_DMA,
	input  [7:0]       Do_MMIO,
	output logic       wr_MMIO,
	input		       rd_DMA,
	input              dma_occupy_vidbus,
	input              dma_occupy_oambus,
	input              dma_occupy_extbus,         
	
	// Boot Rom 0000-00FF
	output logic [15:0]    A_BROM,    
	output logic           cs_BROM,
	input  [7:0]           Do_BROM,
	
	//Cartridge 0000-7FFF & A000-BFFF
	output 	[15:0] 	A_crd,
	output 	[7:0] 	Di_crd,
	input   [7:0]	Do_crd,
	output			cs_crd,
	output			wr_crd,
	output			rd_crd,

   //PPU (Video RAM)8000-9FFF 
	output 	[15:0] 	A_ppu_vram,
	output 	[7:0] 	Di_ppu_vram,
	input	[7:0]	Do_ppu_vram,
	output			cs_ppu_vram,
	output			wr_ppu_vram,
	output			rd_ppu_vram,
    
    //PPU  (OAM)FE00-FE9F 
	output 	[15:0] 	    A_ppu_oam,
	output 	[7:0] 	    Di_ppu_oam,
	input	[7:0]		Do_ppu_oam,
	output				cs_ppu_oam,
	output				wr_ppu_oam,
	output				rd_ppu_oam,
	
	//PPU (MMIO)FF40-FF4B
	output 	[15:0] 	A_ppu_regs,
	output 	[7:0] 	Di_ppu_regs,
	input	[7:0]		Do_ppu_regs,
	output				cs_ppu_regs,
	output				wr_ppu_regs,
	output				rd_ppu_regs,
	
	//RAM C000-DFFF
	output 	[15:0] 	A_ram,
	output 	[7:0] 	Di_ram,
	input	[7:0]		Do_ram,
	output 				cs_ram,
	output				wr_ram,
	output				rd_ram,
   
   //Controller Manager FF00
    output 	[15:0] 	A_ctrlMgr,
	output 	[7:0] 	Di_ctrlMgr,
	input	[7:0]		Do_ctrlMgr,
	output				cs_ctrlMgr,
	output				wr_ctrlMgr,
	output				rd_ctrlMgr,
   
   //Timer FF04-FF07
	output 	[15:0] 	A_timer,
	output 	[7:0] 	Di_timer,
	input		[7:0]		Do_timer,
	output				cs_timer,
	output				wr_timer,
	output				rd_timer,
   
	//Working & Stack RAM FF05-FF40
	output 	[15:0] 	A_wsram,
	output 	[7:0] 	Di_wsram,
	input		[7:0]		Do_wsram,
	output				cs_wsram,
	output				wr_wsram,
	output				rd_wsram,
	
	//High RAM FF80 - FFFE
	output 	[15:0] 	A_HRAM,
	output 	[7:0] 	Di_HRAM,
	input   [7:0]   Do_HRAM,
	output			cs_HRAM,
	output			wr_HRAM,
	output			rd_HRAM,
	
	// Interrupt Enable Flags FFFF
	output [15:0] A_io,
	output [7:0] Di_io,
	input [7:0] Do_io,
	output cs_io,
	output wr_io,
	
	// Joypad Manager
	output [7:0] Di_joy,
	input [7:0] Do_joy,
	output cs_joy,
	output wr_joy
);

assign A_crd = A_cpu;
//assign A_ppu_vram = dma_occupy_vidbus ? A_DMA : A_cpu;
//assign A_ppu_oam = dma_occupy_oambus ? A_DMA : A_cpu;

// TRY:
assign A_ppu_vram = A_cpu;
assign A_ppu_oam = A_cpu;
assign A_ppu_regs = A_cpu;
assign A_ram = A_cpu - 16'hC000;
assign A_wsram = A_cpu - 16'hFF00;
assign A_timer = A_cpu;
// Might be A_HRAM = A_cpu - 16'hFF80
assign A_HRAM = A_cpu - 16'hFF80;
//assign A_BROM = A_cpu;
assign A_io = A_cpu - 16'hFF00;

// Write Enable
assign wr_crd =   cs_crd ? wr_cpu : 1'b0;
//assign wr_ppu_vram = cs_ppu_vram & dma_occupy_vidbus ? 1'b0 : wr_cpu;
//assign wr_ppu_oam = cs_ppu_oam & dma_occupy_oambus ? wr_DMA : wr_cpu;
assign wr_ppu_vram = cs_ppu_vram ? wr_cpu : 1'b0;
assign wr_ppu_oam = cs_ppu_oam ? wr_cpu : 1'b0;
assign wr_ppu_regs = cs_ppu_regs ? wr_cpu : 1'b0;
assign wr_ram =   cs_ram ? wr_cpu : 1'b0;
assign wr_wsram = cs_wsram ? wr_cpu : 1'b0;
assign wr_ctrlMgr = cs_ctrlMgr ? wr_cpu : 1'b0;
assign wr_timer = cs_timer ? wr_cpu : 1'b0;
assign wr_HRAM = cs_HRAM ? wr_cpu : 1'b0;
assign wr_io = cs_io ? wr_cpu : 1'b0;
assign wr_joy = cs_joy ? wr_cpu : 1'b0;

assign wr_MMIO = cs_DMA ? wr_cpu : 1'b0;

// Read Enable
assign rd_crd =   cs_crd ? rd_cpu : 1'b0;
//assign rd_ppu_vram = cs_ppu_vram & dma_occupy_vidbus ? rd_DMA : rd_cpu;
//assign rd_ppu_oam = cs_ppu_oam & dma_occupy_oambus ? rd_DMA : rd_cpu;
assign rd_ppu_vram = cs_ppu_vram ? rd_cpu : 1'b0;
assign rd_ppu_oam = cs_ppu_oam ? rd_cpu : 1'b0;
assign rd_ppu_regs = cs_ppu_regs ? rd_cpu : 1'b0;
assign rd_ram =   cs_ram ? rd_cpu : 1'b0;
assign rd_wsram = cs_wsram ? rd_cpu : 1'b0;
assign rd_ctrlMgr = cs_ctrlMgr ? rd_cpu : 1'b0;
assign rd_timer = cs_timer ? rd_cpu : 1'b0;
assign rd_HRAM = cs_HRAM ? rd_cpu : 1'b0;

//assign cs_BROM = (A_cpu <= 16'h00FF);
// Chip Select Logic
assign cs_crd = (A_cpu >= 16'h0000 && A_cpu < 16'h8000) || (A_cpu >= 16'hA000 && A_cpu < 16'hC000);
assign cs_ppu_vram = (A_cpu >= 16'h8000 && A_cpu < 16'h9FFF);
assign cs_ppu_oam = (A_cpu >= 16'hFE00 && A_cpu < 16'hFEA0);
assign cs_ppu_regs = (A_cpu >= 16'hFF40 && A_cpu < 16'hFF4C);
// assign cs_DMA = (A_DMA >= 16'h8000 && A_DMA < 16'h9FFF) || (A_DMA >= 16'hC000 && A_DMA < 16'hDFFF) || (A_DMA >= 16'hFE00 && A_DMA < 16'hFE9F);
assign cs_DMA = A_cpu == 16'hff46; 
assign cs_ram = (A_cpu >= 16'hC000 && A_cpu < 16'hE000);
assign cs_timer = (A_cpu >= 16'hFF04 && A_cpu < 16'hFF08);
assign cs_HRAM = (A_cpu >= 16'hFF80 && A_cpu < 16'hFFFF);
assign cs_io = (A_cpu > 16'hFF00 && A_cpu < 16'hFF40) || (A_cpu == 16'hFFFF) || (A_cpu == 16'hFF50);
assign cs_joy = A_cpu == 16'hFF00;


// Select DMA input
// DMA (VRAM 8000-9FFF) || (RAM C000-DFFF) || (OAM FE00-FE9F)
always_comb
    begin
        // VRAM
        if (A_DMA >= 16'h8000 && A_DMA < 16'h9FFF)
            Di_DMA = Do_ppu_vram;
        // RAM
        else if (A_DMA >= 16'hC000 && A_DMA < 16'hDFFF)
            Di_DMA = Do_ram;
        // OAM -- May need to change
        else if (A_DMA >= 16'hFE00 && A_DMA < 16'hFE9F)
            Di_DMA = Do_ppu_oam;
            
        else
            Di_DMA = Do_cpu;
    end

//assign CPU_OPCODE = 
//                cs_BROM ? Do_BROM : (
//                cs_crd & ~dma_occupy_extbus? Do_crd : 8'hFF);              
                
//assign cs_ctrlMgr = A_cpu == 16'hFF00;
//assign cs_wsram = (A_cpu >= 16'hFF08 && A_cpu < 16'hFF40);


// Data read into CPU

//cs_crd & ~dma_occupy_extbus? 
assign Di_cpu = cs_crd ? Do_crd : (
                cs_DMA ? Do_MMIO : (
				cs_ppu_vram ? Do_ppu_vram : (
				cs_ppu_oam ? Do_ppu_oam : (
				cs_ppu_regs ? Do_ppu_regs : (
				cs_ram ? Do_ram : (
				cs_HRAM ? Do_HRAM : (
				cs_joy ? Do_joy : (
				cs_timer ? Do_timer : (
				cs_io ? Do_io : 8'b0 
				)))))))));

// Data write from CPU
assign Di_crd = cs_crd ? Do_cpu : 8'b0;
assign Di_ppu_vram = cs_ppu_vram ? Do_cpu : 8'b0;
//assign Di_ppu_oam = cs_ppu_oam & dma_occupy_oambus? Do_DMA : Do_cpu;
assign Di_ppu_oam = cs_ppu_oam ? Do_cpu : 8'b0;
assign Di_ppu_regs = cs_ppu_regs ? Do_cpu : 8'b0;
assign Di_ram = cs_ram ? Do_cpu : 8'b0;
assign Di_wsram = cs_wsram ? Do_cpu : 8'b0;
assign Di_HRAM = cs_HRAM ? Do_cpu : 8'b0;
assign Di_timer = cs_timer ? Do_cpu : 8'b0;
assign Di_io = cs_io ? Do_cpu : 8'b0;
assign Di_joy = cs_joy ? Do_cpu : 8'b0;


endmodule