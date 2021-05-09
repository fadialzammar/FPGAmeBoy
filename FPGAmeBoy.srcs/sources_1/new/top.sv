`timescale 1ns / 1ps

// Top level wrapper for the FPGAmeBoy

module top(
    input CLK,
    input RST,
    input [4:0] INTR,
    // Display outputs
    output logic VGA_HS, VGA_VS,
    output logic [3:0] VGA_RED, VGA_GREEN, VGA_BLUE
);

////////////////////////////
// Clock Divider (10MHz clock)  /*** Currently not connected ***/
////////////////////////////
    logic SCLK;
    
    C_DIV divider(
        .CLK        (CLK),
        .CLK_DIV    (SCLK)
    );
    
////////////////////////////
// Boot ROM
////////////////////////////
logic [15:0] PROG_COUNT;
logic [15:0] BROM_ADDR;
logic [7:0] BROM_IR;
BROM BROM(
    .CLK            (CLK),
    .BROM_ADDR      (BROM_ADDR),
    .BROM_IR        (BROM_IR)
);

////////////////////////////
//  ProgRom / Cartridge
////////////////////////////
logic [7:0] CPU_OPCODE;
ProgRom ProgRom(
    .PROG_CLK       (CLK),
    .PROG_ADDR      (PROG_COUNT),
    .PROG_IR        (CPU_OPCODE)
);

////////////////////////////
// CPU
////////////////////////////
logic [7:0] CPU_DATA_IN, CPU_DATA_OUT;
logic [15:0] CPU_ADDR_OUT;
logic CPU_WE_OUT, CPU_RE_OUT;

CPU_Wrapper CPU(
    .CLK            (CLK),
    .RST            (RST),
    .MEM_DOUT       (CPU_DATA_IN),
    .OPCODE         (CPU_OPCODE),
    .INT_EN         (D_IE),
    .INT_FLAG       (D_IF),

    .MEM_DIN        (CPU_DATA_OUT),
    .MEM_WE         (CPU_WE_OUT),
    .MEM_RE         (CPU_RE_OUT),
    .MEM_ADDR_IN    (CPU_ADDR_OUT),
    .PC             (PROG_COUNT),
    .INTR_ID         (INT_ID),
    .INT_CLR        (INT_CLR)
);

////////////////////////////
// RAM C000-DFFF
////////////////////////////
logic MEM_WE, MEM_RE, MEM_HOLD;
logic [15:0] MEM_ADDR;
logic [7:0] MEM_DIN, MEM_DOUT;

assign MEM_HOLD = ~MEM_RE;

Memory Memory(
    .CLK            (CLK), 
    .WE             (MEM_WE),
    .HOLD           (MEM_HOLD), 
    .ADDR           (MEM_ADDR), 
    .DIN            (MEM_DIN),
    .DOUT           (MEM_DOUT)
);

////////////////////////////
// PPU
////////////////////////////
logic PPU_WE, PPU_RE, PPU_HOLD;
logic [15:0] PPU_ADDR;
logic [7:0] PPU_DIN, PPU_DOUT;

logic VRAM_WE, VRAM_RE, VRAM_HOLD;
logic [15:0] VRAM_ADDR;
logic [7:0] VRAM_DIN, VRAM_DOUT;

logic OAM_WE, OAM_RE, OAM_HOLD;
logic [15:0] OAM_ADDR;
logic [7:0] OAM_DIN, OAM_DOUT;

logic PIXEL_CLK, PIXEL_VALID;
logic [1:0] PPU_PIXEL;

assign PPU_RE = ~PPU_HOLD;      // TODO: replace HOLDs for REs
assign VRAM_RE = ~VRAM_HOLD;
assign OAM_RE = ~OAM_HOLD;

logic [7:0] ppu_debug_scx;
logic [7:0] ppu_debug_scy;
logic [4:0] ppu_debug_state;
ppu PPU(
    .clk            (CLK),
    .rst            (RST),
    // MMIO Bus, 0xFF40 - 0xFF4B, always visible to CPU
    .mmio_a         (PPU_ADDR),
    .mmio_dout      (PPU_DOUT),
    .mmio_din       (PPU_DIN),
    .mmio_rd        (PPU_RE),
    .mmio_wr        (PPU_WE),
    // VRAM Bus, 0x800 - 0x9FFF
    .vram_a         (VRAM_ADDR),
    .vram_dout      (VRAM_DOUT),
    .vram_din       (VRAM_DIN),
    .vram_rd        (VRAM_RE),
    .vram_wr        (VRAM_WE),
    // OAM Bus,  0xFE00 - 0xFE9F
    .oam_a          (OAM_ADDR),
    .oam_dout       (OAM_DOUT),
    .oam_din        (OAM_DIN),
    .oam_rd         (OAM_RE),
    .oam_wr         (OAM_WE),
    // Interrupt interface
    .int_vblank_req (),
    .int_lcdc_req   (),
    .int_vblank_ack (),
    .int_lcdc_ack   (),
    // Pixel output
    .cpl            (PIXEL_CLK),          // Pixel Clock, = ~clk
    .pixel          (PPU_PIXEL),        // Pixel Output
    .valid          (PIXEL_VALID),  // Pixel Valid
    .hs             (VGA_HS),           // Horizontal Sync, High Valid
    .vs             (VGA_VS),           // Vertical Sync, High Valid
    //Debug output
    .scx            (ppu_debug_scx),
    .scy            (ppu_debug_scy),
    .state          (ppu_debug_state)
    );
    
////////////////////////////
// DMA 
////////////////////////////
dma DMA(
    .clk                    (CLK),
    .rst                    (RST),
    .dma_rd                 (DMA_RE),
    .dma_wr                 (DMA_WE),
    .dma_a                  (DMA_ADDR),
    .dma_din                (DMA_DIN),
    .dma_dout               (DMA_DOUT),
    .mmio_wr                (DMA_MMIO_WE),
    .mmio_din               (CPU_DATA_OUT),
    .mmio_dout              (DMA_MMIO_DOUT),
    // DMA use
    // 0x0000 - 0x7FFF, 0xA000 - 0xFFFF
    .dma_occupy_extbus      (dma_occupy_extbus),
    // 0x8000 - 0x9FFF
    .dma_occupy_vidbus      (dma_occupy_vidbus),
    // 0xFE00 - 0xFE9F
    .dma_occupy_oambus      (dma_occupy_oambus)
);

////////////////////////////
// High RAM FF80-FFFE
////////////////////////////
logic [7:0] HRAM [0:127];
logic [15:0] HRAM_ADDR;
logic [7:0] HRAM_DIN, HRAM_DOUT;
logic HRAM_WE, HRAM_RE;

// Write on rising edge
always @(posedge CLK) 
begin
    if (HRAM_WE)
        begin
            HRAM[HRAM_ADDR] <= HRAM_DIN;
        end
end

// Read on falling edge
 always_ff@(negedge CLK)
    begin
        if (HRAM_RE)
        begin
            HRAM_DOUT <= HRAM[HRAM_ADDR];
        end
    end

    
////////////////////////////
// Memory Map
////////////////////////////
memory_map memory_map(
    //CPU 0000-FFFF
    .A_cpu              (CPU_ADDR_OUT),
    .Di_cpu             (CPU_DATA_IN),
    .Do_cpu             (CPU_DATA_OUT),
    .wr_cpu             (CPU_WE_OUT),
    .rd_cpu             (CPU_RE_OUT),
    // DMA (VRAM 8000-H9FFF) || (RAM C000-DFFF) || (OAM FE00-FE9F)
    .A_DMA              (DMA_ADDR),
	.Di_DMA             (DMA_DIN),
	.cs_DMA             (),
	.Do_DMA             (DMA_DOUT),
	.wr_DMA             (DMA_WE),
	.rd_DMA             (DMA_RE),
	.dma_occupy_vidbus  (dma_occupy_vidbus),
	.dma_occupy_oambus  (dma_occupy_oambus), 
	.dma_occupy_extbus  (dma_occupy_extbus),
	//Boot Rom 0000 - 00FF
	.A_BROM            (BROM_ADDR),
	.cs_BROM           (),
	.Do_BROM           (BROM_IR),
    //Cartridge 0100-7FFF & A000-BFFF    // TODO: Incorporate ProgRom into main memory space rather than reading directly from it?
    .A_crd          (),
    .Di_crd         (),
    .Do_crd         (PROG_IR),
    .cs_crd         (),
    .wr_crd         (),
    .rd_crd         (),
    //PPU (Video RAM) 8000-9FFF 
    .A_ppu_vram     (VRAM_ADDR),
    .Di_ppu_vram    (VRAM_DIN),
    .Do_ppu_vram    (VRAM_DOUT),
    .cs_ppu_vram    (),
    .wr_ppu_vram    (VRAM_WE),
    .rd_ppu_vram    (VRAM_RE),
    //PPU  (OAM) FE00-FE9F
    .A_ppu_oam      (OAM_ADDR),
    .Di_ppu_oam     (OAM_DIN),
    .Do_ppu_oam     (OAM_DOUT),
    .cs_ppu_oam     (),
    .wr_ppu_oam     (OAM_WE),
    .rd_ppu_oam     (OAM_RE),
    //PPU (MMIO) FF40-FF4B
    .A_ppu_regs     (FAKE_PPU_ADDR),
    .Di_ppu_regs    (FAKE_PPU_DIN),
    .Do_ppu_regs    (FAKE_PPU_DOUT),
    .cs_ppu_regs    (),
    .wr_ppu_regs    (FAKE_PPU_WE),
    .rd_ppu_regs    (FAKE_PPU_RE),
    //RAM C000-DFFF
    .A_ram          (MEM_ADDR),
    .Di_ram         (MEM_DIN),
    .Do_ram         (MEM_DOUT),
    .cs_ram         (),
    .wr_ram         (MEM_WE),
    .rd_ram         (MEM_RE),
    //Controller Manager FF00
    .A_ctrlMgr      (),
    .Di_ctrlMgr     (),
    .Do_ctrlMgr     (),
    .cs_ctrlMgr     (),
    .wr_ctrlMgr     (),
    .rd_ctrlMgr     (),
    //Timer FF04-FF07
    .A_timer        (),
    .Di_timer       (),
    .Do_timer       (),
    .cs_timer       (),
    .wr_timer       (),
    .rd_timer       (),
    //Working & Stack RAM FF05-FF40
    .A_wsram        (),
    .Di_wsram       (),
    .Do_wsram       (),
    .cs_wsram       (),
    .wr_wsram       (),
    .rd_wsram       (),
    // High Ram FF80-FFFE
    .A_HRAM         (HRAM_ADDR),
	.Di_HRAM        (HRAM_DIN),
	.Do_HRAM        (HRAM_DOUT),
	.cs_HRAM        (),
	.wr_HRAM        (HRAM_WE),
	.rd_HRAM        (HRAM_RE),
        // Hardware I/O Registers FF00-FF40
	.A_io           (A_io),
	.Di_io          (Di_io),
	.Do_io          (Do_io),
	.cs_io          (),
	.wr_io          (wr_io)
   );

////////////////////////////
// VGA
////////////////////////////
// Mapping 2-bit pixel color to a 12-bit VGA output
always_comb begin
    case (PPU_PIXEL)
        2'b00: begin
            VGA_RED = 4'hf;
            VGA_GREEN = 4'hf;
            VGA_BLUE = 4'hf;
        end
        2'b01: begin
            VGA_RED = 4'hb;
            VGA_GREEN = 4'hb;
            VGA_BLUE = 4'hb;
        end
        2'b10: begin
            VGA_RED = 4'h6;
            VGA_GREEN = 4'h6;
            VGA_BLUE = 4'h6;
        end
        2'b11: begin
            VGA_RED = 4'h0;
            VGA_GREEN = 4'h0;
            VGA_BLUE = 4'h0;
        end
    endcase
end

   
   
 
// IO Registers instantiation
   logic [15:0] A_io;
   logic [7:0] Di_io, Do_io;
   logic wr_io;
   logic [7:0] D_IF, D_IE;
   logic INT_CLR;
   logic [2:0] INT_ID;

   
   logic [7:0] INT_IN;
   assign INT_IN = {3'b000, INTR};
   
   // IO/Control Registers
    IO_Reg IO_Reg(
        .ADR(A_io[7:0]),
        .D_IN(Di_io),
        .CLK(CLK),
        .WE(wr_io),
        .INT_IN(INT_IN),
        .D_OUT(Do_io),
        .D_IE(D_IE),
        .D_IF(D_IF), 
        .INT_ID(INT_ID),
        .INT_CLR(INT_CLR)
    );
    
endmodule