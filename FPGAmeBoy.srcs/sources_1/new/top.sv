`timescale 1ns / 1ps

// Top level wrapper for the FPGAmeBoy

module top(
    input CLK100,
    input RST,
    // Display outputs
    output logic VGA_HS, VGA_VS, //PIXEL_VALID, PIXEL_CLK,
    // output logic [1:0] VGA_PIXEL
    output logic [3:0] VGA_RED, VGA_GREEN, VGA_BLUE
);
// Clock divider
// 100MHz oscillator clock to 4.19MHz Game Boy clock
logic [3:0] clk_counter = 0;
logic CLK = 0;
always_ff @ (posedge CLK100) begin
    if (RST) begin
        clk_counter <= 0;
        CLK <= 0;
    end
    else clk_counter <= clk_counter + 1;
    if (clk_counter == 9) begin
        CLK <= ~CLK;
    end
end

////////////////////////////
// CPU
////////////////////////////
logic [7:0] CPU_DATA_IN, CPU_DATA_OUT, CPU_OPCODE;
logic [15:0] CPU_ADDR_OUT;
logic CPU_WE_OUT, CPU_RE_OUT;

CPU_Wrapper CPU(
    .CLK            (CLK),
    .RST            (RST),
    .MEM_DOUT       (CPU_DATA_IN),
    .OPCODE         (CPU_OPCODE),

    .MEM_DIN        (CPU_DATA_OUT),
    .MEM_WE         (CPU_WE_OUT),
    .MEM_RE         (CPU_RE_OUT),
    .MEM_ADDR_IN    (CPU_ADDR_OUT),
    .PC             (PROG_COUNT)
);

////////////////////////////
// ProgRom / Cartridge
////////////////////////////
logic [15:0] PROG_COUNT;

ProgRom ProgRom(
    .PROG_CLK       (CLK),
    .PROG_ADDR      (PROG_COUNT),
    .PROG_IR        (CPU_OPCODE)
);

// Memory Instantiation
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

// logic VGA_VS, VGA_HS;

// assign VGA_nVS = VGA_VS;
// assign VGA_nHS = VGA_HS;

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
    
// ////////////////////////////
// // DMA 
// ////////////////////////////
// logic DMA_RE; // DMA Memory Write Enable
// logic DMA_WE; // DMA Memory Read Enable
// logic [15:0] DMA_ADDR; // Main Address Bus
// logic [7:0]  DMA_DIN; // Main Data Bus
// logic [7:0]  DMA_DOUT;
// logic [7:0]  DMA_MMIO_DOUT;
// logic DMA_MMIO_WE; 
// // Flags for indicating DMA use and CPU disable
// logic dma_occupy_extbus; // 0x0000 - 0x7FFF, 0xA000 - 0xFFFF
// logic dma_occupy_vidbus; // 0x8000 - 0x9FFF
// logic dma_occupy_oambus; // 0xFE00 - 0xFE9F

// // DMA Instantiation
// dma DMA(
//     .clk                    (CLK),
//     .rst                    (RST),
//     .dma_rd                 (DMA_RE),
//     .dma_wr                 (DMA_WE),
//     .dma_a                  (DMA_ADDR),
//     .dma_din                (DMA_DIN),
//     .dma_dout               (DMA_DOUT),
//     .mmio_wr                (DMA_MMIO_WE),
//     .mmio_din               (CPU_DATA_OUT),
//     .mmio_dout              (DMA_MMIO_DOUT),
//     // DMA use
//     // 0x0000 - 0x7FFF, 0xA000 - 0xFFFF
//     .dma_occupy_extbus      (dma_occupy_extbus),
//     // 0x8000 - 0x9FFF
//     .dma_occupy_vidbus      (dma_occupy_vidbus),
//     // 0xFE00 - 0xFE9F
//     .dma_occupy_oambus      (dma_occupy_oambus)
// );
    
// // DMA interface Logic
// // VRAM
// assign VRAM_ADDR = dma_occupy_vidbus ? DMA_ADDR : VRAM_ADDR;
// assign VRAM_WE   = dma_occupy_vidbus ? 1'b0     : VRAM_WE;
// assign VRAM_RE   = dma_occupy_vidbus ? DMA_RE   : VRAM_RE;
// // OAM RAM
// assign OAM_ADDR = dma_occupy_oambus ? DMA_ADDR : OAM_ADDR;
// assign OAM_WE   = dma_occupy_oambus ? DMA_WE   : OAM_WE;
// assign OAM_RE   = dma_occupy_oambus ? DMA_RE   : OAM_RE;
// assign OAM_DIN  = dma_occupy_oambus ? DMA_DOUT : OAM_DIN;

//always_comb 
//begin
//    // DMA in use
//    // Data, Adddres, and control set by DMA
//    // DMA using VRAM bus
//    if (dma_occupy_vidbus)
//        begin
//            VRAM_ADDR = DMA_ADDR;
//            VRAM_WE   = 1'b0;
//            VRAM_RE   = DMA_RE;
//        end
//    // DMA using OAM bus
//    else if (dma_occupy_oambus)
//        begin
//            OAM_ADDR = DMA_ADDR;
//            OAM_DIN  = DMA_DOUT;
//            OAM_WE   = DMA_WE;
//            OAM_RE   = DMA_RE;    
//        end
        
//     else
//        begin
//            VRAM_ADDR = CPU_DATA_OUT;
//            VRAM_WE   = CPU_WE_OUT;
//            VRAM_RE   = CPU_RE_OUT;
            
//            OAM_ADDR = CPU_ADDR_OUT;
//            OAM_DIN  = CPU_DATA_OUT;
//            OAM_WE   = CPU_WE_OUT;
//            OAM_RE   = CPU_RE_OUT;
//        end
//end

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
    .A_cpu          (CPU_ADDR_OUT),
    .Di_cpu         (CPU_DATA_IN),
    .Do_cpu         (CPU_DATA_OUT),
    .wr_cpu         (CPU_WE_OUT),
    .rd_cpu         (CPU_RE_OUT),
    //Cartridge 0000-7FFF & A000-BFFF    // TODO: Incorporate ProgRom into main memory space rather than reading directly from it?
    .A_crd          (),
    .Di_crd         (),
    .Do_crd         (),
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
    .A_ppu_regs     (PPU_ADDR),
    .Di_ppu_regs    (PPU_DIN),
    .Do_ppu_regs    (PPU_DOUT),
    .cs_ppu_regs    (),
    .wr_ppu_regs    (PPU_WE),
    .rd_ppu_regs    (PPU_RE),
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
	.rd_HRAM        (HRAM_RE)
   );

////////////////////////////
// VGA
////////////////////////////
// Mapping 2-bit pixel color to a 12-bit VGA output
always_comb begin
    case (PPU_PIXEL)
        2'b00: begin
            VGA_RED = 4'h0;
            VGA_GREEN = 4'h0;
            VGA_BLUE = 4'h0;
        end
        2'b01: begin
            VGA_RED = 4'h6;
            VGA_GREEN = 4'h6;
            VGA_BLUE = 4'h6;
        end
        2'b10: begin
            VGA_RED = 4'hb;
            VGA_GREEN = 4'hb;
            VGA_BLUE = 4'hb;
        end
        2'b11: begin
            VGA_RED = 4'hf;
            VGA_GREEN = 4'hf;
            VGA_BLUE = 4'hf;
        end
    endcase
end
endmodule