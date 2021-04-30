`timescale 1ns / 1ps

// Top level wrapper for the FPGAmeBoy

module top(
    input CLK,
    input RST
);

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

logic VGA_CLK, VGA_HS, VGA_VS, VGA_PIXEL_VALID;
logic [1:0] VGA_PIXEL;

assign PPU_RE = ~PPU_HOLD;      // TODO: replace HOLDs for REs
assign VRAM_RE = ~VRAM_HOLD;
assign OAM_RE = ~OAM_HOLD;

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
    .cpl            (VGA_CLK),          // Pixel Clock, = ~clk
    .pixel          (VGA_PIXEL),        // Pixel Output
    .valid          (VGA_PIXEL_VALID),  // Pixel Valid
    .hs             (VGA_HS),           // Horizontal Sync, High Valid
    .vs             (VGA_VS),           // Vertical Sync, High Valid
    //Debug output
    .scx            (),
    .scy            (),
    .state          ()
    );

// This is not good
logic[7:0] HL_DATA, MEM_DATA_CPU;
logic HL_HOLD;
always_ff@(posedge CLK)
    begin
    if(HL_HOLD==1)
        HL_DATA <= MEM_DATA_CPU;
    end
    
MUX2to1#(.DATA_SIZE(8)) HL_HOLD_MUX(
    .In0(MEM_DATA_CPU), .In1(HL_DATA), .Out(CPU_DATA_IN), .Sel(HL_HOLD)
);    

memory_map memory_map(
    //CPU 0000-FFFF
    .A_cpu          (CPU_ADDR_OUT),
    .Di_cpu         (MEM_DATA_CPU),
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
    .rd_wsram       ()
   );

endmodule