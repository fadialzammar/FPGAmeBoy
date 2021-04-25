`timescale 1ns / 1ps

// Top level wrapper for the FPGAmeBoy

module top(
    input CLK,
    input RST
);

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

// Program ROM Instantiation
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

memory_map memory_map(
    //Cpu 0000-FFFF
    .A_cpu          (CPU_ADDR_OUT),
    .Di_cpu         (CPU_DATA_IN),
    .Do_cpu         (CPU_DATA_OUT),
    .wr_cpu         (CPU_WE_OUT),
    .rd_cpu         (CPU_RE_OUT),
    //Cartridge 0000-7FFF & A000-BFFF    // TODO: Incorporate ProgRom into main memory
    .A_crd          (),
    .Di_crd         (),
    .Do_crd         (),
    .cs_crd         (),
    .wr_crd         (),
    .rd_crd         (),
    //PPU 8000-9FFF & FE00-FE9F & FF40-FF4B
    .A_ppu          (PPU_ADDR),
    .Di_ppu         (PPU_DIN),
    .Do_ppu         (PPU_DOUT),
    .cs_ppu         (),
    .wr_ppu         (PPU_WE),
    .rd_ppu         (PPU_RE),
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


// PPU Instantiation
logic [15:0] PPU_ADDR;
logic [7:0] PPU_DIN, PPU_DOUT;
logic PPU_WE, PPU_RE;

// video_controller PPU(
//     .reset          (RST),
//     .clock          (CLK),

//     .int_vblank_ack (),
//     .int_vblank_req (),
//     .int_lcdc_ack   (),
//     .int_lcdc_req   (),

//     .A              (PPU_ADDR),
//     .Di             (PPU_DIN),
//     .Do             (PPU_DOUT),
//     .rd_n           ()
// );

logic VGA_CLK, VGA_HS, VGA_VS, VGA_PIXEL_VALID;
logic [1:0] VGA_PIXEL;

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
    .vram_a         (),
    .vram_dout      (),
    .vram_din       (),
    .vram_rd        (),
    .vram_wr        (),
    // OAM Bus,  0xFE00 - 0xFE9F
    .oam_a          (),
    .oam_dout       (),
    .oam_din        (),
    .oam_rd         (),
    .oam_wr         (),
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

endmodule