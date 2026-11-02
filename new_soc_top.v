`timescale 1ns/1ps

//=============================================================================
// new_soc_top.v
//------------------------------------------------------------------------------
// Top level of the CV32E40P AHB/APB SoC (package new_soc), phases 1-5
// complete.  Plain Verilog (IEEE 1364-2001) for all SoC RTL; the vendor core
// is SystemVerilog (no defines needed; assertions off unless
// CV32E40P_ASSERT_ON is defined).
//
// Architecture (see doc/new_soc_block_diagram.svg):
//
//   INSTRUCTION PATH (dedicated, never touches the memory fabric)
//     core instr OBI -> obi_icache (4 kB) -> obi_to_ahb -> ahb_ibus_mux
//       ibus slave 0 = ahb_sram_dp port A   (0x0000_0000, read-only)
//       ibus slave 1 = flash_prefetch -> ahb_flash_dp port A (0x0002_0000)
//       ibus slave 2 = built-in error responder
//
//   DATA PATH
//     core data OBI -> obi_dcache (2 kB, write-through, device bypass for
//     addr[31]) -> obi_to_ahb -> ahb_steer:
//       addr[31]=0 -> MEMORY fabric (ahb_fabric instance)
//       addr[31]=1 -> PERIPHERAL fabric (ahb_fabric instance -> APB bridge)
//
//   MEMORY FABRIC (4 masters / 6 slaves, hard-coded ahb_fabric)
//     master 0 = CPU data (via steer)   [absolute priority]
//     master 1 = DMA0 memory-side steer
//     master 2 = DMA1 memory-side steer
//     master 3 = boot-copy engine
//     slave 0  = ahb_sram_dp port B  (96 kB @ 0x0000_0000, read/write)
//     slave 1  = ahb_flash_dp port B (512 kB @ 0x0002_0000, read-only model)
//     slaves 2..5 = tied error responders (the APB window is reached through
//                   the steer unit, never through this fabric)
//
//   PERIPHERAL FABRIC (4 masters / 6 slaves, hard-coded ahb_fabric)
//     master 0 = CPU data (via steer)
//     master 1 = DMA0 peripheral-side steer
//     master 2 = DMA1 peripheral-side steer
//     master 3 = tied off
//     slave 2  = ahb_to_apb bridge, 12 x 4 kB slots @ 0x8000_0000:
//       0 apb_uart | 1 apb_spi_master | 2 apb_i2c | 3 apb_i2s
//       4 apb_wdg  | 5 apb_crc        | 6 DMA0 regs | 7 DMA1 regs
//       8 intc_apb | 9 boot_ctrl      | 10/11 reserved (SDIO/USB dropped)
//     slaves 0/1/3/4/5 = tied error responders
//
//   INTERRUPTS (E40P legacy PULP handshake: irq_i level, irq_ack_o/irq_id_o)
//     intc_apb sources: 0 uart, 1 spi events, 2 i2c, 3 i2s, 4 watchdog,
//                       5 dma0 done, 6 dma1 done, 7 spare
//     The core vectors through its software trap table (mtvec).
//
//   BOOT (boot_ctrl, boot_mode_i pins sampled during reset)
//     2'b00 SRAM boot   : core released at once, boots at BOOT_ADDR
//     2'b01 flash XIP   : core released at once, boots at 0x0002_0000
//     2'b10 boot-copy   : core held in reset while the engine copies
//                         COPY_WORDS words flash -> SRAM, then boots at
//                         BOOT_ADDR
//     mtvec = boot_addr + MTVEC_ADDR (both default 0 -> legacy behaviour)
//
//   DMA peripheral request mux (same 8 lines feed both controllers):
//     0 uart int | 1 spi events | 2 i2c irq | 3 i2s tx req | 4 i2s rx req
//     5 watchdog irq | 6/7 spare
//=============================================================================

module new_soc_top #(
  parameter [31:0] BOOT_ADDR  = 32'h0000_0000,   // SRAM boot / copy destination
  parameter [31:0] MTVEC_ADDR = 32'h0000_0000,   // added to the boot address
  parameter SRAM_HEX  = "",                      // optional $readmemh image
  parameter FLASH_HEX = ""
)(
  input  wire clk_i,
  input  wire rst_ni,

  // boot mode selector (sampled while rst_ni is low)
  input  wire [1:0] boot_mode_i,

  // UART0 (APB slot 0)
  output wire uart_tx_o,
  input  wire uart_rx_i,
  output wire uart_int_o,

  // SPI0 (APB slot 1) - single-SPI, CS0
  output wire spi_clk_o,
  output wire spi_csn0_o,
  output wire spi_sdo0_o,
  input  wire spi_sdi0_i,
  output wire [1:0] spi_events_o,

  // I2C0 (APB slot 2) - open-drain pad interface
  input  wire i2c_scl_i,
  output wire i2c_scl_o,
  output wire i2c_scl_oen_o,
  input  wire i2c_sda_i,
  output wire i2c_sda_o,
  output wire i2c_sda_oen_o,
  output wire i2c_irq_o,

  // I2S0 (APB slot 3)
  output wire i2s_sck_o,
  output wire i2s_ws_o,
  output wire i2s_sd_o,
  input  wire i2s_sd_i,

  // watchdog (APB slot 4)
  output wire wdg_reset_o,

  // debug/observation
  output wire core_sleep_o,
  output wire dbg_halted_o,
  output wire [63:0] dbg_mcycle_o
);

  // ===========================================================================
  // Boot controller outputs / core complex reset
  // ===========================================================================
  wire [31:0] boot_addr_w;
  wire        boot_done_w;
  wire        core_rst_n;          // rst_ni gated by the boot-copy phase
  wire [31:0] mtvec_w = boot_addr_w + MTVEC_ADDR;

  // ===========================================================================
  // CPU <-> caches (OBI)
  // ===========================================================================
  wire        instr_req, instr_gnt, instr_rvalid;
  wire [31:0] instr_addr, instr_rdata;
  wire        data_req, data_gnt, data_rvalid, data_we;
  wire [3:0]  data_be;
  wire [31:0] data_addr, data_wdata, data_rdata;

  // interrupt controller <-> core
  wire [31:0] intc_irq;
  wire        irq_ack_w;
  wire [4:0]  irq_id_w;

  // ===========================================================================
  // CPU: CV32E40P (RV32IMC; COREV_PULP=0, FPU=0 defaults)
  // ===========================================================================
  cv32e40p_top u_cpu (
    .clk_i              (clk_i),
    .rst_ni             (core_rst_n),
    .pulp_clock_en_i    (1'b1),        // unused when COREV_CLUSTER=0
    .scan_cg_en_i       (1'b0),

    .boot_addr_i        (boot_addr_w),
    .mtvec_addr_i       (mtvec_w),
    .dm_halt_addr_i     (32'h0),
    .hart_id_i          (32'h0),
    .dm_exception_addr_i(32'h0),

    // instruction OBI (via I-cache)
    .instr_req_o        (instr_req),
    .instr_gnt_i        (instr_gnt),
    .instr_rvalid_i     (instr_rvalid),
    .instr_addr_o       (instr_addr),
    .instr_rdata_i      (instr_rdata),

    // data OBI (via D-cache)
    .data_req_o         (data_req),
    .data_gnt_i         (data_gnt),
    .data_rvalid_i      (data_rvalid),
    .data_we_o          (data_we),
    .data_be_o          (data_be),
    .data_addr_o        (data_addr),
    .data_wdata_o       (data_wdata),
    .data_rdata_i       (data_rdata),

    // interrupts: external controller (intc_apb, APB slot 8)
    .irq_i              (intc_irq),
    .irq_ack_o          (irq_ack_w),
    .irq_id_o           (irq_id_w),

    // debug
    .debug_req_i        (1'b0),
    .debug_havereset_o  (),
    .debug_running_o    (),
    .debug_halted_o     (dbg_halted_o),

    .fetch_enable_i     (1'b1),
    .core_sleep_o       (core_sleep_o)
  );

  // E40P does not export mcycle; keep the top-level port for TB compatibility
  assign dbg_mcycle_o = 64'h0;

  // ===========================================================================
  // INSTRUCTION PATH: I-cache -> bridge -> ibus mux -> {sram A, prefetch}
  // ===========================================================================
  wire        ic_m_req, ic_m_gnt, ic_m_rvalid, ic_m_err;
  wire [31:0] ic_m_addr, ic_m_rdata;

  obi_icache u_icache (
    .clk_i(clk_i), .rst_ni(core_rst_n),
    .req_i(instr_req), .gnt_o(instr_gnt), .addr_i(instr_addr),
    .rvalid_o(instr_rvalid), .rdata_o(instr_rdata), .err_o(),
    .m_req_o(ic_m_req), .m_gnt_i(ic_m_gnt), .m_addr_o(ic_m_addr),
    .m_rvalid_i(ic_m_rvalid), .m_rdata_i(ic_m_rdata), .m_err_i(ic_m_err)
  );

  wire        obi0_hbusreq, obi1_hbusreq;
  wire [31:0] obi0_haddr, obi1_haddr;
  wire [1:0]  obi0_htrans, obi1_htrans;
  wire        obi0_hwrite, obi1_hwrite;
  wire [2:0]  obi0_hsize, obi1_hsize;
  wire [2:0]  obi0_hburst, obi1_hburst;
  wire [3:0]  obi0_hprot, obi1_hprot;
  wire [31:0] obi0_hwdata, obi1_hwdata;

  // I-bus mux master-side response wires
  wire        ib_hgrant, ib_hready;
  wire [1:0]  ib_hresp;
  wire [31:0] ib_hrdata;

  obi_to_ahb u_obi_i (
    .req_i  (ic_m_req),
    .addr_i (ic_m_addr),
    .we_i   (1'b0),
    .be_i   (4'hF),
    .wdata_i(32'h0),
    .gnt_o  (ic_m_gnt),
    .rdata_o(ic_m_rdata),
    .rvalid_o(ic_m_rvalid),
    .err_o  (ic_m_err),
    .hbusreq_o(obi0_hbusreq), .hgrant_i(ib_hgrant),
    .haddr_o(obi0_haddr), .htrans_o(obi0_htrans), .hwrite_o(obi0_hwrite),
    .hsize_o(obi0_hsize), .hburst_o(obi0_hburst), .hprot_o(obi0_hprot),
    .hwdata_o(obi0_hwdata),
    .hready_i(ib_hready), .hrdata_i(ib_hrdata), .hresp_i(ib_hresp),
    .clk_i(clk_i), .rst_ni(core_rst_n)
  );

  // ibus mux slave-side wires
  wire        ibs0_hsel, ibs1_hsel;
  wire [31:0] ibs0_haddr, ibs1_haddr;
  wire        ibs0_hwrite, ibs1_hwrite;
  wire [2:0]  ibs0_hsize;
  wire        ibs0_hready, ibs1_hready;
  wire [1:0]  ibs0_hresp, ibs1_hresp;
  wire [31:0] ibs0_hrdata, ibs1_hrdata;

  ahb_ibus_mux u_ibus (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .hbusreq_i(obi0_hbusreq), .haddr_i(obi0_haddr), .htrans_i(obi0_htrans),
    .hwrite_i(obi0_hwrite), .hsize_i(obi0_hsize), .hburst_i(obi0_hburst),
    .hprot_i(obi0_hprot), .hwdata_i(obi0_hwdata),
    .hgrant_o(ib_hgrant), .hready_o(ib_hready),
    .hresp_o(ib_hresp), .hrdata_o(ib_hrdata),
    .s0_hsel_o(ibs0_hsel), .s0_haddr_o(ibs0_haddr), .s0_hwrite_o(ibs0_hwrite),
    .s0_hsize_o(ibs0_hsize),
    .s0_hready_i(ibs0_hready), .s0_hresp_i(ibs0_hresp), .s0_hrdata_i(ibs0_hrdata),
    .s1_hsel_o(ibs1_hsel), .s1_haddr_o(ibs1_haddr), .s1_hwrite_o(ibs1_hwrite),
    .s1_hready_i(ibs1_hready), .s1_hresp_i(ibs1_hresp), .s1_hrdata_i(ibs1_hrdata)
  );

  // flash prefetch buffer <-> flash port A
  wire        pa_hsel, pa_hwrite, pa_hready_o, pa_hready_i;
  wire [31:0] pa_haddr, pa_hrdata;
  wire [1:0]  pa_hresp;

  flash_prefetch u_prefetch (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .hsel_i(ibs1_hsel), .hready_i(ib_hready), .haddr_i(ibs1_haddr),
    .hwrite_i(ibs1_hwrite),
    .hready_o(ibs1_hready), .hresp_o(ibs1_hresp), .hrdata_o(ibs1_hrdata),
    .pa_hsel_o(pa_hsel), .pa_haddr_o(pa_haddr), .pa_hwrite_o(pa_hwrite),
    .pa_hready_o(pa_hready_o),
    .pa_hready_i(pa_hready_i), .pa_hresp_i(pa_hresp), .pa_hrdata_i(pa_hrdata)
  );

  // ===========================================================================
  // DATA PATH: D-cache -> bridge -> steer -> {memory fabric, periph fabric}
  // ===========================================================================
  wire        dc_m_req, dc_m_gnt, dc_m_rvalid, dc_m_err, dc_m_we;
  wire [3:0]  dc_m_be;
  wire [31:0] dc_m_addr, dc_m_wdata, dc_m_rdata;

  obi_dcache u_dcache (
    .clk_i(clk_i), .rst_ni(core_rst_n),
    .req_i(data_req), .gnt_o(data_gnt), .addr_i(data_addr),
    .we_i(data_we), .be_i(data_be), .wdata_i(data_wdata),
    .rvalid_o(data_rvalid), .rdata_o(data_rdata), .err_o(),
    .m_req_o(dc_m_req), .m_gnt_i(dc_m_gnt), .m_addr_o(dc_m_addr),
    .m_we_o(dc_m_we), .m_be_o(dc_m_be), .m_wdata_o(dc_m_wdata),
    .m_rvalid_i(dc_m_rvalid), .m_rdata_i(dc_m_rdata), .m_err_i(dc_m_err)
  );

  // steer unit master-side (from the data bridge) response wires
  wire        st_in_hgrant, st_in_hready;
  wire [1:0]  st_in_hresp;
  wire [31:0] st_in_hrdata;

  obi_to_ahb u_obi_d (
    .req_i  (dc_m_req),
    .addr_i (dc_m_addr),
    .we_i   (dc_m_we),
    .be_i   (dc_m_be),
    .wdata_i(dc_m_wdata),
    .gnt_o  (dc_m_gnt),
    .rdata_o(dc_m_rdata),
    .rvalid_o(dc_m_rvalid),
    .err_o  (dc_m_err),
    .hbusreq_o(obi1_hbusreq), .hgrant_i(st_in_hgrant),
    .haddr_o(obi1_haddr), .htrans_o(obi1_htrans), .hwrite_o(obi1_hwrite),
    .hsize_o(obi1_hsize), .hburst_o(obi1_hburst), .hprot_o(obi1_hprot),
    .hwdata_o(obi1_hwdata),
    .hready_i(st_in_hready), .hrdata_i(st_in_hrdata), .hresp_i(st_in_hresp),
    .clk_i(clk_i), .rst_ni(core_rst_n)
  );

  // steer outputs: memory-fabric side and peripheral-fabric side
  wire        st_mem_hbusreq, st_per_hbusreq;
  wire [31:0] st_mem_haddr, st_per_haddr;
  wire [1:0]  st_mem_htrans, st_per_htrans;
  wire        st_mem_hwrite, st_per_hwrite;
  wire [2:0]  st_mem_hsize, st_per_hsize;
  wire [2:0]  st_mem_hburst, st_per_hburst;
  wire [3:0]  st_mem_hprot, st_per_hprot;
  wire [31:0] st_mem_hwdata, st_per_hwdata;
  wire        st_mem_hgrant, st_per_hgrant;
  wire        st_mem_hready, st_per_hready;
  wire [1:0]  st_mem_hresp, st_per_hresp;
  wire [31:0] st_mem_hrdata, st_per_hrdata;

  ahb_steer u_steer (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .in_hbusreq(obi1_hbusreq), .in_haddr(obi1_haddr), .in_htrans(obi1_htrans),
    .in_hwrite(obi1_hwrite), .in_hsize(obi1_hsize), .in_hburst(obi1_hburst),
    .in_hprot(obi1_hprot), .in_hwdata(obi1_hwdata),
    .in_hgrant(st_in_hgrant), .in_hready(st_in_hready),
    .in_hresp(st_in_hresp), .in_hrdata(st_in_hrdata),
    .mem_hbusreq(st_mem_hbusreq), .mem_haddr(st_mem_haddr),
    .mem_htrans(st_mem_htrans), .mem_hwrite(st_mem_hwrite),
    .mem_hsize(st_mem_hsize), .mem_hburst(st_mem_hburst),
    .mem_hprot(st_mem_hprot), .mem_hwdata(st_mem_hwdata),
    .mem_hgrant(st_mem_hgrant), .mem_hready(st_mem_hready),
    .mem_hresp(st_mem_hresp), .mem_hrdata(st_mem_hrdata),
    .per_hbusreq(st_per_hbusreq), .per_haddr(st_per_haddr),
    .per_htrans(st_per_htrans), .per_hwrite(st_per_hwrite),
    .per_hsize(st_per_hsize), .per_hburst(st_per_hburst),
    .per_hprot(st_per_hprot), .per_hwdata(st_per_hwdata),
    .per_hgrant(st_per_hgrant), .per_hready(st_per_hready),
    .per_hresp(st_per_hresp), .per_hrdata(st_per_hrdata)
  );

  // ===========================================================================
  // MEMORY FABRIC (masters: CPU-D, DMA0, DMA1, boot engine)
  // ===========================================================================
  wire [3:0]   mf_hgrant;
  wire         mf_hready;
  wire [1:0]   mf_hresp;
  wire [31:0]  mf_hrdata;
  wire [5:0]   mf_s_hsel;
  wire [191:0] mf_s_haddr;
  wire [5:0]   mf_s_hwrite;
  wire [17:0]  mf_s_hsize;
  wire [17:0]  mf_s_hburst;
  wire [23:0]  mf_s_hprot;
  wire [191:0] mf_s_hwdata;
  wire [5:0]   mf_s_hready;
  wire [11:0]  mf_s_hresp;
  wire [191:0] mf_s_hrdata;

  // DMA0 / DMA1 / boot engine master wires
  wire        dma0_hbusreq, dma1_hbusreq, boot_hbusreq;
  wire [31:0] dma0_haddr, dma1_haddr, boot_haddr;
  wire [1:0]  dma0_htrans, dma1_htrans, boot_htrans;
  wire        dma0_hwrite, dma1_hwrite, boot_hwrite;
  wire [2:0]  dma0_hsize, dma1_hsize, boot_hsize;
  wire [2:0]  dma0_hburst, dma1_hburst, boot_hburst;
  wire [31:0] dma0_hwdata, dma1_hwdata, boot_hwdata;

  // DMA address steering: each controller sees the response from whichever
  // fabric corresponds to the current beat address (addr[31]).
  wire dma0_m_hbusreq, dma0_p_hbusreq, dma1_m_hbusreq, dma1_p_hbusreq;
  wire [31:0] dma0_m_haddr, dma0_p_haddr, dma1_m_haddr, dma1_p_haddr;
  wire [1:0]  dma0_m_htrans, dma0_p_htrans, dma1_m_htrans, dma1_p_htrans;
  wire dma0_m_hwrite, dma0_p_hwrite, dma1_m_hwrite, dma1_p_hwrite;
  wire [2:0] dma0_m_hsize, dma0_p_hsize, dma1_m_hsize, dma1_p_hsize;
  wire [2:0] dma0_m_hburst, dma0_p_hburst, dma1_m_hburst, dma1_p_hburst;
  wire [3:0] dma0_m_hprot, dma0_p_hprot, dma1_m_hprot, dma1_p_hprot;
  wire [31:0] dma0_m_hwdata, dma0_p_hwdata, dma1_m_hwdata, dma1_p_hwdata;
  wire dma0_hgrant, dma1_hgrant, dma0_hready, dma1_hready;
  wire [1:0] dma0_hresp, dma1_hresp;
  wire [31:0] dma0_hrdata, dma1_hrdata;

  // Peripheral-fabric master response/arbiter wires are declared here so the
  // DMA steers can be placed before either fabric instance.
  wire [3:0]   pf_hgrant;
  wire         pf_hready;
  wire [1:0]   pf_hresp;
  wire [31:0]  pf_hrdata;
  wire [5:0]   pf_s_hsel;
  wire [191:0] pf_s_haddr;
  wire [5:0]   pf_s_hwrite;
  wire [17:0]  pf_s_hsize;
  wire [17:0]  pf_s_hburst;
  wire [23:0]  pf_s_hprot;
  wire [191:0] pf_s_hwdata;
  wire [5:0]   pf_s_hready;
  wire [11:0]  pf_s_hresp;
  wire [191:0] pf_s_hrdata;

  ahb_steer u_dma0_steer (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .in_hbusreq(dma0_hbusreq), .in_haddr(dma0_haddr),
    .in_htrans(dma0_htrans), .in_hwrite(dma0_hwrite),
    .in_hsize(dma0_hsize), .in_hburst(dma0_hburst),
    .in_hprot(4'b0011), .in_hwdata(dma0_hwdata),
    .in_hgrant(dma0_hgrant), .in_hready(dma0_hready),
    .in_hresp(dma0_hresp), .in_hrdata(dma0_hrdata),
    .mem_hbusreq(dma0_m_hbusreq), .mem_haddr(dma0_m_haddr),
    .mem_htrans(dma0_m_htrans), .mem_hwrite(dma0_m_hwrite),
    .mem_hsize(dma0_m_hsize), .mem_hburst(dma0_m_hburst),
    .mem_hprot(dma0_m_hprot), .mem_hwdata(dma0_m_hwdata),
    .mem_hgrant(mf_hgrant[1]), .mem_hready(mf_hready),
    .mem_hresp(mf_hresp), .mem_hrdata(mf_hrdata),
    .per_hbusreq(dma0_p_hbusreq), .per_haddr(dma0_p_haddr),
    .per_htrans(dma0_p_htrans), .per_hwrite(dma0_p_hwrite),
    .per_hsize(dma0_p_hsize), .per_hburst(dma0_p_hburst),
    .per_hprot(dma0_p_hprot), .per_hwdata(dma0_p_hwdata),
    .per_hgrant(pf_hgrant[1]), .per_hready(pf_hready),
    .per_hresp(pf_hresp), .per_hrdata(pf_hrdata)
  );

  ahb_steer u_dma1_steer (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .in_hbusreq(dma1_hbusreq), .in_haddr(dma1_haddr),
    .in_htrans(dma1_htrans), .in_hwrite(dma1_hwrite),
    .in_hsize(dma1_hsize), .in_hburst(dma1_hburst),
    .in_hprot(4'b0011), .in_hwdata(dma1_hwdata),
    .in_hgrant(dma1_hgrant), .in_hready(dma1_hready),
    .in_hresp(dma1_hresp), .in_hrdata(dma1_hrdata),
    .mem_hbusreq(dma1_m_hbusreq), .mem_haddr(dma1_m_haddr),
    .mem_htrans(dma1_m_htrans), .mem_hwrite(dma1_m_hwrite),
    .mem_hsize(dma1_m_hsize), .mem_hburst(dma1_m_hburst),
    .mem_hprot(dma1_m_hprot), .mem_hwdata(dma1_m_hwdata),
    .mem_hgrant(mf_hgrant[2]), .mem_hready(mf_hready),
    .mem_hresp(mf_hresp), .mem_hrdata(mf_hrdata),
    .per_hbusreq(dma1_p_hbusreq), .per_haddr(dma1_p_haddr),
    .per_htrans(dma1_p_htrans), .per_hwrite(dma1_p_hwrite),
    .per_hsize(dma1_p_hsize), .per_hburst(dma1_p_hburst),
    .per_hprot(dma1_p_hprot), .per_hwdata(dma1_p_hwdata),
    .per_hgrant(pf_hgrant[2]), .per_hready(pf_hready),
    .per_hresp(pf_hresp), .per_hrdata(pf_hrdata)
  );

  ahb_fabric u_mem_fabric (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .hbusreq_i({boot_hbusreq, dma1_m_hbusreq, dma0_m_hbusreq, st_mem_hbusreq}),
    .haddr_i  ({boot_haddr, dma1_m_haddr, dma0_m_haddr, st_mem_haddr}),
    .htrans_i ({boot_htrans, dma1_m_htrans, dma0_m_htrans, st_mem_htrans}),
    .hwrite_i ({boot_hwrite, dma1_m_hwrite, dma0_m_hwrite, st_mem_hwrite}),
    .hsize_i  ({boot_hsize, dma1_m_hsize, dma0_m_hsize, st_mem_hsize}),
    .hburst_i ({boot_hburst, dma1_m_hburst, dma0_m_hburst, st_mem_hburst}),
    .hprot_i  ({4'b0011, dma1_m_hprot, dma0_m_hprot, st_mem_hprot}),
    .hwdata_i ({boot_hwdata, dma1_m_hwdata, dma0_m_hwdata, st_mem_hwdata}),
    .hgrant_o(mf_hgrant), .hready_o(mf_hready),
    .hresp_o(mf_hresp), .hrdata_o(mf_hrdata),
    .slave_hsel_o(mf_s_hsel), .slave_haddr_o(mf_s_haddr),
    .slave_hwrite_o(mf_s_hwrite), .slave_hsize_o(mf_s_hsize),
    .slave_hburst_o(mf_s_hburst), .slave_hprot_o(mf_s_hprot),
    .slave_hwdata_o(mf_s_hwdata),
    .slave_hready_i(mf_s_hready), .slave_hresp_i(mf_s_hresp),
    .slave_hrdata_i(mf_s_hrdata)
  );

  assign st_mem_hgrant = mf_hgrant[0];
  assign st_mem_hready = mf_hready;
  assign st_mem_hresp  = mf_hresp;
  assign st_mem_hrdata = mf_hrdata;

  // ===========================================================================
  // MEMORIES (dual-port: A = instruction bus, B = memory fabric)
  // ===========================================================================
  ahb_sram_dp #(
    .MEM_SIZE_BYTES(96 * 1024),
    .INIT_HEX_FILE (SRAM_HEX),
    .INIT_HEX_ALT2 ("fw_sram.hex"),        // manually dropped into xsim's dir
    .INIT_HEX_ALT  ("sim/fw_sram.hex")     // searched upwards from xsim's CWD
  ) u_sram (
    .clk_i(clk_i), .rst_ni(rst_ni),
    // port A: instruction bus
    .a_hsel_i(ibs0_hsel), .a_hready_i(ib_hready), .a_haddr_i(ibs0_haddr),
    .a_hwrite_i(ibs0_hwrite), .a_hsize_i(ibs0_hsize),
    .a_hready_o(ibs0_hready), .a_hresp_o(ibs0_hresp), .a_hrdata_o(ibs0_hrdata),
    // port B: memory fabric slave 0
    .b_hsel_i(mf_s_hsel[0]), .b_hready_i(mf_hready),
    .b_haddr_i(mf_s_haddr[31:0]), .b_hwrite_i(mf_s_hwrite[0]),
    .b_hsize_i(mf_s_hsize[2:0]), .b_hwdata_i(mf_s_hwdata[31:0]),
    .b_hready_o(mf_s_hready[0]), .b_hresp_o(mf_s_hresp[1:0]),
    .b_hrdata_o(mf_s_hrdata[31:0])
  );

  ahb_flash_dp #(
    .MEM_SIZE_BYTES(512 * 1024),
    .FLASH_BASE    (32'h0002_0000),
    .INIT_HEX_FILE (FLASH_HEX),
    .INIT_HEX_ALT2 ("fw_flash.hex"),
    .INIT_HEX_ALT  ("sim/fw_flash.hex")
  ) u_flash (
    .clk_i(clk_i), .rst_ni(rst_ni),
    // port A: instruction prefetch buffer
    .a_hsel_i(pa_hsel), .a_hready_i(pa_hready_o), .a_haddr_i(pa_haddr),
    .a_hwrite_i(pa_hwrite),
    .a_hready_o(pa_hready_i), .a_hresp_o(pa_hresp), .a_hrdata_o(pa_hrdata),
    // port B: memory fabric slave 1
    .b_hsel_i(mf_s_hsel[1]), .b_hready_i(mf_hready),
    .b_haddr_i(mf_s_haddr[63:32]), .b_hwrite_i(mf_s_hwrite[1]),
    .b_hready_o(mf_s_hready[1]), .b_hresp_o(mf_s_hresp[3:2]),
    .b_hrdata_o(mf_s_hrdata[63:32])
  );

  // memory fabric slaves 2..5: tied error responders
  assign mf_s_hready[5:2]   = 4'b1111;
  assign mf_s_hresp[11:4]   = 8'b10_10_10_10;
  assign mf_s_hrdata[191:64] = 128'h0;

  // ===========================================================================
  // PERIPHERAL FABRIC (master 0 = steered CPU data; slave 2 = APB bridge 12)
  // ===========================================================================
  ahb_fabric u_periph_fabric (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .hbusreq_i({1'b0, dma1_p_hbusreq, dma0_p_hbusreq, st_per_hbusreq}),
    .haddr_i  ({32'h0, dma1_p_haddr, dma0_p_haddr, st_per_haddr}),
    .htrans_i ({2'b00, dma1_p_htrans, dma0_p_htrans, st_per_htrans}),
    .hwrite_i ({1'b0, dma1_p_hwrite, dma0_p_hwrite, st_per_hwrite}),
    .hsize_i  ({3'b000, dma1_p_hsize, dma0_p_hsize, st_per_hsize}),
    .hburst_i ({3'b000, dma1_p_hburst, dma0_p_hburst, st_per_hburst}),
    .hprot_i  ({4'b0000, dma1_p_hprot, dma0_p_hprot, st_per_hprot}),
    .hwdata_i ({32'h0, dma1_p_hwdata, dma0_p_hwdata, st_per_hwdata}),
    .hgrant_o(pf_hgrant), .hready_o(pf_hready),
    .hresp_o(pf_hresp), .hrdata_o(pf_hrdata),
    .slave_hsel_o(pf_s_hsel), .slave_haddr_o(pf_s_haddr),
    .slave_hwrite_o(pf_s_hwrite), .slave_hsize_o(pf_s_hsize),
    .slave_hburst_o(pf_s_hburst), .slave_hprot_o(pf_s_hprot),
    .slave_hwdata_o(pf_s_hwdata),
    .slave_hready_i(pf_s_hready), .slave_hresp_i(pf_s_hresp),
    .slave_hrdata_i(pf_s_hrdata)
  );

  assign st_per_hgrant = pf_hgrant[0];
  assign st_per_hready = pf_hready;
  assign st_per_hresp  = pf_hresp;
  assign st_per_hrdata = pf_hrdata;

  // ===========================================================================
  // AHB -> APB bridge (12 slots) on peripheral fabric slave 2
  // ===========================================================================
  wire [11:0]  p_psel;
  wire [143:0] p_paddr;
  wire         p_penable, p_pwrite;
  wire [31:0]  p_pwdata;
  wire [11:0]  p_pready;
  wire [11:0]  p_pslverr;
  wire [383:0] p_prdata;

  wire        apb_hready;
  wire [1:0]  apb_hresp;
  wire [31:0] apb_hrdata;

  ahb_to_apb #(
    .N_APB_SLAVES  (12),
    .APB_BASE_ADDR (32'h8000_0000)
  ) u_apb (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .hsel_i(pf_s_hsel[2]), .hready_i(pf_hready),
    .haddr_i(pf_s_haddr[95:64]), .hwrite_i(pf_s_hwrite[2]),
    .hsize_i(pf_s_hsize[8:6]), .hburst_i(pf_s_hburst[8:6]),
    .hprot_i(pf_s_hprot[11:8]), .hwdata_i(pf_s_hwdata[95:64]),
    .hready_o(apb_hready), .hresp_o(apb_hresp), .hrdata_o(apb_hrdata),
    .psel_o(p_psel), .paddr_o(p_paddr), .penable_o(p_penable),
    .pwrite_o(p_pwrite), .pwdata_o(p_pwdata),
    .pready_i(p_pready), .pslverr_i(p_pslverr), .prdata_i(p_prdata)
  );

  // peripheral fabric slaves 0/1/3/4/5: tied error responders (bridge = 2)
  assign pf_s_hready = {3'b111, apb_hready, 2'b11};
  assign pf_s_hresp  = {{3{2'b10}}, apb_hresp, {2{2'b10}}};
  assign pf_s_hrdata = {96'h0, apb_hrdata, 64'h0};

  // ===========================================================================
  // APB slot 0: UART
  // ===========================================================================
  wire [31:0] uart_prdata_w;
  wire uart_pready_w, uart_pslverr_w;
  wire uart_int_w;

  apb_uart u_uart (
    .CLK(clk_i), .RSTN(rst_ni),
    .PSEL(p_psel[0]), .PENABLE(p_penable), .PWRITE(p_pwrite),
    .PADDR(p_paddr[11:0]), .PWDATA(p_pwdata),
    .PRDATA(uart_prdata_w), .PREADY(uart_pready_w), .PSLVERR(uart_pslverr_w),
    .INT(uart_int_w),
    .OUT1N(), .OUT2N(), .RTSN(), .DTRN(),
    .CTSN(1'b1), .DSRN(1'b1), .DCDN(1'b1), .RIN(1'b1),
    .SIN(uart_rx_i), .SOUT(uart_tx_o)
  );

  assign uart_int_o = uart_int_w;

  // ===========================================================================
  // APB slot 1: SPI master
  // ===========================================================================
  wire [31:0] spi_prdata_w;
  wire spi_pready_w, spi_pslverr_w;
  wire spi_evt_any = |spi_events_o;

  apb_spi_master u_spi (
    .HCLK(clk_i), .HRESETn(rst_ni),
    .PSEL(p_psel[1]), .PENABLE(p_penable), .PWRITE(p_pwrite),
    .PADDR(p_paddr[23:12]), .PWDATA(p_pwdata),
    .PRDATA(spi_prdata_w), .PREADY(spi_pready_w), .PSLVERR(spi_pslverr_w),
    .events_o(spi_events_o),
    .spi_clk(spi_clk_o), .spi_csn0(spi_csn0_o),
    .spi_csn1(), .spi_csn2(), .spi_csn3(),
    .spi_mode(),
    .spi_sdo0(spi_sdo0_o), .spi_sdo1(), .spi_sdo2(), .spi_sdo3(),
    .spi_sdi0(spi_sdi0_i), .spi_sdi1(1'b0), .spi_sdi2(1'b0), .spi_sdi3(1'b0)
  );

  // ===========================================================================
  // APB slot 2: I2C master
  // ===========================================================================
  wire [31:0] i2c_prdata_w;
  wire i2c_pready_w, i2c_pslverr_w;
  wire i2c_irq_w;

  apb_i2c u_i2c (
    .HCLK(clk_i), .HRESETn(rst_ni),
    .PSEL(p_psel[2]), .PENABLE(p_penable), .PWRITE(p_pwrite),
    .PADDR(p_paddr[35:24]), .PWDATA(p_pwdata),
    .PRDATA(i2c_prdata_w), .PREADY(i2c_pready_w), .PSLVERR(i2c_pslverr_w),
    .interrupt_o(i2c_irq_w),
    .scl_pad_i(i2c_scl_i), .scl_pad_o(i2c_scl_o), .scl_padoen_o(i2c_scl_oen_o),
    .sda_pad_i(i2c_sda_i), .sda_pad_o(i2c_sda_o), .sda_padoen_o(i2c_sda_oen_o)
  );

  assign i2c_irq_o = i2c_irq_w;

  // ===========================================================================
  // APB slot 3: I2S
  // ===========================================================================
  wire [31:0] i2s_prdata_w;
  wire i2s_pready_w, i2s_pslverr_w;
  wire i2s_irq_w, i2s_tx_req, i2s_rx_req;

  apb_i2s u_i2s (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .sck_o(i2s_sck_o), .ws_o(i2s_ws_o), .sd_o(i2s_sd_o), .sd_i(i2s_sd_i),
    .irq_o(i2s_irq_w), .tx_req_o(i2s_tx_req), .rx_req_o(i2s_rx_req),
    .psel_i(p_psel[3]), .penable_i(p_penable), .pwrite_i(p_pwrite),
    .paddr_i(p_paddr[47:36]), .pwdata_i(p_pwdata),
    .prdata_o(i2s_prdata_w), .pready_o(i2s_pready_w), .pslverr_o(i2s_pslverr_w)
  );

  // ===========================================================================
  // APB slot 4: watchdog
  // ===========================================================================
  wire [31:0] wdg_prdata_w;
  wire wdg_pready_w, wdg_pslverr_w;
  wire wdg_irq_w;

  apb_wdg u_wdg (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .irq_o(wdg_irq_w), .reset_req_o(wdg_reset_o),
    .psel_i(p_psel[4]), .penable_i(p_penable), .pwrite_i(p_pwrite),
    .paddr_i(p_paddr[59:48]), .pwdata_i(p_pwdata),
    .prdata_o(wdg_prdata_w), .pready_o(wdg_pready_w), .pslverr_o(wdg_pslverr_w)
  );

  // ===========================================================================
  // APB slot 5: CRC
  // ===========================================================================
  wire [31:0] crc_prdata_w;
  wire crc_pready_w, crc_pslverr_w;

  apb_crc u_crc (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .psel_i(p_psel[5]), .penable_i(p_penable), .pwrite_i(p_pwrite),
    .paddr_i(p_paddr[71:60]), .pwdata_i(p_pwdata),
    .prdata_o(crc_prdata_w), .pready_o(crc_pready_w), .pslverr_o(crc_pslverr_w)
  );

  // ===========================================================================
  // DMA peripheral request lines (same wiring for both controllers)
  //   0 uart | 1 spi events | 2 i2c | 3 i2s tx | 4 i2s rx | 5 wdg | 6/7 spare
  // ===========================================================================
  wire [7:0] dma_req_w = {2'b00, wdg_irq_w, i2s_rx_req, i2s_tx_req,
                          i2c_irq_w, spi_evt_any, uart_int_w};

  // ===========================================================================
  // APB slots 6/7: DMA0 / DMA1 (8 streams each; AHB address-steered to
  // memory or peripheral fabric)
  // ===========================================================================
  wire [31:0] dma0_prdata_w, dma1_prdata_w;
  wire dma0_pready_w, dma0_pslverr_w, dma1_pready_w, dma1_pslverr_w;
  wire dma0_irq_w, dma1_irq_w;

  dma8_top u_dma0 (
    .clk(clk_i), .rst_n(rst_ni),
    .dma_req_i(dma_req_w), .dma_irq_o(dma0_irq_w),
    .psel(p_psel[6]), .penable(p_penable), .pwrite(p_pwrite),
    .paddr({20'h0, p_paddr[83:72]}), .pwdata(p_pwdata),
    .prdata(dma0_prdata_w), .pready(dma0_pready_w), .pslverr(dma0_pslverr_w),
    .HGRANT(dma0_hgrant), .HBUSREQ(dma0_hbusreq),
    .HREADY(dma0_hready), .HRDATA(dma0_hrdata),
    .HWDATA(dma0_hwdata), .HSIZE(dma0_hsize), .HADDR(dma0_haddr),
    .HTRANS(dma0_htrans), .HBURST(dma0_hburst), .HWRITE(dma0_hwrite)
  );

  dma8_top u_dma1 (
    .clk(clk_i), .rst_n(rst_ni),
    .dma_req_i(dma_req_w), .dma_irq_o(dma1_irq_w),
    .psel(p_psel[7]), .penable(p_penable), .pwrite(p_pwrite),
    .paddr({20'h0, p_paddr[95:84]}), .pwdata(p_pwdata),
    .prdata(dma1_prdata_w), .pready(dma1_pready_w), .pslverr(dma1_pslverr_w),
    .HGRANT(dma1_hgrant), .HBUSREQ(dma1_hbusreq),
    .HREADY(dma1_hready), .HRDATA(dma1_hrdata),
    .HWDATA(dma1_hwdata), .HSIZE(dma1_hsize), .HADDR(dma1_haddr),
    .HTRANS(dma1_htrans), .HBURST(dma1_hburst), .HWRITE(dma1_hwrite)
  );

  // ===========================================================================
  // APB slot 8: interrupt controller
  // ===========================================================================
  wire [31:0] intc_prdata_w;
  wire intc_pready_w, intc_pslverr_w;
  wire [7:0]  intc_src_w = {1'b0, dma1_irq_w, dma0_irq_w, wdg_irq_w,
                            i2s_irq_w, i2c_irq_w, spi_evt_any, uart_int_w};

  intc_apb u_intc (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .src_i(intc_src_w),
    .irq_o(intc_irq), .irq_ack_i(irq_ack_w), .irq_id_i(irq_id_w),
    .psel_i(p_psel[8]), .penable_i(p_penable), .pwrite_i(p_pwrite),
    .paddr_i(p_paddr[107:96]), .pwdata_i(p_pwdata),
    .prdata_o(intc_prdata_w), .pready_o(intc_pready_w),
    .pslverr_o(intc_pslverr_w)
  );

  // ===========================================================================
  // APB slot 9: boot controller (+ copy engine on memory fabric master 3)
  // ===========================================================================
  wire [31:0] boot_prdata_w;
  wire boot_pready_w, boot_pslverr_w;

  boot_ctrl #(
    .FLASH_BASE (32'h0002_0000),
    .SRAM_BASE  (BOOT_ADDR),
    .COPY_WORDS (4096)
  ) u_boot (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .boot_mode_i(boot_mode_i),
    .boot_addr_o(boot_addr_w), .boot_done_o(boot_done_w),
    .core_rst_no(core_rst_n),
    .psel_i(p_psel[9]), .penable_i(p_penable), .pwrite_i(p_pwrite),
    .paddr_i(p_paddr[119:108]), .pwdata_i(p_pwdata),
    .prdata_o(boot_prdata_w), .pready_o(boot_pready_w),
    .pslverr_o(boot_pslverr_w),
    .hbusreq_o(boot_hbusreq), .hgrant_i(mf_hgrant[3]),
    .haddr_o(boot_haddr), .htrans_o(boot_htrans), .hwrite_o(boot_hwrite),
    .hsize_o(boot_hsize), .hburst_o(boot_hburst), .hwdata_o(boot_hwdata),
    .hready_i(mf_hready), .hrdata_i(mf_hrdata), .hresp_i(mf_hresp)
  );

  // ===========================================================================
  // APB response merge (slots 10/11 reserved: pready=1, prdata=0)
  // ===========================================================================
  assign p_prdata  = {32'h0, 32'h0, boot_prdata_w, intc_prdata_w,
                      dma1_prdata_w, dma0_prdata_w, crc_prdata_w, wdg_prdata_w,
                      i2s_prdata_w, i2c_prdata_w, spi_prdata_w, uart_prdata_w};
  assign p_pready  = {2'b11, boot_pready_w, intc_pready_w,
                      dma1_pready_w, dma0_pready_w, crc_pready_w, wdg_pready_w,
                      i2s_pready_w, i2c_pready_w, spi_pready_w, uart_pready_w};
  assign p_pslverr = {2'b00, boot_pslverr_w, intc_pslverr_w,
                      dma1_pslverr_w, dma0_pslverr_w, crc_pslverr_w,
                      wdg_pslverr_w, i2s_pslverr_w, i2c_pslverr_w,
                      spi_pslverr_w, uart_pslverr_w};

endmodule
