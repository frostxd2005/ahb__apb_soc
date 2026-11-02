`timescale 1ns/1ps

//=============================================================================
// dma8_top.v
//-----------------------------------------------------------------------------
// Top level of the 8-stream AHB/APB DMA controller with peripheral request
// multiplexor (the phase-5 DMA; two instances live in new_soc: DMA0 at APB
// slot 6 and DMA1 at slot 7).
//
//   * APB slave side  -> one 4 kB slot of the ahb_to_apb bridge (12-slot
//                        peripheral fabric).  Stream 0 register offsets are
//                        identical to the old 4-channel controller, so the
//                        existing firmware works unchanged.
//   * AHB master side -> one master port of the MEMORY fabric (DMAs only
//                        ever move data to/from memories; their registers
//                        are reached by the CPU, not by the DMA itself).
//   * peripheral request mux: each stream's CFG.REQ_SEL (bits 7:4) picks its
//                        trigger: 0 = software/memory-to-memory (runs as
//                        soon as START is set), n = 1..8 -> the stream only
//                        requests arbitration while dma_req_i[n-1] is high.
//                        Both DMA controllers see the same request lines
//                        (wired at the top level from UART/SPI/I2C/I2S/WDG).
//   * dma_irq_o pulses one cycle whenever any stream completes (INTC
//                        source, rising-edge captured).
//
// Plain Verilog (IEEE 1364-2001).
//============================================================================

module dma8_top (
  input  wire        clk,
  input  wire        rst_n,

  // peripheral request lines (shared by both controllers)
  input  wire [7:0]  dma_req_i,
  output wire        dma_irq_o,

  // APB slave (configuration, from CPU via the peripheral fabric)
  input  wire        psel,
  input  wire        penable,
  input  wire        pwrite,
  input  wire [31:0] paddr,
  input  wire [31:0] pwdata,
  output wire [31:0] prdata,
  output wire        pready,
  output wire        pslverr,

  // AHB-Lite master (data movement, memory fabric master port)
  input  wire        HGRANT,
  output wire        HBUSREQ,
  input  wire        HREADY,
  input  wire [31:0] HRDATA,
  output wire [31:0] HWDATA,
  output wire [2:0]  HSIZE,
  output wire [31:0] HADDR,
  output wire [1:0]  HTRANS,
  output wire [2:0]  HBURST,
  output wire        HWRITE
);

  // config wires (apb -> ahb master)
  wire [255:0] w_src;
  wire [255:0] w_dest;
  wire [255:0] w_count;
  wire [31:0]  w_reqsel;
  wire [7:0]   w_src_fixed;
  wire [7:0]   w_dest_fixed;

  // handshake signals
  wire [7:0] w_start;          // START bits from the register file
  wire [7:0] w_req;            // gated arbitration requests
  wire [7:0] w_grant;
  wire [7:0] w_transfer_done;

  // FIFO wires
  wire        w_fifo_w_en, w_fifo_r_en;
  wire [31:0] w_fifo_wdata;
  wire [31:0] w_fifo_rdata;
  wire        w_fifo_full, w_fifo_empty;

  // -------------------------------------------------------------------------
  // Peripheral request mux: stream m with REQ_SEL = n (1..8) only requests
  // while dma_req_i[n-1] is asserted; REQ_SEL = 0 is memory-to-memory.
  // -------------------------------------------------------------------------
  reg [7:0] preq_ok_c;
  reg [3:0] sel_m;
  integer   m;

  always @(*) begin
    for (m = 0; m < 8; m = m + 1) begin
      sel_m = w_reqsel[m*4 +: 4];
      if (sel_m == 4'd0)         preq_ok_c[m] = 1'b1;
      else if (sel_m <= 4'd8)    preq_ok_c[m] = dma_req_i[sel_m - 4'd1];
      else                       preq_ok_c[m] = 1'b0;   // reserved selector
    end
  end

  assign w_req = w_start & preq_ok_c;

  dma_fifo_1 u_fifo (
    .clk(clk), .rst_n(rst_n),
    .w_en(w_fifo_w_en), .r_en(w_fifo_r_en),
    .data_in(w_fifo_wdata), .data_out(w_fifo_rdata),
    .full(w_fifo_full), .empty(w_fifo_empty)
  );

  dma8_arbiter u_arbiter (
    .clk(clk), .rst_n(rst_n),
    .req(w_req), .grant(w_grant)
  );

  dma8_apb_slave u_apb_slave (
    .clk(clk), .rstn(rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite),
    .pready(pready), .pslverr(pslverr),
    .paddr(paddr), .pwdata(pwdata), .prdata(prdata),
    .req(w_start), .done(w_transfer_done),
    .req_sel_flat(w_reqsel), .src_fixed(w_src_fixed),
    .dest_fixed(w_dest_fixed),
    .src_addr(w_src), .dest_addr(w_dest), .count_addr(w_count)
  );

  dma8_ahb_master u_ahb_master (
    .clk(clk), .rstn(rst_n),
    .grant(w_grant),
    .transfer_done(w_transfer_done),
    .src_addr_flat(w_src), .dest_addr_flat(w_dest), .count_flat(w_count),
    .src_fixed(w_src_fixed), .dest_fixed(w_dest_fixed),
    .stream_start(w_start), .stream_req(w_req),
    .fifo_full(w_fifo_full), .fifo_empty(w_fifo_empty),
    .fifo_w_en(w_fifo_w_en), .fifo_r_en(w_fifo_r_en),
    .fifo_wdata(w_fifo_wdata), .fifo_rdata(w_fifo_rdata),
    .HGRANT(HGRANT), .HBUSREQ(HBUSREQ),
    .HREADY(HREADY), .HRDATA(HRDATA), .HWDATA(HWDATA),
    .HSIZE(HSIZE), .HADDR(HADDR), .HTRANS(HTRANS),
    .HBURST(HBURST), .HWRITE(HWRITE)
  );

  assign dma_irq_o = |w_transfer_done;

endmodule
