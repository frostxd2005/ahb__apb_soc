`timescale 1ns/1ps

//=============================================================================
// dma_top.v
//------------------------------------------------------------------------------
// Top level of the 4-channel AHB/APB DMA controller. Adapted from
// varun23-2004/-4-Channel-AHB-APB-DMA-Controller-RTL-Design-.
//
// Integration (see bd_wiring_guide.md):
//   * APB slave side  -> one 4 kB slot of the ahb_to_apb bridge
//                        (this project: slot 6, base 0x8000_6000; connect the
//                        bridge's 12-bit window address to paddr[11:0] and
//                        tie paddr[31:12] = 0).
//   * AHB master side -> one master port of ahb_fabric (this project: port 2):
//                        HADDR/HTRANS/HSIZE/HBURST/HWRITE/HWDATA into the
//                        fabric's master-2 slices; HRDATA/HREADY from the
//                        fabric's shared response outputs; HGRANT from
//                        hgrant_o[2]; HBUSREQ into hbusreq_i[2].
//
// Changes over the original: the AHB master sub-module gained HGRANT/HBUSREQ
// (fabric arbitration handshake, same convention as obi_to_ahb) and is now
// named dma_ahb_master (was dma_ahb_mas).
//=============================================================================

module dma_top (
  input  wire        clk,
  input  wire        rst_n,

  // APB slave (configuration, from CPU via ahb_to_apb)
  input  wire        psel,
  input  wire        penable,
  input  wire        pwrite,
  input  wire [31:0] paddr,
  input  wire [31:0] pwdata,
  output wire [31:0] prdata,
  output wire        pready,
  output wire        pslverr,

  // AHB-Lite master (data movement, to ahb_fabric master port 2)
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
  wire [127:0] w_src;
  wire [127:0] w_dest;
  wire [127:0] w_count;

  // handshake signals
  wire [3:0] w_req;
  wire [3:0] w_grant;
  wire [3:0] w_transfer_done;

  // FIFO wires
  wire        w_fifo_w_en, w_fifo_r_en;
  wire [31:0] w_fifo_wdata;
  wire [31:0] w_fifo_rdata;
  wire        w_fifo_full, w_fifo_empty;

  dma_fifo_1 u_fifo (
    .clk(clk), .rst_n(rst_n),
    .w_en(w_fifo_w_en), .r_en(w_fifo_r_en),
    .data_in(w_fifo_wdata), .data_out(w_fifo_rdata),
    .full(w_fifo_full), .empty(w_fifo_empty)
  );

  dma_arbiter u_arbiter (
    .clk(clk), .rst_n(rst_n),
    .req(w_req), .grant(w_grant)
  );

  dma_apb_slave u_apb_slave (
    .clk(clk), .rstn(rst_n),
    .psel(psel), .penable(penable), .pwrite(pwrite),
    .pready(pready), .pslverr(pslverr),
    .paddr(paddr), .pwdata(pwdata), .prdata(prdata),
    .req(w_req), .done(w_transfer_done),
    .src_addr(w_src), .dest_addr(w_dest), .count_addr(w_count)
  );

  dma_ahb_master u_ahb_master (
    .clk(clk), .rstn(rst_n),
    .grant(w_grant),
    .transfer_done(w_transfer_done),
    .src_addr_flat(w_src), .dest_addr_flat(w_dest), .count_flat(w_count),
    .fifo_full(w_fifo_full), .fifo_empty(w_fifo_empty),
    .fifo_w_en(w_fifo_w_en), .fifo_r_en(w_fifo_r_en),
    .fifo_wdata(w_fifo_wdata), .fifo_rdata(w_fifo_rdata),
    .HGRANT(HGRANT), .HBUSREQ(HBUSREQ),
    .HREADY(HREADY), .HRDATA(HRDATA), .HWDATA(HWDATA),
    .HSIZE(HSIZE), .HADDR(HADDR), .HTRANS(HTRANS),
    .HBURST(HBURST), .HWRITE(HWRITE)
  );

endmodule
