`timescale 1ns/1ps

//=============================================================================
// ahb_steer.v
//-----------------------------------------------------------------------------
// Address steering unit: ONE AHB master input, TWO AHB master outputs.
// Splits the CPU data-side traffic between the two fabrics:
//
//   haddr[31] == 0  ->  memory fabric output      (SRAM / flash / default)
//   haddr[31] == 1  ->  peripheral fabric output  (APB island @ 0x8000_0000)
//
// This is what makes the "separate fabrics for memory and peripherals" work
// for the core: the CPU has a single data port, and every beat is steered by
// its address.  DMA and boot-engine masters do not need steering -- they are
// direct masters of the memory fabric and only ever touch memories.
//
// Protocol notes (matches obi_to_ahb / ahb_fabric conventions):
//   * the bridge asserts hbusreq with haddr already valid and holds both
//     stable for the whole beat, so the route select is sampled at request
//     time (sel_c) and latched at grant (sel_q) for the address+data phases;
//   * hbusreq is forwarded only to the selected fabric, so the other fabric's
//     arbiter never sees phantom requests;
//   * the bridge runs one beat at a time (request -> grant -> NONSEQ -> IDLE
//     -> data phase -> rvalid), so there is never a beat in flight on both
//     outputs at once.
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module ahb_steer (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- master input side (from the CPU data bridge) ----
  input  wire         in_hbusreq,
  input  wire [31:0]  in_haddr,
  input  wire [1:0]   in_htrans,
  input  wire         in_hwrite,
  input  wire [2:0]   in_hsize,
  input  wire [2:0]   in_hburst,
  input  wire [3:0]   in_hprot,
  input  wire [31:0]  in_hwdata,
  output wire         in_hgrant,
  output wire         in_hready,
  output wire [1:0]   in_hresp,
  output wire [31:0]  in_hrdata,

  // ---- memory fabric master output ----
  output wire         mem_hbusreq,
  output wire [31:0]  mem_haddr,
  output wire [1:0]   mem_htrans,
  output wire         mem_hwrite,
  output wire [2:0]   mem_hsize,
  output wire [2:0]   mem_hburst,
  output wire [3:0]   mem_hprot,
  output wire [31:0]  mem_hwdata,
  input  wire         mem_hgrant,
  input  wire         mem_hready,
  input  wire [1:0]   mem_hresp,
  input  wire [31:0]  mem_hrdata,

  // ---- peripheral fabric master output ----
  output wire         per_hbusreq,
  output wire [31:0]  per_haddr,
  output wire [1:0]   per_htrans,
  output wire         per_hwrite,
  output wire [2:0]   per_hsize,
  output wire [2:0]   per_hburst,
  output wire [3:0]   per_hprot,
  output wire [31:0]  per_hwdata,
  input  wire         per_hgrant,
  input  wire         per_hready,
  input  wire [1:0]   per_hresp,
  input  wire [31:0]  per_hrdata
);

  // route select: 1 = peripheral fabric, 0 = memory fabric
  wire sel_c = in_haddr[31];

  reg  sel_q;
  always @(posedge clk_i) begin
    if (!rst_ni)                        sel_q <= 1'b0;
    else if (in_hbusreq && in_hgrant)   sel_q <= sel_c;   // latched at grant
  end

  // ---- request phase: busreq routed by the live address ----
  assign mem_hbusreq = in_hbusreq && !sel_c;
  assign per_hbusreq = in_hbusreq &&  sel_c;
  assign in_hgrant   = sel_c ? per_hgrant : mem_hgrant;

  // ---- address + data phase: everything routed by the latched select ----
  assign mem_htrans  = sel_q ? 2'b00     : in_htrans;
  assign mem_haddr   = in_haddr;
  assign mem_hwrite  = in_hwrite;
  assign mem_hsize   = in_hsize;
  assign mem_hburst  = in_hburst;
  assign mem_hprot   = in_hprot;
  assign mem_hwdata  = in_hwdata;

  assign per_htrans  = sel_q ? in_htrans : 2'b00;
  assign per_haddr   = in_haddr;
  assign per_hwrite  = in_hwrite;
  assign per_hsize   = in_hsize;
  assign per_hburst  = in_hburst;
  assign per_hprot   = in_hprot;
  assign per_hwdata  = in_hwdata;

  // ---- response mux ----
  assign in_hready = sel_q ? per_hready : mem_hready;
  assign in_hresp  = sel_q ? per_hresp  : mem_hresp;
  assign in_hrdata = sel_q ? per_hrdata : mem_hrdata;

endmodule
