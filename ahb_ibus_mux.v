`timescale 1ns/1ps

//=============================================================================
// ahb_ibus_mux.v
//-----------------------------------------------------------------------------
// Instruction-bus interconnect: ONE master (the CPU instruction-side
// obi_to_ahb bridge, via the I-cache) and two external slaves plus a built-in
// error responder.  This is the "separate instruction bus" of the split I/D
// architecture -- instruction fetches never touch the memory fabric.
//
//   slave 0 = SRAM port A      (range 0x0000_0000..0x0001_7FFF)
//   slave 1 = flash port A via prefetch buffer
//                              (range 0x0002_0000..0x0009_FFFF)
//   slave 2 = built-in error responder (everything else)
//
// Model: same two-phase select as ahb_fabric (address-phase decode, data-phase
// response mux tracked by d_sel_q).  Single master -> grant is simply the
// request (no arbitration needed); this matches what ahb_fabric does for an
// uncontended port 0.  Error responses are single-cycle (hready=1, hresp=
// ERROR), the established convention of this project.
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================
`include "ahb_defines.v"

module ahb_ibus_mux (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- master port (CPU instruction side) ----
  input  wire         hbusreq_i,
  input  wire [31:0]  haddr_i,
  input  wire [1:0]   htrans_i,
  input  wire         hwrite_i,
  input  wire [2:0]   hsize_i,
  input  wire [2:0]   hburst_i,
  input  wire [3:0]   hprot_i,
  input  wire [31:0]  hwdata_i,
  output wire         hgrant_o,
  output wire         hready_o,
  output wire [1:0]   hresp_o,
  output wire [31:0]  hrdata_o,

  // ---- slave 0: SRAM port A ----
  output wire         s0_hsel_o,
  output wire [31:0]  s0_haddr_o,
  output wire         s0_hwrite_o,
  output wire [2:0]   s0_hsize_o,
  input  wire         s0_hready_i,
  input  wire [1:0]   s0_hresp_i,
  input  wire [31:0]  s0_hrdata_i,

  // ---- slave 1: flash prefetch buffer (flash port A) ----
  output wire         s1_hsel_o,
  output wire [31:0]  s1_haddr_o,
  output wire         s1_hwrite_o,
  input  wire         s1_hready_i,
  input  wire [1:0]   s1_hresp_i,
  input  wire [31:0]  s1_hrdata_i
);

  // single master: always granted (same as uncontended fabric port 0)
  assign hgrant_o = hbusreq_i;

  // -------------------------------------------------------------------------
  // Address-phase decode
  // -------------------------------------------------------------------------
  wire [2:0] hit;
  assign hit[0] = (haddr_i < 32'h0001_8000); // SRAM aperture: 96 kB @0x0000_0000
  assign hit[1] = (haddr_i >= 32'h0002_0000) && (haddr_i < 32'h000A_0000); // flash: 512 kB @0x0002_0000
  assign hit[2] = ~(hit[0] | hit[1]);                           // error

  wire present_valid = (htrans_i != `HTRANS_IDLE);
  wire [2:0] new_hsel = present_valid ? hit : 3'b000;

  // -------------------------------------------------------------------------
  // Data-phase tracking + response mux
  // -------------------------------------------------------------------------
  reg [2:0] d_sel_q;

  always @(posedge clk_i) begin
    if (!rst_ni) d_sel_q <= 3'b000;
    else if (hready_o) d_sel_q <= new_hsel;
  end

  reg [31:0] rdata_c;
  reg        rready_c;
  reg [1:0]  rresp_c;

  always @(*) begin
    rdata_c  = 32'h0;
    rready_c = 1'b1;
    rresp_c  = `HRESP_OKAY;
    if      (d_sel_q[0]) begin rdata_c = s0_hrdata_i; rready_c = s0_hready_i; rresp_c = s0_hresp_i; end
    else if (d_sel_q[1]) begin rdata_c = s1_hrdata_i; rready_c = s1_hready_i; rresp_c = s1_hresp_i; end
    else if (d_sel_q[2]) begin rdata_c = 32'h0;       rready_c = 1'b1;        rresp_c = `HRESP_ERROR; end
  end

  assign hready_o = rready_c;
  assign hresp_o  = rresp_c;
  assign hrdata_o = rdata_c;

  // -------------------------------------------------------------------------
  // Slave address-phase outputs
  // -------------------------------------------------------------------------
  assign s0_hsel_o   = new_hsel[0];
  assign s0_haddr_o  = haddr_i;
  assign s0_hwrite_o = hwrite_i;
  assign s0_hsize_o  = hsize_i;

  assign s1_hsel_o   = new_hsel[1];
  assign s1_haddr_o  = haddr_i;
  assign s1_hwrite_o = hwrite_i;

  // hburst_i / hprot_i / hwdata_i are irrelevant for instruction fetches and
  // are intentionally left unconnected inside this mux.

endmodule
