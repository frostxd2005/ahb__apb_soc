`timescale 1ns/1ps

//=============================================================================
// ahb_fabric.v  — HARD-CODED edition
//------------------------------------------------------------------------------
// Multi-master AHB-Lite fabric: 4 masters, 6 slaves, everything hard-coded
// (no parameters, no `include, no macros, no generate, no loops, only
// constant bit ranges). Plain Verilog (IEEE 1364-2001).
//
//   masters: 0 = CPU instruction, 1 = CPU data, 2 = DMA, 3 = reserved
//   slaves : 0 = SRAM 96 kB          (range 0x0000_0000..0x0001_7FFF)
//            1 = flash 512 kB        (range 0x0002_0000..0x0009_FFFF)
//            2 = APB bridge          (base 0x8000_0000, mask 0xFFFF_0000)
//            3 = reserved            (base 0x8001_0000, mask 0xFFFF_0000)
//            4 = reserved            (base 0x8002_0000, mask 0xFFFF_0000)
//            5 = default/error responder (everything else)
//
// Model (classic two-phase select):
//   * grant is combinational; a master that presents non-IDLE HTRANS keeps the
//     bus (burst hold); otherwise round-robin with port 0 (CPU instruction)
//     having absolute priority.
//   * the address phase is combinational: hsel to the slaves is the decode of
//     the granted master's current address, gated by HTRANS != IDLE.
//   * the data phase is tracked by d_sel_q: which slave's HREADY/HRDATA/HRESP
//     feeds the masters.  d_sel_q advances only when the current data phase
//     completes (hready_o == 1), so wait states are handled correctly.
//
// Port widths (fixed): 4 masters x 32-bit addr/data, 6 slaves x 32-bit.
//   master m: haddr_i[m*32 +: 32] -> written below with constant slices
//             ([31:0], [63:32], [95:64], [127:96]).
//=============================================================================

module ahb_fabric (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- master request ports (4 masters, flattened) ----
  input  wire [3:0]   hbusreq_i,
  input  wire [127:0] haddr_i,
  input  wire [7:0]   htrans_i,
  input  wire [3:0]   hwrite_i,
  input  wire [11:0]  hsize_i,
  input  wire [11:0]  hburst_i,
  input  wire [15:0]  hprot_i,
  input  wire [127:0] hwdata_i,

  // ---- master response outputs (shared HREADY) ----
  output wire [3:0]   hgrant_o,
  output wire         hready_o,
  output wire [1:0]   hresp_o,
  output wire [31:0]  hrdata_o,

  // ---- slave interface (6 slaves, flattened) ----
  output wire [5:0]   slave_hsel_o,
  output wire [191:0] slave_haddr_o,
  output wire [5:0]   slave_hwrite_o,
  output wire [17:0]  slave_hsize_o,
  output wire [17:0]  slave_hburst_o,
  output wire [23:0]  slave_hprot_o,
  output wire [191:0] slave_hwdata_o,
  input  wire [5:0]   slave_hready_i,
  input  wire [11:0]  slave_hresp_i,
  input  wire [191:0] slave_hrdata_i
);

  // ===========================================================================
  // Arbitration
  // ===========================================================================
  reg [1:0] grant_q;   // registered grant pointer
  reg [1:0] grant_c;   // combinational next grant

  // HTRANS of the currently granted master (constant slices only)
  wire [1:0] grantee_htrans =
    (grant_q == 2'd0) ? htrans_i[1:0] :
    (grant_q == 2'd1) ? htrans_i[3:2] :
    (grant_q == 2'd2) ? htrans_i[5:4] :
                        htrans_i[7:6];

  // BUSREQ of the currently granted master (constant slices only)
  wire grantee_busreq =
    (grant_q == 2'd0) ? hbusreq_i[0] :
    (grant_q == 2'd1) ? hbusreq_i[1] :
    (grant_q == 2'd2) ? hbusreq_i[2] :
                        hbusreq_i[3];

  // Keep the bus while the grantee is mid-transfer (NONSEQ/SEQ/BUSY) OR has
  // its request pending (BUSREQ high, address phase not yet presented).
  // Without the second term a master could lose the grant in the 1-2 idle
  // cycles between seeing HGRANT and presenting NONSEQ; its address phase
  // would be ignored and it would latch another master's data phase.
  // Masters must therefore deassert BUSREQ during their data phase (the
  // data phase itself is protected by d_sel_q), which keeps multi-beat
  // DMA transfers interleaved with CPU accesses.
  wire hold_grant = (grantee_htrans != 2'b00) || grantee_busreq;

  // round-robin from (grant_q + 1), port 0 absolute priority
  always @(*) begin
    grant_c = grant_q;
    if (!hold_grant) begin
      if (grant_q == 2'd0) begin
        if      (hbusreq_i[1]) grant_c = 2'd1;
        else if (hbusreq_i[2]) grant_c = 2'd2;
        else if (hbusreq_i[3]) grant_c = 2'd3;
      end else if (grant_q == 2'd1) begin
        if      (hbusreq_i[2]) grant_c = 2'd2;
        else if (hbusreq_i[3]) grant_c = 2'd3;
        else if (hbusreq_i[0]) grant_c = 2'd0;
      end else if (grant_q == 2'd2) begin
        if      (hbusreq_i[3]) grant_c = 2'd3;
        else if (hbusreq_i[0]) grant_c = 2'd0;
        else if (hbusreq_i[1]) grant_c = 2'd1;
      end else begin
        if      (hbusreq_i[0]) grant_c = 2'd0;
        else if (hbusreq_i[1]) grant_c = 2'd1;
        else if (hbusreq_i[2]) grant_c = 2'd2;
      end
      // port 0 (CPU instruction fetch) absolute priority
      if (hbusreq_i[0]) grant_c = 2'd0;
    end
  end

  // one-hot grant output
  assign hgrant_o = (grant_c == 2'd0) ? 4'b0001 :
                    (grant_c == 2'd1) ? 4'b0010 :
                    (grant_c == 2'd2) ? 4'b0100 : 4'b1000;

  // ===========================================================================
  // Mux the granted master's request fields (constant slices only)
  // ===========================================================================
  wire [31:0] cur_addr =
    (grant_c == 2'd0) ? haddr_i[31:0] :
    (grant_c == 2'd1) ? haddr_i[63:32] :
    (grant_c == 2'd2) ? haddr_i[95:64] :
                        haddr_i[127:96];

  wire [1:0] cur_trans =
    (grant_c == 2'd0) ? htrans_i[1:0] :
    (grant_c == 2'd1) ? htrans_i[3:2] :
    (grant_c == 2'd2) ? htrans_i[5:4] :
                        htrans_i[7:6];

  wire cur_write =
    (grant_c == 2'd0) ? hwrite_i[0] :
    (grant_c == 2'd1) ? hwrite_i[1] :
    (grant_c == 2'd2) ? hwrite_i[2] :
                        hwrite_i[3];

  wire [2:0] cur_size =
    (grant_c == 2'd0) ? hsize_i[2:0] :
    (grant_c == 2'd1) ? hsize_i[5:3] :
    (grant_c == 2'd2) ? hsize_i[8:6] :
                        hsize_i[11:9];

  wire [2:0] cur_burst =
    (grant_c == 2'd0) ? hburst_i[2:0] :
    (grant_c == 2'd1) ? hburst_i[5:3] :
    (grant_c == 2'd2) ? hburst_i[8:6] :
                        hburst_i[11:9];

  wire [3:0] cur_prot =
    (grant_c == 2'd0) ? hprot_i[3:0] :
    (grant_c == 2'd1) ? hprot_i[7:4] :
    (grant_c == 2'd2) ? hprot_i[11:8] :
                        hprot_i[15:12];

  wire [31:0] cur_wdata =
    (grant_c == 2'd0) ? hwdata_i[31:0] :
    (grant_c == 2'd1) ? hwdata_i[63:32] :
    (grant_c == 2'd2) ? hwdata_i[95:64] :
                        hwdata_i[127:96];

  // ===========================================================================
  // Address decode (hard-coded map; slave 5 = default/error)
  // ===========================================================================
  wire [5:0] hit;
  assign hit[0] = (cur_addr < 32'h0001_8000); // SRAM aperture: 96 kB @0x0000_0000
  assign hit[1] = (cur_addr >= 32'h0002_0000) && (cur_addr < 32'h000A_0000); // flash: 512 kB @0x0002_0000
  assign hit[2] = ((cur_addr & 32'hFFFF_0000) == 32'h8000_0000); // APB island
  assign hit[3] = ((cur_addr & 32'hFFFF_0000) == 32'h8001_0000); // reserved
  assign hit[4] = ((cur_addr & 32'hFFFF_0000) == 32'h8002_0000); // reserved
  assign hit[5] = ~(hit[0] | hit[1] | hit[2] | hit[3] | hit[4]); // default

  wire present_valid = (cur_trans != 2'b00);
  wire [5:0] new_hsel = present_valid ? hit : 6'b000000;

  // ===========================================================================
  // Data phase select + response mux
  // ===========================================================================
  reg [5:0] d_sel_q;

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      grant_q <= 2'd0;
      d_sel_q <= 6'd0;
    end else begin
      grant_q <= grant_c;
      if (hready_o) d_sel_q <= new_hsel;
      // if hready_o == 0 (wait state) d_sel_q holds the current transfer
    end
  end

  reg [31:0] rdata_c;
  reg        rready_c;
  reg [1:0]  rresp_c;

  always @(*) begin
    rdata_c  = 32'd0;
    rready_c = 1'b1;
    rresp_c  = 2'b00;
    if      (d_sel_q[0]) begin rdata_c = slave_hrdata_i[31:0];   rready_c = slave_hready_i[0]; rresp_c = slave_hresp_i[1:0];  end
    else if (d_sel_q[1]) begin rdata_c = slave_hrdata_i[63:32];  rready_c = slave_hready_i[1]; rresp_c = slave_hresp_i[3:2];  end
    else if (d_sel_q[2]) begin rdata_c = slave_hrdata_i[95:64];  rready_c = slave_hready_i[2]; rresp_c = slave_hresp_i[5:4];  end
    else if (d_sel_q[3]) begin rdata_c = slave_hrdata_i[127:96]; rready_c = slave_hready_i[3]; rresp_c = slave_hresp_i[7:6];  end
    else if (d_sel_q[4]) begin rdata_c = slave_hrdata_i[159:128];rready_c = slave_hready_i[4]; rresp_c = slave_hresp_i[9:8];  end
    else if (d_sel_q[5]) begin rdata_c = slave_hrdata_i[191:160];rready_c = slave_hready_i[5]; rresp_c = slave_hresp_i[11:10];end
  end

  assign hready_o = rready_c;
  assign hresp_o  = rresp_c;
  assign hrdata_o = rdata_c;

  // ===========================================================================
  // Slave address-phase outputs (every slave sees the granted transfer)
  // ===========================================================================
  assign slave_hsel_o   = new_hsel;
  assign slave_haddr_o  = {cur_addr, cur_addr, cur_addr, cur_addr, cur_addr, cur_addr};
  assign slave_hwrite_o = {6{cur_write}};
  assign slave_hsize_o  = {cur_size, cur_size, cur_size, cur_size, cur_size, cur_size};
  assign slave_hburst_o = {cur_burst, cur_burst, cur_burst, cur_burst, cur_burst, cur_burst};
  assign slave_hprot_o  = {cur_prot, cur_prot, cur_prot, cur_prot, cur_prot, cur_prot};
  assign slave_hwdata_o = {cur_wdata, cur_wdata, cur_wdata, cur_wdata, cur_wdata, cur_wdata};

endmodule
