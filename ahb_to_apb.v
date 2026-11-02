`timescale 1ns/1ps

//=============================================================================
// ahb_to_apb.v
//------------------------------------------------------------------------------
// AHB-Lite MASTER -> APB3 MASTER bridge.  One AHB transfer = one APB transfer;
// APB wait states (PREADY=0) are reflected back as AHB wait states.
// Plain Verilog edition.
//
// PSEL[s] is the decode of the AHB address (4 kB per APB slave, base
// APB_BASE_ADDR).  All APB slaves see the same PADDR/PWRITE/PWDATA.
// Response timing: hready_o asserted only in DONE, after read data captured.
//
// Port convention (flattened): slave s occupies paddr_o[s*APB_ADDR_WIDTH +: APB_ADDR_WIDTH],
// prdata_i[s*32 +: 32], pready_i[s], pslverr_i[s].
//=============================================================================
`include "ahb_defines.v"

module ahb_to_apb #(
  parameter AHB_ADDR_WIDTH = 32,
  parameter APB_ADDR_WIDTH = 12,        // 4 kB per slave
  parameter N_APB_SLAVES   = 8,
  parameter APB_BASE_ADDR  = 32'h8000_0000
)(
  input  wire clk_i,
  input  wire rst_ni,

  // ---- AHB slave (from fabric) ----
  input  wire                    hsel_i,
  input  wire                    hready_i,
  input  wire [AHB_ADDR_WIDTH-1:0] haddr_i,
  input  wire                    hwrite_i,
  input  wire [2:0]              hsize_i,
  input  wire [2:0]              hburst_i,
  input  wire [3:0]              hprot_i,
  input  wire [31:0]             hwdata_i,
  output wire                    hready_o,
  output wire [1:0]              hresp_o,
  output wire [31:0]             hrdata_o,

  // ---- APB master (flattened) ----
  output wire [N_APB_SLAVES-1:0]              psel_o,
  output wire [N_APB_SLAVES*APB_ADDR_WIDTH-1:0] paddr_o,
  output wire                                 penable_o,
  output wire                                 pwrite_o,
  output wire [31:0]                          pwdata_o,
  input  wire [N_APB_SLAVES-1:0]              pready_i,
  input  wire [N_APB_SLAVES-1:0]              pslverr_i,
  input  wire [N_APB_SLAVES*32-1:0]           prdata_i
);

  // -------------------------------------------------------------------------
  // APB slave index decode (from the AHB address)
  // -------------------------------------------------------------------------
  reg [31:0]                 sel_idx_c;
  reg [N_APB_SLAVES-1:0]     slave_sel_c;
  wire [APB_ADDR_WIDTH-1:0]  paddr_c = haddr_i[APB_ADDR_WIDTH-1:0];

  always @(*) begin
    sel_idx_c = haddr_i[AHB_ADDR_WIDTH-1:12] - (APB_BASE_ADDR >> 12);
    slave_sel_c = {N_APB_SLAVES{1'b0}};
    if (sel_idx_c < N_APB_SLAVES) begin
      slave_sel_c[sel_idx_c] = 1'b1;
    end
  end

  // -------------------------------------------------------------------------
  // APB transfer state machine
  // -------------------------------------------------------------------------
  localparam [1:0] ST_IDLE   = 2'd0;   // no AHB transfer pending
  localparam [1:0] ST_SETUP  = 2'd1;   // PSEL asserted, PENABLE low
  localparam [1:0] ST_ACCESS = 2'd2;   // PSEL + PENABLE, wait for PREADY
  localparam [1:0] ST_DONE   = 2'd3;   // data captured, hready_o = 1

  reg [1:0]                  state_q;
  reg [N_APB_SLAVES-1:0]     psel_q;
  reg                        pwrite_q;
  reg [APB_ADDR_WIDTH-1:0]   paddr_q;
  reg [31:0]                 pwdata_q;
  reg [31:0]                 prdata_q;
  reg                        pslverr_q;

  // pready/pslverr/prdata of the selected APB slave
  reg              pready_sel_c;
  reg              pslverr_sel_c;
  reg [31:0]       prdata_sel_c;
  integer s;

  always @(*) begin
    pready_sel_c  = 1'b1;
    pslverr_sel_c = 1'b0;
    prdata_sel_c  = 32'h0;
    for (s = 0; s < N_APB_SLAVES; s = s + 1) begin
      if (psel_q[s]) begin
        pready_sel_c  = pready_i[s];
        pslverr_sel_c = pslverr_i[s];
        prdata_sel_c  = prdata_i[s*32 +: 32];
      end
    end
  end

  reg [1:0] next_state;
  always @(*) begin
    case (state_q)
      ST_IDLE:   next_state = (hsel_i && hready_i) ? ST_SETUP : ST_IDLE;
      ST_SETUP:  next_state = ST_ACCESS;
      ST_ACCESS: next_state = pready_sel_c ? ST_DONE : ST_ACCESS;
      // Accept a following AHB address while completing the current APB
      // response; otherwise the fabric can select the next transfer before
      // the bridge returns to IDLE, creating a circular HREADY stall.
      ST_DONE:   next_state = (hsel_i && hready_i) ? ST_SETUP : ST_IDLE;
      default:   next_state = ST_IDLE;
    endcase
  end

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q   <= ST_IDLE;
      psel_q    <= {N_APB_SLAVES{1'b0}};
      pwrite_q  <= 1'b0;
      paddr_q   <= {APB_ADDR_WIDTH{1'b0}};
      pwdata_q  <= 32'h0;
      prdata_q  <= 32'h0;
      pslverr_q <= 1'b0;
    end else begin
      state_q <= next_state;

      // Capture a newly accepted AHB address from IDLE or on the final
      // response cycle. This supports back-to-back CPU/DMA APB transfers.
      if (hsel_i && hready_i && ((state_q == ST_IDLE) || (state_q == ST_DONE))) begin
        psel_q   <= slave_sel_c;
        pwrite_q <= hwrite_i;
        paddr_q  <= paddr_c;
        pwdata_q <= hwdata_i;
      end

      if (state_q == ST_ACCESS) begin
        if (pready_sel_c) begin
          prdata_q  <= prdata_sel_c;
          pslverr_q <= pslverr_sel_c;
        end
      end

      if ((state_q == ST_DONE) && !(hsel_i && hready_i)) begin
          psel_q <= {N_APB_SLAVES{1'b0}};
      end
    end
  end

  // -------------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------------
  // APB PSEL is asserted in SETUP and ACCESS only; PENABLE is asserted in
  // ACCESS only.  Dropping both as soon as ACCESS completes prevents APB
  // slaves (especially write registers) from seeing a duplicate transfer
  // during the AHB response/DONE cycle.
  assign psel_o    = ((state_q == ST_SETUP) || (state_q == ST_ACCESS)) ?
                     psel_q : {N_APB_SLAVES{1'b0}};
  assign paddr_o   = {N_APB_SLAVES{paddr_q}};
  assign penable_o = (state_q == ST_ACCESS);
  assign pwrite_o  = pwrite_q;
  assign pwdata_o  = pwdata_q;

  assign hready_o = (state_q == ST_DONE);
  assign hresp_o  = (state_q == ST_DONE) ? {1'b0, pslverr_q} : `HRESP_OKAY;
  assign hrdata_o = prdata_q;

endmodule
