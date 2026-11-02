`timescale 1ns/1ps

//=============================================================================
// obi_to_ahb.v
//------------------------------------------------------------------------------
// OBI (OpenHW Bus Interface, as used by CV32E40S) MASTER -> AHB-Lite MASTER
// bridge (plain Verilog edition).
//
// Turns the CPU's OBI request interface into an AHB-Lite master port that can
// be attached to the ahb_fabric master ports.  One transfer at a time.
//
// AHB master "contract" followed by this bridge (and by axi4_to_ahb.v):
//   * hbusreq_o is asserted whenever a transfer is pending and its address has
//     not yet been presented.
//   * hgrant_i is sampled at the rising edge; one cycle later the bridge
//     presents the address phase (HTRANS = NONSEQ).
//   * after presenting, the bridge waits for hready_i == 1 (data phase
//     complete) before responding on the OBI side.
//   * during wait states the bridge drives HTRANS = IDLE so the fabric can
//     release the bus and re-arbitrate.
//=============================================================================
`include "ahb_defines.v"

module obi_to_ahb #(
  parameter ADDR_WIDTH = 32,
  parameter DATA_WIDTH = 32
)(
  // ---- OBI master (connect to the CPU's OBI slave port) ----
  input  wire                    req_i,
  input  wire [ADDR_WIDTH-1:0]   addr_i,
  input  wire                    we_i,
  input  wire [DATA_WIDTH-1:0]   wdata_i,
  input  wire [DATA_WIDTH/8-1:0] be_i,
  output reg                     gnt_o,
  output reg  [DATA_WIDTH-1:0]   rdata_o,
  output reg                     rvalid_o,
  output reg                     err_o,

  // ---- AHB-Lite master (connect to an ahb_fabric master port) ----
  output reg                     hbusreq_o,
  input  wire                    hgrant_i,
  output reg  [ADDR_WIDTH-1:0]   haddr_o,
  output reg  [1:0]              htrans_o,
  output reg                     hwrite_o,
  output reg  [2:0]              hsize_o,
  output reg  [2:0]              hburst_o,
  output reg  [3:0]              hprot_o,
  output reg  [DATA_WIDTH-1:0]   hwdata_o,
  input  wire                    hready_i,
  input  wire [DATA_WIDTH-1:0]   hrdata_i,
  input  wire [1:0]              hresp_i,

  input  wire clk_i,
  input  wire rst_ni
);

  // -------------------------------------------------------------------------
  // FSM states
  // -------------------------------------------------------------------------
  localparam [2:0] ST_IDLE    = 3'd0;   // waiting for an OBI request
  localparam [2:0] ST_PEND    = 3'd1;   // request latched, waiting for grant
  localparam [2:0] ST_PRESENT = 3'd2;   // presenting the address phase
  localparam [2:0] ST_DATA    = 3'd3;   // data phase, waiting for hready_i
  localparam [2:0] ST_DONE    = 3'd4;   // transfer complete, rvalid_o=1

  reg [2:0]               state_q;
  reg [ADDR_WIDTH-1:0]    addr_q;
  reg                     we_q;
  reg [DATA_WIDTH-1:0]    wdata_q;
  reg [DATA_WIDTH/8-1:0]  be_q;
  reg [DATA_WIDTH-1:0]    rdata_q;
  reg                     err_q;

  wire [2:0]              hsize_c;
  wire [1:0]              addr_lsb_c;
  reg [2:0]               next_state;

  // -------------------------------------------------------------------------
  // Next state
  // -------------------------------------------------------------------------
  always @(*) begin
    case (state_q)
      ST_IDLE:    next_state = req_i ? ST_PEND : ST_IDLE;
      ST_PEND:    next_state = hgrant_i ? ST_PRESENT : ST_PEND;
      // address phase: NONSEQ (combinational from the state) is held until
      // hready_i == 1 accepts it -- HREADY=0 here means another master's
      // data phase is still in flight; dropping NONSEQ early would lose the
      // transfer (the fabric only loads its data-phase select while hready).
      ST_PRESENT: next_state = hready_i ? ST_DATA : ST_PRESENT;
      ST_DATA:    next_state = hready_i ? ST_DONE : ST_DATA;
      ST_DONE:    next_state = req_i ? ST_PEND : ST_IDLE;
      default:    next_state = ST_IDLE;
    endcase
  end

  // -------------------------------------------------------------------------
  // State + latches
  // -------------------------------------------------------------------------
  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q  <= ST_IDLE;
      addr_q   <= {ADDR_WIDTH{1'b0}};
      we_q     <= 1'b0;
      wdata_q  <= {DATA_WIDTH{1'b0}};
      be_q     <= {(DATA_WIDTH/8){1'b0}};
      rdata_q  <= {DATA_WIDTH{1'b0}};
      err_q    <= 1'b0;
    end else begin
      state_q <= next_state;
      // Accept the OBI request when we leave ST_IDLE, or a new one that is
      // already pending when we finish the previous transfer.
      if ((state_q == ST_IDLE) && req_i) begin
        addr_q  <= addr_i;
        we_q    <= we_i;
        wdata_q <= wdata_i;
        be_q    <= be_i;
      end
      if ((state_q == ST_DONE) && req_i) begin
        addr_q  <= addr_i;
        we_q    <= we_i;
        wdata_q <= wdata_i;
        be_q    <= be_i;
      end
      // Capture read data / error at data-phase completion.
      if ((state_q == ST_DATA) && hready_i) begin
        rdata_q <= hrdata_i;
        err_q   <= (hresp_i != `HRESP_OKAY);
      end
    end
  end

  // -------------------------------------------------------------------------
  // AHB master outputs
  // -------------------------------------------------------------------------
  assign {hsize_c, addr_lsb_c} = `OBI_BE_TO_HSIZE(be_q);

  always @(*) begin
    hbusreq_o = (state_q == ST_PEND);
    htrans_o  = (state_q == ST_PRESENT) ? `HTRANS_NONSEQ : `HTRANS_IDLE;
    haddr_o   = {addr_q[ADDR_WIDTH-1:2], addr_lsb_c};
    hwrite_o  = we_q;
    hsize_o   = hsize_c;
    hburst_o  = `HBURST_SINGLE;
    hprot_o   = 4'b0011;               // privileged, non-cacheable
    hwdata_o  = wdata_q;
  end

  // -------------------------------------------------------------------------
  // OBI outputs
  // -------------------------------------------------------------------------
  always @(*) begin
    gnt_o    = (state_q == ST_PEND);
    rvalid_o = (state_q == ST_DONE);
    rdata_o  = rdata_q;
    err_o    = (state_q == ST_DONE) && err_q;
  end

endmodule
