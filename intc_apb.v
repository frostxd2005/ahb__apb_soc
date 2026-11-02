`timescale 1ns/1ps

//=============================================================================
// intc_apb.v
//-----------------------------------------------------------------------------
// External interrupt controller for CV32E40P (APB slot 8).
//
// The E40P has no CLIC, so interrupt handling is split: this controller
// aggregates the peripheral sources and drives the core's legacy PULP
// interrupt interface (level irq_i[31:0], irq_ack_o + irq_id_o[4:0] back);
// the core vectors through the software trap table at mtvec, and the ISR
// reads the CLAIM register to identify the source.
//
// Features (8 sources wired, registers 32-bit wide for future expansion):
//   * per-source ENABLE
//   * per-source PENDING: set on a rising edge of the source line (works for
//     pulses like DMA-done and for levels like UART INT, which re-assert
//     with a new edge after the ISR services the peripheral), cleared by
//     writing 1 (W1C) or automatically when the core acknowledges with a
//     matching custom interrupt ID (16 + source index)
//   * per-source 4-bit PRIORITY; the winner is the highest-priority
//     pending+enabled source, lowest index breaks ties; only the winner is
//     presented to the core (one-hot irq_o), which keeps the ack handshake
//     unambiguous
//
// APB register map (offsets within the 4 kB window):
//   0x00 ENABLE    RW  bit i enables source i
//   0x04 PENDING   RO / W1C  bit i = source i pending
//   0x08 PRIORITY  RW  nibble i = priority of source i (0 = lowest, reset 0)
//   0x0C CLAIM     RO  {26'b0, valid, id[4:0]} of the current winner
//   0x10 STATUS    RO  pending & enable snapshot
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module intc_apb (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- interrupt sources ----
  input  wire [7:0]   src_i,

  // ---- core interface (E40P legacy PULP handshake) ----
  output wire [31:0]  irq_o,
  input  wire         irq_ack_i,
  input  wire [4:0]   irq_id_i,

  // ---- APB slave (slot 8) ----
  input  wire         psel_i,
  input  wire         penable_i,
  input  wire         pwrite_i,
  input  wire [11:0]  paddr_i,
  input  wire [31:0]  pwdata_i,
  output reg  [31:0]  prdata_o,
  output wire         pready_o,
  output wire         pslverr_o
);

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  reg [7:0]  enable_q;
  reg [7:0]  pending_q;
  reg [31:0] priority_q;
  reg [7:0]  src_q;              // for rising-edge detect
  reg [2:0]  last_claim_id_q;
  reg        last_claim_valid_q;

  wire [7:0] edge_c  = src_i & ~src_q;
  wire [7:0] pend_en = pending_q & enable_q;

  // clear sources: W1C write to PENDING, or core ack with matching id
  reg [7:0] clr_c;
  always @(*) begin
    clr_c = 8'h00;
    if (psel_i && penable_i && pwrite_i && (paddr_i[3:2] == 2'b01))
      clr_c = pwdata_i[7:0];
    // E40P custom interrupt IDs 16..23 map back to INTC sources 0..7.
    if (irq_ack_i && (irq_id_i[4:3] == 2'b10))
      clr_c[irq_id_i[2:0]] = 1'b1;
  end

  integer k;
  always @(posedge clk_i) begin
    if (!rst_ni) begin
      enable_q   <= 8'h00;
      pending_q  <= 8'h00;
      priority_q <= 32'h0;
      src_q              <= 8'h00;
      last_claim_id_q    <= 3'd0;
      last_claim_valid_q <= 1'b0;
    end else begin
      src_q <= src_i;
      if (irq_ack_i && (irq_id_i[4:3] == 2'b10)) begin
        last_claim_id_q    <= irq_id_i[2:0];
        last_claim_valid_q <= 1'b1;
      end else if (psel_i && penable_i && !pwrite_i &&
                   (paddr_i[7:2] == 6'h03)) begin
        last_claim_valid_q <= 1'b0;
      end
      for (k = 0; k < 8; k = k + 1) begin
        if (edge_c[k])              pending_q[k] <= 1'b1;   // set wins over
        else if (clr_c[k])          pending_q[k] <= 1'b0;   // clear
      end
      if (psel_i && penable_i && pwrite_i) begin
        case (paddr_i[3:2])
          2'b00:   enable_q   <= pwdata_i[7:0];
          2'b10:   priority_q <= pwdata_i;
          default: ;                       // PENDING is W1C (handled above)
        endcase
      end
    end
  end

  // -------------------------------------------------------------------------
  // Winner selection: highest priority nibble, lowest index on ties
  // -------------------------------------------------------------------------
  reg        win_valid_c;
  reg [2:0]  win_id_c;
  reg [3:0]  win_prio_c;
  integer    j;

  always @(*) begin
    win_valid_c = 1'b0;
    win_id_c    = 3'd0;
    win_prio_c  = 4'd0;
    for (j = 0; j < 8; j = j + 1) begin
      if (pend_en[j] && (!win_valid_c ||
                         (priority_q[j*4 +: 4] > win_prio_c))) begin
        win_valid_c = 1'b1;
        win_id_c    = j[2:0];
        win_prio_c  = priority_q[j*4 +: 4];
      end
    end
  end

  // One-hot presentation on the E40P custom interrupt range.  The E40P
  // masks standard/user IRQ IDs 0..10 by default; using 16..23 lets every
  // peripheral source reach the core without modifying vendor-core files.
  assign irq_o = win_valid_c ? (32'h0001_0000 << win_id_c) : 32'h0;

  // -------------------------------------------------------------------------
  // APB read path
  // -------------------------------------------------------------------------
  always @(*) begin
    prdata_o = 32'h0;
    if (psel_i && penable_i && !pwrite_i) begin
      case (paddr_i[7:2])
        6'h00:   prdata_o = {24'h0, enable_q};
        6'h01:   prdata_o = {24'h0, pending_q};
        6'h02:   prdata_o = priority_q;
        6'h03: begin
          if (last_claim_valid_q)
            prdata_o = {26'h0, 1'b1, 2'b00, last_claim_id_q};
          else
            prdata_o = {26'h0, win_valid_c, 2'b00, win_id_c};
        end
        6'h04:   prdata_o = {24'h0, pend_en};
        default: prdata_o = 32'h0;
      endcase
    end
  end

  assign pready_o  = 1'b1;
  assign pslverr_o = 1'b0;

endmodule
