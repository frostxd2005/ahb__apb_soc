`timescale 1ns/1ps

//=============================================================================
// boot_ctrl.v
//-----------------------------------------------------------------------------
// Boot controller: boot-mode selection + flash-to-SRAM boot-copy engine.
//
// Boot modes (boot_mode_i pins, sampled while reset is asserted):
//   2'b00  SRAM boot    : core released immediately, boot address 0x0000_0000
//   2'b01  flash XIP    : core released immediately, boot address FLASH_BASE
//   2'b10  boot-copy    : core held in reset while the engine copies
//                         COPY_WORDS words from FLASH_BASE to 0x0000_0000,
//                         then the core is released at 0x0000_0000
//   2'b11  reserved     : treated like SRAM boot
//
// The copy engine is an AHB master on the memory fabric (master port 3) and
// follows the project's master convention (HBUSREQ -> HGRANT -> one-cycle
// NONSEQ -> IDLE, write data valid on the address-acceptance cycle, shared
// HREADY watched in the data phase) -- the same contract as dma_ahb_master.
//
// APB register map (slot 9, offsets within the 4 kB window):
//   0x00 MODE       RO  sampled boot_mode_i
//   0x04 STAT       RO  bit0 busy, bit1 done, bit2 copy error
//   0x08 PTR        RO  current copy word index (debug)
//   0x0C COPY_WORDS RO  configured copy size in words
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================
`include "ahb_defines.v"

module boot_ctrl #(
  parameter [31:0] FLASH_BASE = 32'h0002_0000,
  parameter [31:0] SRAM_BASE  = 32'h0000_0000,
  parameter        COPY_WORDS = 4096          // 16 kB
)(
  input  wire         clk_i,
  input  wire         rst_ni,

  input  wire [1:0]   boot_mode_i,            // sampled during reset
  output wire [31:0]  boot_addr_o,
  output wire         boot_done_o,
  output wire         core_rst_no,            // gated reset for core+caches

  // ---- APB slave (slot 9) ----
  input  wire         psel_i,
  input  wire         penable_i,
  input  wire         pwrite_i,
  input  wire [11:0]  paddr_i,
  input  wire [31:0]  pwdata_i,
  output reg  [31:0]  prdata_o,
  output wire         pready_o,
  output wire         pslverr_o,

  // ---- AHB master (memory fabric port 3) ----
  output wire         hbusreq_o,
  input  wire         hgrant_i,
  output wire [31:0]  haddr_o,
  output wire [1:0]   htrans_o,
  output wire         hwrite_o,
  output wire [2:0]   hsize_o,
  output wire [2:0]   hburst_o,
  output wire [31:0]  hwdata_o,
  input  wire         hready_i,
  input  wire [31:0]  hrdata_i,
  input  wire [1:0]   hresp_i
);

  localparam CW = `log2(COPY_WORDS);   // width of the word counter

  // -------------------------------------------------------------------------
  // Mode sampling + status
  // -------------------------------------------------------------------------
  reg [1:0]  mode_q;
  reg        done_q;
  reg        err_q;
  reg [CW-1:0] cnt_q;
  reg [31:0] rdata_cap_q;

  always @(posedge clk_i) begin
    if (!rst_ni) mode_q <= boot_mode_i;   // pins stable while reset asserted
  end

  wire copy_needed = (mode_q == 2'b10);

  // -------------------------------------------------------------------------
  // Copy engine FSM
  // -------------------------------------------------------------------------
  localparam [2:0] C_IDLE  = 3'd0;
  localparam [2:0] C_RREQ  = 3'd1;   // request bus for the flash read
  localparam [2:0] C_RADDR = 3'd2;   // NONSEQ read, one cycle
  localparam [2:0] C_RDATA = 3'd3;   // wait shared HREADY, capture
  localparam [2:0] C_WREQ  = 3'd4;   // request bus for the SRAM write
  localparam [2:0] C_WADDR = 3'd5;   // NONSEQ write, one cycle
  localparam [2:0] C_WDATA = 3'd6;   // wait shared HREADY
  localparam [2:0] C_DONE  = 3'd7;

  reg [2:0] state_q;

  wire last_word = (cnt_q == (COPY_WORDS - 1));

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q     <= C_IDLE;
      done_q      <= 1'b0;
      err_q       <= 1'b0;
      cnt_q       <= {CW{1'b0}};
      rdata_cap_q <= 32'h0;
    end else begin
      case (state_q)
        C_IDLE: begin
          if (!copy_needed) begin
            done_q  <= 1'b1;             // nothing to copy: boot is complete
            state_q <= C_DONE;
          end else begin
            state_q <= C_RREQ;
          end
        end
        C_RREQ:  if (hgrant_i)          state_q <= C_RADDR;
        // address phase held until HREADY=1 accepts it (AHB stretch)
        C_RADDR: if (hready_i)          state_q <= C_RDATA;
        C_RDATA: begin
          if (hready_i) begin
            rdata_cap_q <= hrdata_i;
            if (hresp_i != `HRESP_OKAY) err_q <= 1'b1;
            state_q <= C_WREQ;
          end
        end
        C_WREQ:  if (hgrant_i)          state_q <= C_WADDR;
        C_WADDR: if (hready_i)          state_q <= C_WDATA;
        C_WDATA: begin
          if (hready_i) begin
            if (hresp_i != `HRESP_OKAY) err_q <= 1'b1;
            if (last_word) begin
              done_q  <= 1'b1;
              state_q <= C_DONE;
            end else begin
              cnt_q   <= cnt_q + 1'b1;
              state_q <= C_RREQ;
            end
          end
        end
        C_DONE:  ;                       // terminal
        default: state_q <= C_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // AHB master outputs (convention: address+write-data valid in the one-cycle
  // NONSEQ state; hbusreq held for the whole beat)
  // -------------------------------------------------------------------------
  // address phases only (see dma8_ahb_master for the contract)
  assign hbusreq_o = (state_q == C_RREQ) || (state_q == C_RADDR) ||
                     (state_q == C_WREQ) || (state_q == C_WADDR);
  assign htrans_o  = ((state_q == C_RADDR) || (state_q == C_WADDR)) ?
                     2'b10 : 2'b00;                     // NONSEQ / IDLE
  assign haddr_o   = (state_q == C_RADDR) ? (FLASH_BASE + {18'h0, cnt_q, 2'b00}) :
                     (state_q == C_WADDR) ? (SRAM_BASE  + {18'h0, cnt_q, 2'b00}) : 32'h0;
  assign hwrite_o  = (state_q == C_WADDR);
  assign hsize_o   = 3'b010;                            // word
  assign hburst_o  = 3'b000;                            // single
  assign hwdata_o  = rdata_cap_q;

  // -------------------------------------------------------------------------
  // Boot address / core reset gating
  // -------------------------------------------------------------------------
  assign boot_addr_o = (mode_q == 2'b01) ? FLASH_BASE : SRAM_BASE;
  assign boot_done_o = done_q;
  assign core_rst_no = rst_ni && (!copy_needed || done_q);

  // -------------------------------------------------------------------------
  // APB slave (read-only registers)
  // -------------------------------------------------------------------------
  always @(*) begin
    prdata_o = 32'h0;
    if (psel_i && penable_i && !pwrite_i) begin
      case (paddr_i[3:2])
        2'b00:   prdata_o = {30'h0, mode_q};
        2'b01:   prdata_o = {29'h0, err_q, done_q, copy_needed && !done_q};
        2'b10:   prdata_o = {{(32-CW){1'b0}}, cnt_q};
        2'b11:   prdata_o = COPY_WORDS;
      endcase
    end
  end

  assign pready_o  = 1'b1;
  assign pslverr_o = 1'b0;

  // pwdata_i is unused: all registers are read-only.

endmodule
