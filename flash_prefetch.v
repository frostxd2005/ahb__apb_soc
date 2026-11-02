`timescale 1ns/1ps

//=============================================================================
// flash_prefetch.v
//-----------------------------------------------------------------------------
// Flash prefetch buffer for the XIP instruction path.  Sits between the
// instruction-bus mux (slave 1) and flash port A.
//
//   * CPU-side: AHB slave with the project's response convention
//     (hready_o high only when data is valid; single-cycle ERROR for writes).
//   * Flash-side: drives the ahb_flash_dp port A slave signals directly.
//   * 4-entry FIFO of {address, data}.  On a CPU fetch MISS the FIFO is
//     flushed, the word is fetched from flash, and sequential prefetching
//     (addr+4, +8, ...) refills the FIFO while the I-bus is idle.  A
//     subsequent sequential fetch HITTING in the FIFO is served immediately
//     (2 cycles instead of the miss path's ~7).
//   * correctness invariants:
//       - a FIFO entry is only ever returned for an exact word-address match
//         and flash content never changes at runtime -> hits cannot be stale;
//       - any miss flushes the FIFO;
//       - the engine latches its own target address (e_addr_q) at launch, so
//         a miss reprogramming pf_addr_q mid-beat can never mistag an entry;
//       - engine launch is blocked on the very cycle a miss is accepted, and
//         the engine's pf_addr increment is suppressed if a flush happened
//         during its beat.
//   * prefetching only runs inside the flash window
//     [0x0002_0000, 0x000A_0000) and stops at the window end.
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================
`include "ahb_defines.v"

module flash_prefetch (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- CPU side (AHB slave, from ahb_ibus_mux slave 1) ----
  input  wire         hsel_i,
  input  wire         hready_i,        // mux-level HREADY (previous cycle)
  input  wire [31:0]  haddr_i,
  input  wire         hwrite_i,
  output wire         hready_o,
  output wire [1:0]   hresp_o,
  output wire [31:0]  hrdata_o,

  // ---- flash side (drives ahb_flash_dp port A) ----
  output wire         pa_hsel_o,
  output wire [31:0]  pa_haddr_o,
  output wire         pa_hwrite_o,
  output wire         pa_hready_o,     // port A accepts when hsel && hready
  input  wire         pa_hready_i,     // port A response strobe (DONE cycle)
  input  wire [1:0]   pa_hresp_i,
  input  wire [31:0]  pa_hrdata_i
);

  // -------------------------------------------------------------------------
  // FIFO storage
  // -------------------------------------------------------------------------
  reg [31:0] f_addr  [0:3];
  reg [31:0] f_data  [0:3];
  reg [3:0]  f_valid;
  reg [1:0]  wr_ptr_q;

  // -------------------------------------------------------------------------
  // FSMs
  // -------------------------------------------------------------------------
  localparam [2:0] P_IDLE = 3'd0;
  localparam [2:0] P_MISS = 3'd1;   // wait for the flash port to be free
  localparam [2:0] P_FWD  = 3'd2;   // drive port A address phase
  localparam [2:0] P_WAIT = 3'd3;   // wait for port A response
  localparam [2:0] P_RESP = 3'd4;   // hready_o = 1, data valid
  localparam [2:0] P_WERR = 3'd5;   // write attempt -> single-cycle ERROR

  localparam [1:0] E_IDLE = 2'd0;
  localparam [1:0] E_FWD  = 2'd1;
  localparam [1:0] E_WAIT = 2'd2;

  reg [2:0]  p_state_q;
  reg [1:0]  e_state_q;

  reg [31:0] addr_q;        // CPU word being served
  reg [31:0] data_q;        // captured response data
  reg [1:0]  resp_q;        // captured response
  reg [31:0] pf_addr_q;     // next sequential prefetch address
  reg [31:0] e_addr_q;      // address the engine is actually fetching
  reg        pf_active_q;   // prefetch armed (after a miss)
  reg        flush_q;       // a miss flushed the FIFO during the engine beat

  // ---- FIFO hit lookup on the incoming address phase ----
  reg        hit_c;
  reg [1:0]  hit_idx_c;
  reg [31:0] hit_data_c;
  integer    i;

  always @(*) begin
    hit_c      = 1'b0;
    hit_idx_c  = 2'd0;
    hit_data_c = 32'h0;
    for (i = 0; i < 4; i = i + 1) begin
      if (f_valid[i] && (f_addr[i] == haddr_i)) begin
        hit_c      = 1'b1;
        hit_idx_c  = i[1:0];
        hit_data_c = f_data[i];
      end
    end
  end

  wire accept_c  = hsel_i && hready_i && (p_state_q == P_IDLE);
  wire miss_c    = accept_c && !hwrite_i && !hit_c;   // flush + reprogram
  wire pf_in_win = (pf_addr_q >= 32'h0002_0000) && (pf_addr_q < 32'h000A_0000);
  wire fifo_full = (f_valid == 4'hF);

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      p_state_q   <= P_IDLE;
      e_state_q   <= E_IDLE;
      addr_q      <= 32'h0;
      data_q      <= 32'h0;
      resp_q      <= `HRESP_OKAY;
      pf_addr_q   <= 32'h0002_0000;
      e_addr_q    <= 32'h0002_0000;
      pf_active_q <= 1'b0;
      flush_q     <= 1'b0;
      f_valid     <= 4'h0;
      wr_ptr_q    <= 2'd0;
    end else begin
      // -----------------------------------------------------------------
      // CPU side
      // -----------------------------------------------------------------
      case (p_state_q)
        P_IDLE: begin
          if (accept_c) begin
            addr_q <= haddr_i;
            if (hwrite_i) begin
              p_state_q <= P_WERR;
            end else if (hit_c) begin
              data_q             <= hit_data_c;
              f_valid[hit_idx_c] <= 1'b0;      // consume the entry
              resp_q             <= `HRESP_OKAY;
              p_state_q          <= P_RESP;
            end else begin
              f_valid     <= 4'h0;             // flush: non-sequential fetch
              wr_ptr_q    <= 2'd0;
              pf_addr_q   <= haddr_i + 32'd4;
              pf_active_q <= 1'b1;
              if (e_state_q != E_IDLE) flush_q <= 1'b1;  // engine mid-beat
              p_state_q   <= P_MISS;
            end
          end
        end
        P_MISS: begin
          if (e_state_q == E_IDLE) p_state_q <= P_FWD;
        end
        P_FWD:  p_state_q <= P_WAIT;
        P_WAIT: begin
          if (pa_hready_i) begin
            data_q    <= pa_hrdata_i;
            resp_q    <= pa_hresp_i;
            p_state_q <= P_RESP;
          end
        end
        P_RESP: p_state_q <= P_IDLE;
        P_WERR: p_state_q <= P_IDLE;
        default: p_state_q <= P_IDLE;
      endcase

      // -----------------------------------------------------------------
      // Prefetch engine (only while the CPU side is idle and not missing)
      // -----------------------------------------------------------------
      case (e_state_q)
        E_IDLE: begin
          if (pf_active_q && !fifo_full && pf_in_win && !miss_c &&
              (p_state_q == P_IDLE)) begin
            e_addr_q  <= pf_addr_q;
            e_state_q <= E_FWD;
          end
        end
        E_FWD: e_state_q <= E_WAIT;
        E_WAIT: begin
          if (pa_hready_i) begin
            f_addr[wr_ptr_q] <= e_addr_q;
            f_data[wr_ptr_q] <= pa_hrdata_i;
            if (pa_hresp_i == `HRESP_OKAY) f_valid[wr_ptr_q] <= 1'b1;
            wr_ptr_q <= wr_ptr_q + 2'd1;
            // advance the sequential pointer only if no miss reprogrammed it
            if (!flush_q && !miss_c) pf_addr_q <= e_addr_q + 32'd4;
            flush_q   <= 1'b0;
            e_state_q <= E_IDLE;
          end
        end
        default: e_state_q <= E_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Flash port A driving (CPU forward and engine beats are mutually
  // exclusive: the CPU leaves P_MISS only when the engine is idle, and the
  // engine launches only when the CPU is in P_IDLE).
  // -------------------------------------------------------------------------
  assign pa_hsel_o   = (p_state_q == P_FWD) || (e_state_q == E_FWD);
  assign pa_haddr_o  = (p_state_q == P_FWD) ? addr_q : e_addr_q;
  assign pa_hwrite_o = 1'b0;
  assign pa_hready_o = 1'b1;   // single-cycle address pulses; port A is IDLE

  // -------------------------------------------------------------------------
  // CPU-side response
  // -------------------------------------------------------------------------
  assign hready_o = (p_state_q == P_RESP) || (p_state_q == P_WERR);
  assign hresp_o  = (p_state_q == P_WERR) ? `HRESP_ERROR :
                    (p_state_q == P_RESP) ? resp_q : `HRESP_OKAY;
  assign hrdata_o = data_q;

endmodule
