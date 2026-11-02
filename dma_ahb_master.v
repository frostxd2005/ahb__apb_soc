`timescale 1ns/1ps

//=============================================================================
// dma_ahb_master.v
//------------------------------------------------------------------------------
// AHB-Lite execution master of the 4-channel DMA controller. Adapted from
// varun23-2004/-4-Channel-AHB-APB-DMA-Controller-RTL-Design- (dma_ahb_master.v,
// module originally named dma_ahb_mas) so that it obeys the same fabric
// convention as obi_to_ahb in this project:
//
//   * HBUSREQ is asserted while the engine has pending bus work; the fabric
//     answers on HGRANT (combinational one-hot grant).
//   * the address phase (HTRANS = NONSEQ) is presented for exactly ONE cycle,
//     starting on the first cycle HGRANT is seen (the fabric guarantees
//     HREADY == 1 on that cycle); afterwards HTRANS returns to IDLE.
//   * the data phase then waits for the shared HREADY; write data (HWDATA)
//     is driven from the FIFO head combinationally so it is already valid on
//     the address-acceptance cycle (this project's AHB slaves sample HWDATA
//     when hsel && hready, i.e. at acceptance).
//   * the bus is re-requested between the read and the write phase of every
//     beat, so the CPU is never locked out during long copies.
//
// Fixes over the original repository file:
//   * the channel configurator copied src into next_dest and read an
//     undeclared signal dest_flat for next_count (compile error + the
//     destination was never used). Now: next_dest = dest_addr_flat,
//     next_count = count_flat.
//   * HWDATA was registered one cycle too late for acceptance-sampling
//     slaves; it is now the FIFO head (combinational) during write phases.
//   * the FSM watched the (shared, multi-master) HREADY before being granted
//     the bus, which corrupts transfers on a multi-master fabric; it now
//     gates on HGRANT exactly like obi_to_ahb gates on hgrant_i.
//   * mid-transfer channel switches reload the configuration instead of
//     continuing with stale addresses.
//
// Plain Verilog (IEEE 1364-2001). Word (32-bit) transfers only, addresses
// word-aligned, HBURST fixed to SINGLE.
//=============================================================================

module dma_ahb_master (
  input  wire         clk,
  input  wire         rstn,

  // from the channel arbiter (one-hot)
  input  wire [3:0]   grant,

  // channel configuration (from the APB slave), 4 x 32-bit packed
  input  wire [127:0] src_addr_flat,
  input  wire [127:0] dest_addr_flat,
  input  wire [127:0] count_flat,
  output reg  [3:0]   transfer_done,

  // internal buffer FIFO
  input  wire         fifo_full,
  input  wire         fifo_empty,
  input  wire [31:0]  fifo_rdata,
  output reg          fifo_w_en,
  output reg          fifo_r_en,
  output wire [31:0]  fifo_wdata,

  // AHB-Lite master (fabric convention)
  input  wire         HGRANT,
  output wire         HBUSREQ,
  input  wire         HREADY,
  input  wire [31:0]  HRDATA,
  output wire [31:0]  HWDATA,
  output wire [2:0]   HSIZE,
  output reg  [31:0]  HADDR,
  output reg  [1:0]   HTRANS,
  output wire [2:0]   HBURST,
  output reg          HWRITE
);

  // fixed constraints: 32-bit words, SINGLE bursts
  assign HSIZE  = 3'b010;
  assign HBURST = 3'b000;

  // read data flows straight into the buffer; write data is the FIFO head,
  // valid on the write address-acceptance cycle
  assign fifo_wdata = HRDATA;
  assign HWDATA     = fifo_rdata;

  localparam [2:0] S_IDLE      = 3'd0;
  localparam [2:0] S_REQ_R     = 3'd1; // request fabric for the read phase
  localparam [2:0] S_READ_ADDR = 3'd2; // NONSEQ, one cycle
  localparam [2:0] S_READ_DATA = 3'd3; // IDLE, wait HREADY, capture to FIFO
  localparam [2:0] S_REQ_W     = 3'd4; // request fabric for the write phase
  localparam [2:0] S_WRITE_ADDR= 3'd5; // NONSEQ, one cycle, HWDATA live
  localparam [2:0] S_WRITE_DATA= 3'd6; // IDLE, wait HREADY
  localparam [2:0] S_CHECK     = 3'd7;

  reg [2:0]  state_q;
  reg [1:0]  active_ch_q;
  reg [31:0] current_src_q;
  reg [31:0] current_dest_q;
  reg [31:0] current_count_q;

  // request the fabric whenever the engine is active
  assign HBUSREQ = (state_q != S_IDLE);

  // ---- channel configurator (one-hot grant selects the lane) ----
  reg [31:0] next_src, next_dest, next_count;
  reg [1:0]  next_ch;

  always @(*) begin
    next_src  = 32'd0;
    next_dest = 32'd0;
    next_count= 32'd0;
    next_ch   = 2'd0;
    case (grant)
      4'b0001: begin next_src = src_addr_flat[31:0];   next_dest = dest_addr_flat[31:0];   next_count = count_flat[31:0];   next_ch = 2'd0; end
      4'b0010: begin next_src = src_addr_flat[63:32];  next_dest = dest_addr_flat[63:32];  next_count = count_flat[63:32];  next_ch = 2'd1; end
      4'b0100: begin next_src = src_addr_flat[95:64];  next_dest = dest_addr_flat[95:64];  next_count = count_flat[95:64];  next_ch = 2'd2; end
      4'b1000: begin next_src = src_addr_flat[127:96]; next_dest = dest_addr_flat[127:96]; next_count = count_flat[127:96]; next_ch = 2'd3; end
      default: begin next_src = 32'd0; next_dest = 32'd0; next_count = 32'd0; next_ch = 2'd0; end
    endcase
  end

  wire [3:0] active_onehot_c = 4'b0001 << active_ch_q;

  // ---- main FSM ----
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state_q        <= S_IDLE;
      active_ch_q    <= 2'd0;
      current_src_q  <= 32'd0;
      current_dest_q <= 32'd0;
      current_count_q<= 32'd0;
      HADDR          <= 32'd0;
      HWRITE         <= 1'b0;
      HTRANS         <= 2'b00;
      fifo_w_en      <= 1'b0;
      fifo_r_en      <= 1'b0;
      transfer_done  <= 4'b0000;
    end else begin
      // default: IDLE on the bus, no FIFO motion, no done pulse
      HTRANS    <= 2'b00;
      fifo_w_en <= 1'b0;
      fifo_r_en <= 1'b0;
      transfer_done <= 4'b0000;

      case (state_q)
        S_IDLE: begin
          if (grant != 4'b0000) begin
            current_src_q   <= next_src;
            current_dest_q  <= next_dest;
            current_count_q <= next_count;
            active_ch_q     <= next_ch;
            state_q         <= S_REQ_R;
          end
        end

        S_REQ_R: begin
          if (HGRANT) state_q <= S_READ_ADDR;
        end

        S_READ_ADDR: begin
          if (!fifo_full) begin
            HADDR  <= current_src_q;
            HWRITE <= 1'b0;
            HTRANS <= 2'b10;          // NONSEQ for exactly one cycle
            state_q <= S_READ_DATA;
          end
        end

        S_READ_DATA: begin
          if (HREADY) begin
            fifo_w_en     <= 1'b1;    // buffer the read beat
            current_src_q <= current_src_q + 32'd4;
            state_q       <= S_REQ_W;
          end
        end

        S_REQ_W: begin
          if (HGRANT && !fifo_empty) state_q <= S_WRITE_ADDR;
        end

        S_WRITE_ADDR: begin
          HADDR   <= current_dest_q;
          HWRITE  <= 1'b1;
          HTRANS  <= 2'b10;           // HWDATA = FIFO head, live now
          fifo_r_en <= !fifo_empty;   // pop on the acceptance edge
          state_q <= S_WRITE_DATA;
        end

        S_WRITE_DATA: begin
          if (HREADY) begin
            current_dest_q  <= current_dest_q + 32'd4;
            current_count_q <= current_count_q - 32'd1;
            state_q         <= S_CHECK;
          end
        end

        S_CHECK: begin
          if (current_count_q == 32'd0) begin
            transfer_done[active_ch_q] <= 1'b1;
            state_q <= S_IDLE;
          end else if (grant != active_onehot_c) begin
            state_q <= S_IDLE;        // arbiter moved on: reload config
          end else begin
            state_q <= S_REQ_R;       // next beat of the same channel
          end
        end

        default: state_q <= S_IDLE;
      endcase
    end
  end

endmodule
