`timescale 1ns/1ps

//=============================================================================
// apb_wdg.v
//-----------------------------------------------------------------------------
// Watchdog timer (APB slot 4).  Down-counter clocked by the SoC clock; on
// timeout it raises a sticky interrupt (INTC source 4 / DMA request line 5)
// and exports a reset-request level (top-level pin; the SoC does NOT feed it
// back into its own reset -- that is a board/integration decision).
//
// APB register map (offsets within the 4 kB window):
//   0x00 LOAD    RW   reload value (words of clk_i)
//   0x04 VALUE   RO   current counter value
//   0x08 CTRL    RW   bit0 = enable; bit1 = timeout flag (sticky, W1C by
//                     writing 1); writing bit0=1 also reloads the counter
//   0x0C KICK    W    any write reloads VALUE from LOAD and clears timeout
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module apb_wdg (
  input  wire         clk_i,
  input  wire         rst_ni,

  output wire         irq_o,            // sticky timeout interrupt
  output wire         reset_req_o,      // timeout reset request (export only)

  // ---- APB slave (slot 4) ----
  input  wire         psel_i,
  input  wire         penable_i,
  input  wire         pwrite_i,
  input  wire [11:0]  paddr_i,
  input  wire [31:0]  pwdata_i,
  output reg  [31:0]  prdata_o,
  output wire         pready_o,
  output wire         pslverr_o
);

  reg [31:0] load_q;
  reg [31:0] value_q;
  reg        en_q;
  reg        timeout_q;

  wire wr_c     = psel_i && penable_i && pwrite_i;
  wire kick_c   = wr_c && (paddr_i[3:2] == 2'b11);
  wire ctrl_c   = wr_c && (paddr_i[3:2] == 2'b10);

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      load_q    <= 32'h0000_FFFF;
      value_q   <= 32'h0000_FFFF;
      en_q      <= 1'b0;
      timeout_q <= 1'b0;
    end else begin
      if (wr_c && (paddr_i[3:2] == 2'b00)) load_q <= pwdata_i;

      if (ctrl_c) begin
        en_q <= pwdata_i[0];
        if (pwdata_i[0]) value_q <= load_q;        // enable also reloads
        if (pwdata_i[1]) timeout_q <= 1'b0;        // W1C timeout flag
      end

      if (kick_c) begin
        value_q   <= load_q;
        timeout_q <= 1'b0;
      end

      if (en_q && !kick_c && !(ctrl_c && pwdata_i[0])) begin
        if (value_q != 32'h0) begin
          value_q <= value_q - 32'd1;
        end else begin
          timeout_q <= 1'b1;                       // sticky until kick/W1C
        end
      end
    end
  end

  assign irq_o       = timeout_q;
  assign reset_req_o = timeout_q;

  // ---- APB read path ----
  always @(*) begin
    prdata_o = 32'h0;
    if (psel_i && penable_i && !pwrite_i) begin
      case (paddr_i[3:2])
        2'b00:   prdata_o = load_q;
        2'b01:   prdata_o = value_q;
        2'b10:   prdata_o = {30'h0, timeout_q, en_q};
        default: prdata_o = 32'h0;                 // KICK is write-only
      endcase
    end
  end

  assign pready_o  = 1'b1;
  assign pslverr_o = 1'b0;

endmodule
