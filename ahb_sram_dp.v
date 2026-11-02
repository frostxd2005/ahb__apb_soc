`timescale 1ns/1ps

//=============================================================================
// ahb_sram_dp.v
//-----------------------------------------------------------------------------
// Dual-port AHB-Lite SRAM, 96 kB, plain Verilog (IEEE 1364-2001).
//
//   Port A: instruction side (read-only; writes are accepted and ignored),
//           connected to the instruction-bus mux.  1 wait state.
//   Port B: data side (read/write), connected to the memory fabric.
//           1 wait state.  Byte-enabled writes via HSIZE/HADDR[1:0].
//
// Both ports share one reg array; only port B ever writes it, so simulation
// semantics are deterministic (port A reads the pre-clock value on a
// same-address collision, like a simple dual-port BRAM).
//
// Timing convention identical to ahb_sram:
//   * transfer accepted when hsel_i && hready_i (fabric HREADY of the
//     previous cycle)
//   * WAIT1: read data registered / write committed
//   * WAIT2: hready_o = 1, hrdata_o valid
//
// Image loading (simulation only) tries, in order:
//   1. INIT_HEX_FILE  verbatim in xsim's working directory
//   2. INIT_HEX_ALT2  verbatim (e.g. "fw_sram.hex" dropped into xsim.dir's
//      parent by hand)
//   3. INIT_HEX_ALT   (e.g. "sim/fw_sram.hex") walking up to 9 parent dirs
//=============================================================================
`include "ahb_defines.v"

module ahb_sram_dp #(
  parameter MEM_SIZE_BYTES = 96 * 1024,
  parameter AHB_ADDR_WIDTH = 32,
  parameter INIT_HEX_FILE  = "",
  parameter INIT_HEX_ALT2  = "",
  parameter INIT_HEX_ALT   = ""
)(
  input  wire                      clk_i,
  input  wire                      rst_ni,

  // ---- port A: instruction bus (read-only) ----
  input  wire                      a_hsel_i,
  input  wire                      a_hready_i,
  input  wire [AHB_ADDR_WIDTH-1:0] a_haddr_i,
  input  wire                      a_hwrite_i,
  input  wire [2:0]                a_hsize_i,
  output wire                      a_hready_o,
  output wire [1:0]                a_hresp_o,
  output wire [31:0]               a_hrdata_o,

  // ---- port B: memory fabric (read/write) ----
  input  wire                      b_hsel_i,
  input  wire                      b_hready_i,
  input  wire [AHB_ADDR_WIDTH-1:0] b_haddr_i,
  input  wire                      b_hwrite_i,
  input  wire [2:0]                b_hsize_i,
  input  wire [31:0]               b_hwdata_i,
  output wire                      b_hready_o,
  output wire [1:0]                b_hresp_o,
  output wire [31:0]               b_hrdata_o
);

  localparam DEPTH = MEM_SIZE_BYTES / 4;
  localparam AW    = `log2(DEPTH);

  reg [31:0] mem [0:DEPTH-1];

  // -------------------------------------------------------------------------
  // Image load (simulation only) -- see header for the search order.
  // -------------------------------------------------------------------------
  initial begin : mem_init
    integer fd;
    integer i;
    reg [8*512-1:0] cand;
    reg [8*512-1:0] used;
    fd   = 0;
    cand = "";
    used = "";
    if (INIT_HEX_FILE != "") begin
      fd = $fopen(INIT_HEX_FILE, "r");
      if (fd != 0) begin
        $fclose(fd);
        used = INIT_HEX_FILE;
        $readmemh(INIT_HEX_FILE, mem);
      end
    end
    if (used == "" && INIT_HEX_ALT2 != "") begin
      fd = $fopen(INIT_HEX_ALT2, "r");
      if (fd != 0) begin
        $fclose(fd);
        used = INIT_HEX_ALT2;
        $readmemh(INIT_HEX_ALT2, mem);
      end
    end
    if (used == "" && INIT_HEX_ALT != "") begin
      // Walk up to 9 levels.  Success = mem[0] leaves X (computed names
      // cannot go through $fopen under strict 1364-2005 lint; our images
      // always define word 0).  Failed attempts print a benign warning.
      cand = INIT_HEX_ALT;
      for (i = 0; i < 9 && used == ""; i = i + 1) begin
        $readmemh(cand, mem);
        if (mem[0] !== 32'hxxxxxxxx) begin
          used = cand;
        end else begin
          cand = {"../", cand};
        end
      end
    end
    if (used != "") begin
      $display("[%0t] %m: memory image '%0s' loaded", $time, used);
    end else if (INIT_HEX_FILE != "" || INIT_HEX_ALT2 != "" || INIT_HEX_ALT != "") begin
      $display("[%0t] %m: WARNING: no memory image found ('%0s' / '%0s' / '%0s' + 9 parent dirs)",
               $time, INIT_HEX_FILE, INIT_HEX_ALT2, INIT_HEX_ALT);
    end
  end

  // -------------------------------------------------------------------------
  // Port A: instruction side (read-only), 1 wait state
  // -------------------------------------------------------------------------
  localparam [1:0] A_IDLE  = 2'd0;
  localparam [1:0] A_WAIT1 = 2'd1;
  localparam [1:0] A_WAIT2 = 2'd2;
  localparam [1:0] A_ERR   = 2'd3;

  reg [1:0]    a_state_q;
  reg [AW-1:0] a_addr_q;
  reg [31:0]   a_rdata_q;

  wire        a_oob_c = (a_haddr_i[AW+1:2] >= DEPTH);

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      a_state_q <= A_IDLE;
      a_addr_q  <= {AW{1'b0}};
      a_rdata_q <= 32'h0;
    end else begin
      case (a_state_q)
        A_IDLE: begin
          if (a_hsel_i && a_hready_i) begin
            if (a_oob_c || a_hwrite_i) begin
              a_state_q <= A_ERR;        // OOB, or write to the RO port
            end else begin
              a_addr_q  <= a_haddr_i[AW+1:2];
              a_state_q <= A_WAIT1;
            end
          end
        end
        A_WAIT1: begin
          a_rdata_q <= mem[a_addr_q];
          a_state_q <= A_WAIT2;
        end
        // Accept a pipelined next address on the same edge that completes
        // this data phase.  AHB masters may keep HSEL asserted for the next
        // transfer while the current read is in its final wait cycle.
        A_WAIT2, A_ERR: begin
          if (a_hsel_i && a_hready_i) begin
            if (a_oob_c || a_hwrite_i) begin
              a_state_q <= A_ERR;
            end else begin
              a_addr_q  <= a_haddr_i[AW+1:2];
              a_state_q <= A_WAIT1;
            end
          end else begin
            a_state_q <= A_IDLE;
          end
        end
        default: a_state_q <= A_IDLE;
      endcase
    end
  end

  assign a_hready_o = (a_state_q == A_WAIT2) || (a_state_q == A_ERR);
  assign a_hresp_o  = (a_state_q == A_ERR) ? `HRESP_ERROR : `HRESP_OKAY;
  assign a_hrdata_o = a_rdata_q;

  // -------------------------------------------------------------------------
  // Port B: data side (read/write), 1 wait state, byte-enabled writes
  // -------------------------------------------------------------------------
  localparam [1:0] B_IDLE  = 2'd0;
  localparam [1:0] B_WAIT1 = 2'd1;
  localparam [1:0] B_WAIT2 = 2'd2;
  localparam [1:0] B_ERR   = 2'd3;

  reg [1:0]    b_state_q;
  reg [AW-1:0] b_addr_q;
  reg          b_write_q;
  reg [3:0]    b_be_q;
  reg [31:0]   b_wdata_q;
  reg [31:0]   b_rdata_q;

  wire [3:0]   b_be_c  = `AHB_BE_FROM_HSIZE(b_hsize_i, b_haddr_i[1:0]);
  wire         b_oob_c = (b_haddr_i[AW+1:2] >= DEPTH);

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      b_state_q <= B_IDLE;
      b_addr_q  <= {AW{1'b0}};
      b_write_q <= 1'b0;
      b_be_q    <= 4'h0;
      b_wdata_q <= 32'h0;
      b_rdata_q <= 32'h0;
    end else begin
      case (b_state_q)
        B_IDLE: begin
          if (b_hsel_i && b_hready_i) begin
            if (b_oob_c) begin
              b_state_q <= B_ERR;
            end else begin
              b_addr_q  <= b_haddr_i[AW+1:2];
              b_write_q <= b_hwrite_i;
              b_be_q    <= b_be_c;
              b_wdata_q <= b_hwdata_i;
              b_state_q <= B_WAIT1;
            end
          end
        end
        B_WAIT1: begin
          if (!b_write_q) begin
            b_rdata_q <= mem[b_addr_q];
          end else begin
            if (b_be_q[0]) mem[b_addr_q][ 7: 0] <= b_wdata_q[ 7: 0];
            if (b_be_q[1]) mem[b_addr_q][15: 8] <= b_wdata_q[15: 8];
            if (b_be_q[2]) mem[b_addr_q][23:16] <= b_wdata_q[23:16];
            if (b_be_q[3]) mem[b_addr_q][31:24] <= b_wdata_q[31:24];
          end
          b_state_q <= B_WAIT2;
        end
        // Keep throughput correct when a new address is presented during
        // the final ready cycle of the previous data phase.
        B_WAIT2, B_ERR: begin
          if (b_hsel_i && b_hready_i) begin
            if (b_oob_c) begin
              b_state_q <= B_ERR;
            end else begin
              b_addr_q  <= b_haddr_i[AW+1:2];
              b_write_q <= b_hwrite_i;
              b_be_q    <= b_be_c;
              b_wdata_q <= b_hwdata_i;
              b_state_q <= B_WAIT1;
            end
          end else begin
            b_state_q <= B_IDLE;
          end
        end
        default: b_state_q <= B_IDLE;
      endcase
    end
  end

  assign b_hready_o = (b_state_q == B_WAIT2) || (b_state_q == B_ERR);
  assign b_hresp_o  = (b_state_q == B_ERR) ? `HRESP_ERROR : `HRESP_OKAY;
  assign b_hrdata_o = b_rdata_q;

endmodule
