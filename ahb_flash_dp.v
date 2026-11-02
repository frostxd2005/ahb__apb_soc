`timescale 1ns/1ps

//=============================================================================
// ahb_flash_dp.v
//-----------------------------------------------------------------------------
// Dual-port AHB-Lite flash model (simulation / bring-up placeholder), 512 kB
// mapped at FLASH_BASE=0x0002_0000, plain Verilog (IEEE 1364-2001). The
// absolute AHB address is converted to an image-relative word index. Read-only:
// writes are accepted on port B and ignored (respond OKAY), rejected on port A.
//
//   Port A: instruction side, fed through flash_prefetch (2 wait states).
//   Port B: memory fabric (XIP data reads, boot-copy engine reads).
//
// Timing convention identical to ahb_flash: IDLE -> WAIT1 -> WAIT2 -> DONE,
// hready_o high only on DONE (data valid).
//
// Image loading: same three-step search as ahb_sram_dp (INIT_HEX_FILE,
// INIT_HEX_ALT2, then INIT_HEX_ALT walking up to 9 parent directories).
//=============================================================================
`include "ahb_defines.v"

module ahb_flash_dp #(
  parameter MEM_SIZE_BYTES = 512 * 1024,
  parameter [31:0] FLASH_BASE = 32'h0002_0000,
  parameter AHB_ADDR_WIDTH = 32,
  parameter INIT_HEX_FILE  = "",
  parameter INIT_HEX_ALT2  = "",
  parameter INIT_HEX_ALT   = ""
)(
  input  wire                      clk_i,
  input  wire                      rst_ni,

  // ---- port A: instruction prefetch path (read-only) ----
  input  wire                      a_hsel_i,
  input  wire                      a_hready_i,
  input  wire [AHB_ADDR_WIDTH-1:0] a_haddr_i,
  input  wire                      a_hwrite_i,
  output wire                      a_hready_o,
  output wire [1:0]                a_hresp_o,
  output wire [31:0]               a_hrdata_o,

  // ---- port B: memory fabric (read-only in this model) ----
  input  wire                      b_hsel_i,
  input  wire                      b_hready_i,
  input  wire [AHB_ADDR_WIDTH-1:0] b_haddr_i,
  input  wire                      b_hwrite_i,
  output wire                      b_hready_o,
  output wire [1:0]                b_hresp_o,
  output wire [31:0]               b_hrdata_o
);

  localparam DEPTH = MEM_SIZE_BYTES / 4;
  localparam AW    = `log2(DEPTH);

  reg [31:0] mem [0:DEPTH-1];

  // Images are stored from word 0, but the AHB map begins at FLASH_BASE.
  // Convert absolute bus addresses to offsets before indexing the array.
  wire [31:0] a_offset_c = a_haddr_i - FLASH_BASE;
  wire [31:0] b_offset_c = b_haddr_i - FLASH_BASE;
  wire        a_oob_c = (a_haddr_i < FLASH_BASE) || (a_offset_c >= MEM_SIZE_BYTES);
  wire        b_oob_c = (b_haddr_i < FLASH_BASE) || (b_offset_c >= MEM_SIZE_BYTES);

  // -------------------------------------------------------------------------
  // Image load (simulation only)
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
  // Port A: 2 wait states, read-only (writes -> ERROR)
  // -------------------------------------------------------------------------
  localparam [1:0] A_IDLE  = 2'd0;
  localparam [1:0] A_WAIT1 = 2'd1;
  localparam [1:0] A_WAIT2 = 2'd2;
  localparam [1:0] A_DONE  = 2'd3;

  reg [1:0]    a_state_q;
  reg [AW-1:0] a_addr_q;
  reg [31:0]   a_rdata_q;
  reg          a_err_q;

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      a_state_q <= A_IDLE;
      a_addr_q  <= {AW{1'b0}};
      a_rdata_q <= 32'h0;
      a_err_q   <= 1'b0;
    end else begin
      case (a_state_q)
        A_IDLE: begin
          if (a_hsel_i && a_hready_i) begin
            a_err_q   <= a_hwrite_i || a_oob_c; // read-only / outside flash
            a_addr_q  <= a_oob_c ? {AW{1'b0}} : a_offset_c[AW+1:2];
            a_state_q <= A_WAIT1;
          end
        end
        A_WAIT1: begin
          if (!a_err_q) a_rdata_q <= mem[a_addr_q];
          a_state_q <= A_WAIT2;
        end
        A_WAIT2: a_state_q <= A_DONE;
        A_DONE: begin
          if (a_hsel_i && a_hready_i) begin
            a_err_q   <= a_hwrite_i || a_oob_c;
            a_addr_q  <= a_oob_c ? {AW{1'b0}} : a_offset_c[AW+1:2];
            a_state_q <= A_WAIT1;
          end else begin
            a_state_q <= A_IDLE;
          end
        end
        default: a_state_q <= A_IDLE;
      endcase
    end
  end

  assign a_hready_o = (a_state_q == A_DONE);
  assign a_hresp_o  = a_err_q ? `HRESP_ERROR : `HRESP_OKAY;
  assign a_hrdata_o = a_rdata_q;

  // -------------------------------------------------------------------------
  // Port B: 2 wait states, reads served; writes accepted and ignored
  // -------------------------------------------------------------------------
  localparam [1:0] B_IDLE  = 2'd0;
  localparam [1:0] B_WAIT1 = 2'd1;
  localparam [1:0] B_WAIT2 = 2'd2;
  localparam [1:0] B_DONE  = 2'd3;

  reg [1:0]    b_state_q;
  reg [AW-1:0] b_addr_q;
  reg [31:0]   b_rdata_q;
  reg          b_read_q;
  reg          b_err_q;

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      b_state_q <= B_IDLE;
      b_addr_q  <= {AW{1'b0}};
      b_rdata_q <= 32'h0;
      b_read_q  <= 1'b0;
      b_err_q   <= 1'b0;
    end else begin
      case (b_state_q)
        B_IDLE: begin
          if (b_hsel_i && b_hready_i) begin
            b_addr_q  <= b_oob_c ? {AW{1'b0}} : b_offset_c[AW+1:2];
            b_read_q  <= !b_hwrite_i;
            b_err_q   <= b_oob_c;
            b_state_q <= B_WAIT1;
          end
        end
        B_WAIT1: begin
          if (b_read_q && !b_err_q) b_rdata_q <= mem[b_addr_q];
          b_state_q <= B_WAIT2;
        end
        B_WAIT2: b_state_q <= B_DONE;
        B_DONE: begin
          if (b_hsel_i && b_hready_i) begin
            b_addr_q  <= b_oob_c ? {AW{1'b0}} : b_offset_c[AW+1:2];
            b_read_q  <= !b_hwrite_i;
            b_err_q   <= b_oob_c;
            b_state_q <= B_WAIT1;
          end else begin
            b_state_q <= B_IDLE;
          end
        end
        default: b_state_q <= B_IDLE;
      endcase
    end
  end

  assign b_hready_o = (b_state_q == B_DONE);
  assign b_hresp_o  = b_err_q ? `HRESP_ERROR : `HRESP_OKAY;
  assign b_hrdata_o = b_rdata_q;

endmodule
