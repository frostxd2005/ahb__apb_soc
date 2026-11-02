`timescale 1ns/1ps

//=============================================================================
// ahb_sram.v
//------------------------------------------------------------------------------
// AHB-Lite single-port synchronous SRAM (block RAM friendly), 1 wait state.
// Plain Verilog edition.
//
// Timing (consistent across all slaves in this package):
//   * Transfer accepted when hsel_i && hready_i -> address/byte-enables latched
//   * WAIT1 cycle: read data registered (or write committed)
//   * WAIT2 cycle: hready_o = 1, hrdata_o valid -> master samples on the
//     following edge.  So hready_o is only ever high when data is valid.
//
// Writes are byte-enabled via HSIZE/HADDR[1:0] (ahb_defines.v macro).
// Optional $readmemh init via INIT_HEX_FILE.
//=============================================================================
`include "ahb_defines.v"

module ahb_sram #(
  parameter MEM_SIZE_BYTES = 96 * 1024,
  parameter AHB_ADDR_WIDTH = 32,
  parameter INIT_HEX_FILE  = "",
  parameter INIT_HEX_ALT   = ""      // fallback path, searched upwards (sim only)
)(
  input  wire                    clk_i,
  input  wire                    rst_ni,

  input  wire                    hsel_i,
  input  wire                    hready_i,        // fabric HREADY (previous cycle)
  input  wire [AHB_ADDR_WIDTH-1:0] haddr_i,
  input  wire                    hwrite_i,
  input  wire [2:0]              hsize_i,
  input  wire [2:0]              hburst_i,
  input  wire [3:0]              hprot_i,
  input  wire [31:0]             hwdata_i,
  output wire                    hready_o,
  output wire [1:0]              hresp_o,
  output wire [31:0]             hrdata_o
);

  localparam DEPTH = MEM_SIZE_BYTES / 4;
  localparam AW    = `log2(DEPTH);

  reg [31:0] mem [0:DEPTH-1];

  // -------------------------------------------------------------------------
  // Memory initialisation (simulation-only)
  //   1. try INIT_HEX_FILE verbatim in xsim's working directory (CLI flow:
  //      package root, after "copy sim\fw_sram.hex sram.hex" -- or no copy
  //      at all thanks to step 2).
  //   2. if that fails and INIT_HEX_ALT is set (e.g. "sim/fw_sram.hex"),
  //      retry it and walk up to 8 parent directories.  This makes the
  //      Vivado GUI flow work no matter where xsim.dir/CWD ends up
  //      (<proj>.sim/sim_1/behav/xsim, <proj>.runs/sim_1, ...).
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
    if (used == "" && INIT_HEX_ALT != "") begin
      // Walk up to 9 levels looking for the alternate image.  Success is
      // detected via mem[0] leaving X (computed names cannot be passed to
      // $fopen under strict 1364-2005 lint, and our images always define
      // word 0).  Failed attempts print a benign "cannot be opened" warning.
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
    end else if (INIT_HEX_FILE != "" || INIT_HEX_ALT != "") begin
      $display("[%0t] %m: WARNING: no memory image found ('%0s', then '%0s' walking up 9 dirs)",
               $time, INIT_HEX_FILE, INIT_HEX_ALT);
    end
  end

  // -------------------------------------------------------------------------
  // Transfer state machine
  // -------------------------------------------------------------------------
  localparam [1:0] ST_IDLE  = 2'd0;
  localparam [1:0] ST_WAIT1 = 2'd1;    // read data registered / write committed
  localparam [1:0] ST_WAIT2 = 2'd2;    // hready_o = 1, data valid
  localparam [1:0] ST_ERR   = 2'd3;    // out-of-range address -> ERROR

  reg [1:0]    state_q;
  reg [AW-1:0] addr_q;
  reg          write_q;
  reg [3:0]    be_q;
  reg [31:0]   wdata_q;
  reg [31:0]   rdata_q;
  reg          err_q;

  wire [3:0] be_c = `AHB_BE_FROM_HSIZE(hsize_i, haddr_i[1:0]);
  // 96 kB is not a power of two; the fabric maps a 128 kB window here, so
  // addresses above the real SRAM size must error out instead of aliasing.
  wire        oob_c = (haddr_i[AW+1:2] >= DEPTH);

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q <= ST_IDLE;
      addr_q  <= {AW{1'b0}};
      write_q <= 1'b0;
      be_q    <= 4'h0;
      wdata_q <= 32'h0;
      rdata_q <= 32'h0;
      err_q   <= 1'b0;
    end else begin
      case (state_q)
        ST_IDLE: begin
          if (hsel_i && hready_i) begin
            if (oob_c) begin
              err_q   <= 1'b1;
              state_q <= ST_ERR;
            end else begin
              addr_q  <= haddr_i[AW+1:2];
              write_q <= hwrite_i;
              be_q    <= be_c;
              wdata_q <= hwdata_i;
              state_q <= ST_WAIT1;
            end
          end
        end
        ST_WAIT1: begin
          if (!write_q) begin
            rdata_q <= mem[addr_q];
          end else begin
            if (be_q[0]) mem[addr_q][ 7: 0] <= wdata_q[ 7: 0];
            if (be_q[1]) mem[addr_q][15: 8] <= wdata_q[15: 8];
            if (be_q[2]) mem[addr_q][23:16] <= wdata_q[23:16];
            if (be_q[3]) mem[addr_q][31:24] <= wdata_q[31:24];
          end
          state_q <= ST_WAIT2;
        end
        ST_WAIT2: state_q <= ST_IDLE;
        ST_ERR:   state_q <= ST_IDLE;
        default:  state_q <= ST_IDLE;
      endcase
    end
  end

  assign hready_o = (state_q == ST_WAIT2) || (state_q == ST_ERR);
  assign hresp_o  = (state_q == ST_ERR) ? `HRESP_ERROR : `HRESP_OKAY;
  assign hrdata_o = rdata_q;

endmodule
