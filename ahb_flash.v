`timescale 1ns/1ps

//=============================================================================
// ahb_flash.v
//------------------------------------------------------------------------------
// AHB-Lite read-only flash model (simulation / bring-up placeholder), 2 wait
// states.  Same response timing convention as ahb_sram.  Plain Verilog.
//
// Intentionally simple: models the 512 kB memory map so the SoC can boot and
// run before the real flash controller exists.  Replace with the real SPI NOR
// / memory-mapped flash controller (with prefetch buffer) when available.
//
// Writes are ignored (read-only).  A minimal status register is exposed.
//=============================================================================
`include "ahb_defines.v"

module ahb_flash #(
  parameter MEM_SIZE_BYTES = 512 * 1024,
  parameter AHB_ADDR_WIDTH = 32,
  parameter INIT_HEX_FILE  = "",
  parameter INIT_HEX_ALT   = ""      // fallback path, searched upwards (sim only)
)(
  input  wire                    clk_i,
  input  wire                    rst_ni,

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

  // ---- flash controller status (placeholder) ----
  output wire [31:0]             ctrl_status_o,
  output wire                    ctrl_busy_o
);

  localparam DEPTH = MEM_SIZE_BYTES / 4;
  localparam AW    = `log2(DEPTH);

  reg [31:0] mem [0:DEPTH-1];

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
  // Transfer state machine: IDLE -> WAIT1 -> WAIT2 -> DONE
  // hready_o high only on DONE (data valid).
  // -------------------------------------------------------------------------
  localparam [1:0] ST_IDLE  = 2'd0;
  localparam [1:0] ST_WAIT1 = 2'd1;
  localparam [1:0] ST_WAIT2 = 2'd2;
  localparam [1:0] ST_DONE  = 2'd3;

  reg [1:0]    state_q;
  reg [AW-1:0] addr_q;
  reg          read_q;
  reg [31:0]   rdata_q;

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q <= ST_IDLE;
      addr_q  <= {AW{1'b0}};
      read_q  <= 1'b0;
      rdata_q <= 32'h0;
    end else begin
      case (state_q)
        ST_IDLE: begin
          if (hsel_i && hready_i) begin
            addr_q  <= haddr_i[AW+1:2];
            read_q  <= !hwrite_i;
            state_q <= ST_WAIT1;
          end
        end
        ST_WAIT1: begin
          if (read_q) begin
            rdata_q <= mem[addr_q];
          end
          state_q <= ST_WAIT2;
        end
        ST_WAIT2: state_q <= ST_DONE;
        ST_DONE:  state_q <= ST_IDLE;
        default:  state_q <= ST_IDLE;
      endcase
    end
  end

  assign hready_o     = (state_q == ST_DONE);
  assign hresp_o      = `HRESP_OKAY;
  assign hrdata_o     = rdata_q;

  assign ctrl_status_o = 32'h0000_0000;
  assign ctrl_busy_o   = 1'b0;

endmodule
