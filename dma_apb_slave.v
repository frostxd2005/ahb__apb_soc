`timescale 1ns/1ps

//=============================================================================
// dma_apb_slave.v
//------------------------------------------------------------------------------
// APB configuration slave of the 4-channel DMA controller. Adapted from
// varun23-2004/-4-Channel-AHB-APB-DMA-Controller-RTL-Design-.
//
// Register map (per channel, 16-byte stride; offsets within the channel):
//   +0x00  SRC    source start address (word aligned)
//   +0x04  DEST   destination start address (word aligned)
//   +0x08  COUNT  transfer count in 32-bit words
//   +0x0C  CFG    bit0 = START; hardware auto-clears on transfer_done
//
// Channel select = paddr[5:4], register select = paddr[3:2]; paddr[31:6] and
// paddr[1:0] must be zero. Because this project's ahb_to_apb bridge presents
// a 12-bit window offset per APB slave, the whole register file lives in the
// first 64 bytes of its 4 kB window (e.g. base 0x8000_6000, ch3 CFG at
// 0x8000_603C).
//
// Fixes over the original repository file:
//   * the write decoder stored DEST writes into reg_count (reg_dest was never
//     writable); 2'b01 now writes reg_dest.
//   * reset never cleared reg_dest; it is now reset like the others.
//
// pready is always 1 (single-cycle APB accesses); pslverr on out-of-range or
// misaligned offsets.
//=============================================================================

module dma_apb_slave (
  input  wire         clk,
  input  wire         rstn,

  // APB slave
  input  wire         psel,
  input  wire         penable,
  input  wire         pwrite,
  input  wire [31:0]  paddr,
  input  wire [31:0]  pwdata,
  output reg  [31:0]  prdata,
  output wire         pready,
  output wire         pslverr,

  // configuration fan-out to the DMA engine (4 x 32-bit packed)
  output wire [127:0] src_addr,
  output wire [127:0] dest_addr,
  output wire [127:0] count_addr,
  output wire [3:0]   req,
  input  wire [3:0]   done
);

  reg [31:0] reg_src   [0:3];
  reg [31:0] reg_dest  [0:3];
  reg [31:0] reg_count [0:3];
  reg [31:0] reg_config[0:3];

  wire [1:0] ch_id  = paddr[5:4];
  wire [1:0] reg_id = paddr[3:2];
  wire       addr_valid = (paddr[31:6] == 26'd0) && (paddr[1:0] == 2'b00);

  integer i;

  // ---- write path / auto-clear of START bits ----
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      for (i = 0; i < 4; i = i + 1) begin
        reg_src[i]    <= 32'd0;
        reg_dest[i]   <= 32'd0;
        reg_count[i]  <= 32'd0;
        reg_config[i] <= 32'd0;
      end
    end else begin
      for (i = 0; i < 4; i = i + 1) begin
        if (done[i]) reg_config[i][0] <= 1'b0;
      end
      if (psel && penable && pwrite && addr_valid && !req[ch_id]) begin
        case (reg_id)
          2'b00:   reg_src[ch_id]    <= pwdata;
          2'b01:   reg_dest[ch_id]   <= pwdata;
          2'b10:   reg_count[ch_id]  <= pwdata;
          2'b11:   reg_config[ch_id] <= pwdata;
          default: ;
        endcase
      end
    end
  end

  // ---- read path ----
  always @(*) begin
    if (psel && penable && !pwrite) begin
      case (reg_id)
        2'b00:   prdata = reg_src[ch_id];
        2'b01:   prdata = reg_dest[ch_id];
        2'b10:   prdata = reg_count[ch_id];
        2'b11:   prdata = reg_config[ch_id];
        default: prdata = 32'd0;
      endcase
    end else begin
      prdata = 32'd0;
    end
  end

  assign pready  = 1'b1;
  assign pslverr = psel && penable && !addr_valid;

  // ---- packed fan-out ----
  assign src_addr   = {reg_src[3],   reg_src[2],   reg_src[1],   reg_src[0]};
  assign dest_addr  = {reg_dest[3],  reg_dest[2],  reg_dest[1],  reg_dest[0]};
  assign count_addr = {reg_count[3], reg_count[2], reg_count[1], reg_count[0]};
  assign req        = {reg_config[3][0], reg_config[2][0], reg_config[1][0], reg_config[0][0]};

endmodule
