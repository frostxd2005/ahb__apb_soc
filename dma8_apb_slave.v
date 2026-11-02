`timescale 1ns/1ps

//=============================================================================
// dma8_apb_slave.v
//-----------------------------------------------------------------------------
// APB configuration slave of the 8-stream DMA controller.  Register model
// compatible with the proven 4-channel dma_apb_slave (same offsets for the
// low channels), extended to 8 streams and a peripheral-request selector.
//
// Register map (per stream, 16-byte stride; offsets within the stream):
//   +0x00  SRC    source start address (word aligned)
//   +0x04  DEST   destination start address (word aligned)
//   +0x08  COUNT  transfer count in 32-bit words
//   +0x0C  CFG    bit0    = START; hardware auto-clears on transfer_done
//                 bit1    = SRC_FIXED (do not increment source address)
//                 bit2    = DEST_FIXED (do not increment destination address)
//                 bits7:4 = REQ_SEL: 0 = software/memory-to-memory,
//                 n (1..8) = stream fires only while peripheral request
//                 line n-1 (dma req_i[n-1]) is asserted
//
// Stream select = paddr[6:4], register select = paddr[3:2]; paddr[31:7] and
// paddr[1:0] must be zero.  Stream 0 therefore keeps exactly the addresses
// the old 4-channel firmware used (SRC 0x00, DEST 0x04, COUNT 0x08,
// CFG 0x0C within the DMA window).
//
// pready is always 1; pslverr on out-of-range or misaligned offsets.
// Plain Verilog (IEEE 1364-2001).
//============================================================================

module dma8_apb_slave (
  input  wire          clk,
  input  wire          rstn,

  // APB slave
  input  wire          psel,
  input  wire          penable,
  input  wire          pwrite,
  input  wire [31:0]   paddr,
  input  wire [31:0]   pwdata,
  output reg  [31:0]   prdata,
  output wire          pready,
  output wire          pslverr,

  // configuration fan-out to the DMA engine (8 x 32-bit packed)
  output wire [255:0]  src_addr,
  output wire [255:0]  dest_addr,
  output wire [255:0]  count_addr,
  output wire [31:0]   req_sel_flat,   // 8 x 4-bit REQ_SEL fields
  output wire [7:0]    src_fixed,      // CFG[1] per stream
  output wire [7:0]    dest_fixed,     // CFG[2] per stream
  output wire [7:0]    req,            // START bits
  input  wire [7:0]    done
);

  reg [31:0] reg_src   [0:7];
  reg [31:0] reg_dest  [0:7];
  reg [31:0] reg_count [0:7];
  reg [31:0] reg_config[0:7];

  wire [2:0] ch_id  = paddr[6:4];
  wire [1:0] reg_id = paddr[3:2];
  wire       addr_valid = (paddr[31:7] == 25'd0) && (paddr[1:0] == 2'b00);

  integer i;

  // ---- write path / auto-clear of START bits ----
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      for (i = 0; i < 8; i = i + 1) begin
        reg_src[i]    <= 32'd0;
        reg_dest[i]   <= 32'd0;
        reg_count[i]  <= 32'd0;
        reg_config[i] <= 32'd0;
      end
    end else begin
      for (i = 0; i < 8; i = i + 1) begin
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
  assign src_addr   = {reg_src[7],   reg_src[6],   reg_src[5],   reg_src[4],
                       reg_src[3],   reg_src[2],   reg_src[1],   reg_src[0]};
  assign dest_addr  = {reg_dest[7],  reg_dest[6],  reg_dest[5],  reg_dest[4],
                       reg_dest[3],  reg_dest[2],  reg_dest[1],  reg_dest[0]};
  assign count_addr = {reg_count[7], reg_count[6], reg_count[5], reg_count[4],
                       reg_count[3], reg_count[2], reg_count[1], reg_count[0]};
  assign req_sel_flat = {reg_config[7][7:4], reg_config[6][7:4],
                         reg_config[5][7:4], reg_config[4][7:4],
                         reg_config[3][7:4], reg_config[2][7:4],
                         reg_config[1][7:4], reg_config[0][7:4]};
  assign src_fixed  = {reg_config[7][1], reg_config[6][1], reg_config[5][1],
                       reg_config[4][1], reg_config[3][1], reg_config[2][1],
                       reg_config[1][1], reg_config[0][1]};
  assign dest_fixed = {reg_config[7][2], reg_config[6][2], reg_config[5][2],
                       reg_config[4][2], reg_config[3][2], reg_config[2][2],
                       reg_config[1][2], reg_config[0][2]};
  // Suppress a completed stream immediately during the cycle before the
  // registered APB auto-clear becomes visible. Without this done gating, a
  // still-high START can be re-granted once and repeat a single-beat DMA.
  assign req = {reg_config[7][0] & ~done[7], reg_config[6][0] & ~done[6],
                reg_config[5][0] & ~done[5], reg_config[4][0] & ~done[4],
                reg_config[3][0] & ~done[3], reg_config[2][0] & ~done[2],
                reg_config[1][0] & ~done[1], reg_config[0][0] & ~done[0]};

endmodule
