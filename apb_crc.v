`timescale 1ns/1ps

//=============================================================================
// apb_crc.v
//-----------------------------------------------------------------------------
// CRC-32 engine (APB slot 5).  Bit-serial, MSB-first, polynomial 0x04C11DB7,
// initial value 0xFFFFFFFF, no input/output reflection, no final XOR
// (i.e. CRC-32/MPEG-2).  One 32-bit word takes 32 clocks after the DATAIN
// write; words fed back-to-back accumulate (do not re-init between them).
//
// APB register map (offsets within the 4 kB window):
//   0x00 CTRL    W    bit0 = 1: reinitialize the CRC register to 0xFFFFFFFF
//   0x04 DATAIN  W    push a 32-bit word into the engine (starts 32 cycles)
//   0x08 RESULT  RO   current CRC value
//   0x0C STATUS  RO   bit0 = busy
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module apb_crc (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- APB slave (slot 5) ----
  input  wire         psel_i,
  input  wire         penable_i,
  input  wire         pwrite_i,
  input  wire [11:0]  paddr_i,
  input  wire [31:0]  pwdata_i,
  output reg  [31:0]  prdata_o,
  output wire         pready_o,
  output wire         pslverr_o
);

  localparam [31:0] CRC_POLY = 32'h04C11DB7;

  reg [31:0] crc_q;
  reg [31:0] shift_q;
  reg [5:0]  cnt_q;          // 32..1 while busy, 0 = idle
  reg        busy_q;

  wire wr_c   = psel_i && penable_i && pwrite_i;
  wire init_c = wr_c && (paddr_i[3:2] == 2'b00) && pwdata_i[0];
  wire push_c = wr_c && (paddr_i[3:2] == 2'b01) && !busy_q;

  wire       fb_c = crc_q[31] ^ shift_q[31];

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      crc_q   <= 32'hFFFF_FFFF;
      shift_q <= 32'h0;
      cnt_q   <= 6'd0;
      busy_q  <= 1'b0;
    end else begin
      if (init_c) begin
        crc_q  <= 32'hFFFF_FFFF;
        busy_q <= 1'b0;
        cnt_q  <= 6'd0;
      end else if (push_c) begin
        shift_q <= pwdata_i;
        cnt_q   <= 6'd32;
        busy_q  <= 1'b1;
      end else if (busy_q) begin
        crc_q   <= fb_c ? {crc_q[30:0], 1'b0} ^ CRC_POLY
                        : {crc_q[30:0], 1'b0};
        shift_q <= {shift_q[30:0], 1'b0};
        cnt_q   <= cnt_q - 6'd1;
        if (cnt_q == 6'd1) busy_q <= 1'b0;
      end
    end
  end

  // ---- APB read path ----
  always @(*) begin
    prdata_o = 32'h0;
    if (psel_i && penable_i && !pwrite_i) begin
      case (paddr_i[3:2])
        2'b10:   prdata_o = crc_q;
        2'b11:   prdata_o = {31'h0, busy_q};
        default: prdata_o = 32'h0;   // CTRL / DATAIN are write-only
      endcase
    end
  end

  assign pready_o  = 1'b1;
  assign pslverr_o = 1'b0;

endmodule
