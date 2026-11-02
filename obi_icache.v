`timescale 1ns/1ps

//=============================================================================
// obi_icache.v
//-----------------------------------------------------------------------------
// Instruction cache on the OBI side of the CPU: core-side OBI slave,
// memory-side OBI master (feeds the existing obi_to_ahb instruction bridge).
//
//   * direct-mapped, 256 lines x 4 words = 4 kByte, word granularity
//   * addr: tag = [31:12], index = [11:4], word = [3:2]
//   * hit latency: rvalid 2 cycles after the req/gnt handshake
//   * miss: full-line fill (4 sequential memory beats), response after the
//     line is complete and marked valid
//   * addresses with addr[31] set (device/peripheral region) bypass the
//     cache entirely (single memory beat, no allocation) -- instruction
//     fetches never go there in practice, this is pure safety
//   * OBI grant is given combinationally in IDLE, so exactly one request is
//     in flight -- the same serialization the obi_to_ahb bridge already
//     applies, and what the E40P instruction port expects from a memory
//   * reset invalidates all lines
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module obi_icache (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- core side (OBI slave) ----
  input  wire         req_i,
  output wire         gnt_o,
  input  wire [31:0]  addr_i,
  output wire         rvalid_o,
  output wire [31:0]  rdata_o,
  output wire         err_o,

  // ---- memory side (OBI master) ----
  output wire         m_req_o,
  input  wire         m_gnt_i,
  output wire [31:0]  m_addr_o,
  input  wire         m_rvalid_i,
  input  wire [31:0]  m_rdata_i,
  input  wire         m_err_i
);

  // -------------------------------------------------------------------------
  // Storage
  // -------------------------------------------------------------------------
  reg [31:0]  cdata  [0:1023];   // 256 lines x 4 words
  reg [255:0] cvalid;            // one valid bit per line
  reg [19:0]  ctag   [0:255];    // tag per line

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  localparam [2:0] S_IDLE     = 3'd0;
  localparam [2:0] S_TAG      = 3'd1;  // tag compare / hit response prep
  localparam [2:0] S_FILL_REQ = 3'd2;  // line fill: request memory beat
  localparam [2:0] S_FILL_DAT = 3'd3;  // line fill: wait for rvalid
  localparam [2:0] S_BYPS_REQ = 3'd4;  // device bypass: request
  localparam [2:0] S_BYPS_DAT = 3'd5;  // device bypass: wait rvalid
  localparam [2:0] S_RESP     = 3'd6;  // rvalid to the core

  reg [2:0]  state_q;
  reg [31:0] addr_q;
  reg [1:0]  cnt_q;              // fill word counter
  reg [31:0] rsp_q;
  reg        err_q;

  wire [7:0]  idx_c  = addr_q[11:4];
  wire [1:0]  off_c  = addr_q[3:2];
  wire [19:0] tag_c  = addr_q[31:12];
  wire        dev_c  = addr_q[31];
  wire        hit_c  = cvalid[idx_c] && (ctag[idx_c] == tag_c);

  wire [9:0]  hit_word  = {idx_c, off_c};
  wire [9:0]  fill_word = {idx_c, cnt_q};

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q <= S_IDLE;
      addr_q  <= 32'h0;
      cnt_q   <= 2'd0;
      rsp_q   <= 32'h0;
      err_q   <= 1'b0;
      cvalid  <= 256'd0;
    end else begin
      case (state_q)
        S_IDLE: begin
          err_q <= 1'b0;
          if (req_i) begin
            addr_q  <= addr_i;
            state_q <= S_TAG;
          end
        end

        S_TAG: begin
          if (dev_c) begin
            state_q <= S_BYPS_REQ;             // device region: bypass
          end else if (hit_c) begin
            rsp_q   <= cdata[hit_word];
            state_q <= S_RESP;
          end else begin
            cnt_q   <= 2'd0;
            state_q <= S_FILL_REQ;
          end
        end

        S_FILL_REQ: begin
          if (m_gnt_i) state_q <= S_FILL_DAT;
        end

        S_FILL_DAT: begin
          if (m_rvalid_i) begin
            cdata[fill_word] <= m_rdata_i;
            if (m_err_i) err_q <= 1'b1;
            if (cnt_q == off_c) rsp_q <= m_rdata_i;
            if (cnt_q == 2'd3) begin
              if (!err_q && !m_err_i) begin
                cvalid[idx_c] <= 1'b1;         // line complete -> valid
                ctag[idx_c]   <= tag_c;
              end
              state_q <= S_RESP;
            end else begin
              cnt_q   <= cnt_q + 2'd1;
              state_q <= S_FILL_REQ;
            end
          end
        end

        S_BYPS_REQ: begin
          if (m_gnt_i) state_q <= S_BYPS_DAT;
        end

        S_BYPS_DAT: begin
          if (m_rvalid_i) begin
            rsp_q   <= m_rdata_i;
            err_q   <= m_err_i;
            state_q <= S_RESP;
          end
        end

        S_RESP: state_q <= S_IDLE;

        default: state_q <= S_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Outputs
  // -------------------------------------------------------------------------
  assign gnt_o    = req_i && (state_q == S_IDLE);
  assign rvalid_o = (state_q == S_RESP);
  assign rdata_o  = rsp_q;
  assign err_o    = (state_q == S_RESP) && err_q;

  assign m_req_o  = (state_q == S_FILL_REQ) || (state_q == S_BYPS_REQ);
  assign m_addr_o = (state_q == S_BYPS_REQ) ? addr_q
                                            : {tag_c, idx_c, cnt_q, 2'b00};

endmodule
