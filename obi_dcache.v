`timescale 1ns/1ps

//=============================================================================
// obi_dcache.v
//-----------------------------------------------------------------------------
// Data cache on the OBI side of the CPU: core-side OBI slave, memory-side
// OBI master (feeds the existing obi_to_ahb data bridge, which sits in front
// of the ahb_steer unit -> memory fabric / peripheral fabric).
//
//   * direct-mapped, 128 lines x 4 words = 2 kByte, word granularity
//   * addr: tag = [31:11], index = [10:4], word = [3:2]
//   * WRITE-THROUGH, NO-WRITE-ALLOCATE:
//       - store hit  : cache word updated (byte-enable merged) AND the beat
//                      goes to memory
//       - store miss : beat goes to memory only, nothing allocated
//     -> memory is always up to date, so DMA / boot-engine writes to memory
//        are seen by the CPU as long as software does not hold cached lines
//        of a region another master is writing (true for the smoke firmware:
//        the DMA destination is only read after the DMA has completed).
//   * read miss: full-line fill (4 beats), response after the line completes
//   * device region (addr[31], the APB island): ALWAYS bypassed -- single
//     memory beat, no lookup, no allocation.  This is what keeps UART/DMA
//     register polling working.
//   * OBI grant combinationally in IDLE -> one outstanding request, same
//     serialization as obi_to_ahb
//   * reset invalidates all lines
//
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module obi_dcache (
  input  wire         clk_i,
  input  wire         rst_ni,

  // ---- core side (OBI slave) ----
  input  wire         req_i,
  output wire         gnt_o,
  input  wire [31:0]  addr_i,
  input  wire         we_i,
  input  wire [3:0]   be_i,
  input  wire [31:0]  wdata_i,
  output wire         rvalid_o,
  output wire [31:0]  rdata_o,
  output wire         err_o,

  // ---- memory side (OBI master) ----
  output wire         m_req_o,
  input  wire         m_gnt_i,
  output wire [31:0]  m_addr_o,
  output wire         m_we_o,
  output wire [3:0]   m_be_o,
  output wire [31:0]  m_wdata_o,
  input  wire         m_rvalid_i,
  input  wire [31:0]  m_rdata_i,
  input  wire         m_err_i
);

  // -------------------------------------------------------------------------
  // Storage
  // -------------------------------------------------------------------------
  reg [31:0]  cdata  [0:511];    // 128 lines x 4 words
  reg [127:0] cvalid;
  reg [20:0]  ctag   [0:127];

  // -------------------------------------------------------------------------
  // FSM
  // -------------------------------------------------------------------------
  localparam [3:0] S_IDLE     = 4'd0;
  localparam [3:0] S_TAG      = 4'd1;
  localparam [3:0] S_FILL_REQ = 4'd2;  // read-miss line fill
  localparam [3:0] S_FILL_DAT = 4'd3;
  localparam [3:0] S_WR_REQ   = 4'd4;  // write-through beat
  localparam [3:0] S_WR_DAT   = 4'd5;
  localparam [3:0] S_BYPS_REQ = 4'd6;  // device-region beat
  localparam [3:0] S_BYPS_DAT = 4'd7;
  localparam [3:0] S_RESP     = 4'd8;

  reg [3:0]  state_q;
  reg [31:0] addr_q;
  reg        we_q;
  reg [3:0]  be_q;
  reg [31:0] wdata_q;
  reg [1:0]  cnt_q;
  reg [31:0] rsp_q;
  reg        err_q;

  wire [6:0]  idx_c  = addr_q[10:4];
  wire [1:0]  off_c  = addr_q[3:2];
  wire [20:0] tag_c  = addr_q[31:11];
  wire        dev_c  = addr_q[31];
  wire        hit_c  = cvalid[idx_c] && (ctag[idx_c] == tag_c);

  wire [8:0]  hit_word  = {idx_c, off_c};
  wire [8:0]  fill_word = {idx_c, cnt_q};

  always @(posedge clk_i) begin
    if (!rst_ni) begin
      state_q <= S_IDLE;
      addr_q  <= 32'h0;
      we_q    <= 1'b0;
      be_q    <= 4'h0;
      wdata_q <= 32'h0;
      cnt_q   <= 2'd0;
      rsp_q   <= 32'h0;
      err_q   <= 1'b0;
      cvalid  <= 128'd0;
    end else begin
      case (state_q)
        S_IDLE: begin
          err_q <= 1'b0;
          if (req_i) begin
            addr_q  <= addr_i;
            we_q    <= we_i;
            be_q    <= be_i;
            wdata_q <= wdata_i;
            state_q <= S_TAG;
          end
        end

        S_TAG: begin
          if (dev_c) begin
            state_q <= S_BYPS_REQ;               // device region: bypass
          end else if (we_q) begin
            if (hit_c) begin                     // write hit: update cache
              if (be_q[0]) cdata[hit_word][ 7: 0] <= wdata_q[ 7: 0];
              if (be_q[1]) cdata[hit_word][15: 8] <= wdata_q[15: 8];
              if (be_q[2]) cdata[hit_word][23:16] <= wdata_q[23:16];
              if (be_q[3]) cdata[hit_word][31:24] <= wdata_q[31:24];
            end
            state_q <= S_WR_REQ;                 // write-through either way
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
                cvalid[idx_c] <= 1'b1;
                ctag[idx_c]   <= tag_c;
              end
              state_q <= S_RESP;
            end else begin
              cnt_q   <= cnt_q + 2'd1;
              state_q <= S_FILL_REQ;
            end
          end
        end

        S_WR_REQ: begin
          if (m_gnt_i) state_q <= S_WR_DAT;
        end

        S_WR_DAT: begin
          if (m_rvalid_i) begin
            rsp_q   <= m_rdata_i;
            err_q   <= m_err_i;
            state_q <= S_RESP;
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

  assign m_req_o  = (state_q == S_FILL_REQ) || (state_q == S_WR_REQ) ||
                    (state_q == S_BYPS_REQ);

  assign m_addr_o = (state_q == S_FILL_REQ) ? {tag_c, idx_c, cnt_q, 2'b00}
                                            : addr_q;

  assign m_we_o    = ((state_q == S_WR_REQ) || (state_q == S_WR_DAT))  ? 1'b1 :
                     ((state_q == S_BYPS_REQ) || (state_q == S_BYPS_DAT)) ? we_q :
                     1'b0;

  assign m_be_o    = ((state_q == S_BYPS_REQ) || (state_q == S_BYPS_DAT)) ? be_q :
                     ((state_q == S_WR_REQ)   || (state_q == S_WR_DAT))   ? be_q :
                     4'hF;

  assign m_wdata_o = wdata_q;

endmodule
