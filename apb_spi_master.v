//=============================================================================
// apb_spi_master.v
//------------------------------------------------------------------------------
// SPI master on an APB3 slave port, written in plain Verilog (1364-2001).
// Register addresses match the pulp-platform apb_spi_master (spi_master_apb_if):
//
//   0x00 STATUS  W: [0] rd, [1] wr, [2] qrd, [3] qwr(unsupported), [4] swrst,
//                [11:8] csreg (bitmask of CS to assert, active high)
//                R: [0] busy, [1] eot(sticky), [2] tx_full, [3] rx_empty,
//                   [15:8] rx_level, [23:16] tx_level
//   0x04 CLKDIV  [7:0] clock divider; SPI clock period = 2*(clk_div+1) sysclks
//   0x08 CMD     [0] EOT (deassert CS after transfer), [3:2] SPI mode {CPOL,CPHA}
//   0x0C ADDR    [31:0] address shifted out before data
//   0x10 LEN     [5:0] cmd_len, [13:8] addr_len, [23:16]+[31:24] data_len (bits/word)
//   0x14 DUM     [15:0] dummy_rd bits, [31:16] dummy_wr bits
//   0x18 TXFIFO  (W) push 32-bit data word
//   0x20 RXFIFO  (R) pop 32-bit data word
//   0x24 INTCFG  [7:0] tx_thresh, [15:8] rx_thresh, [30] cnt_en, [31] int_en
//   0x28 INTSTA  (R) read clears interrupts
//
// Number of data words: taken from the TXFIFO occupancy at transfer start
// (for reads, push N placeholder words to select N receive words; their
// values are ignored).
//
// Data path: single-SPI (MOSI/MISO) mode, MSB first, SPI mode 0..3.
//=============================================================================
`timescale 1ns/1ps

module apb_spi_master #(
  parameter APB_ADDR_WIDTH = 12
)(
  input  wire                  HCLK,
  input  wire                  HRESETn,
  input  wire [APB_ADDR_WIDTH-1:0] PADDR,
  input  wire [31:0]           PWDATA,
  input  wire                  PWRITE,
  input  wire                  PSEL,
  input  wire                  PENABLE,
  output reg  [31:0]           PRDATA,
  output wire                  PREADY,
  output wire                  PSLVERR,

  output wire [1:0]            events_o,

  output reg                   spi_clk,
  output wire                  spi_csn0,
  output wire                  spi_csn1,
  output wire                  spi_csn2,
  output wire                  spi_csn3,
  output wire [1:0]            spi_mode,
  output reg                   spi_sdo0,
  output wire                  spi_sdo1,
  output wire                  spi_sdo2,
  output wire                  spi_sdo3,
  input  wire                  spi_sdi0,
  input  wire                  spi_sdi1,
  input  wire                  spi_sdi2,
  input  wire                  spi_sdi3
);

  wire [3:0] a = PADDR[5:2];          // word index

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  reg  [7:0] clk_div_q;
  reg [31:0] spi_cmd_q;
  reg [31:0] spi_addr_q;
  reg  [5:0] cmd_len_q;
  reg  [5:0] addr_len_q;
  reg [15:0] data_len_q;
  reg [15:0] dummy_rd_q;
  reg [15:0] dummy_wr_q;
  reg  [3:0] csreg_q;
  reg  [7:0] tx_thresh_q;
  reg  [7:0] rx_thresh_q;
  reg        int_en_q;
  reg        eot_sticky_q;

  reg  [1:0] spi_op_q;                // [1]=wr, [0]=rd
  wire       rd_op = spi_op_q[0];
  wire       wr_op = spi_op_q[1];

  wire [1:0] mode_c   = spi_cmd_q[3:2];
  wire       cpol     = spi_cmd_q[3];
  wire       cpha     = spi_cmd_q[2];
  wire       eot_c    = spi_cmd_q[0];

  // -------------------------------------------------------------------------
  // TX / RX FIFOs (depth 8, 32-bit words)
  // -------------------------------------------------------------------------
  localparam FIFO_DEPTH = 8;
  reg [31:0] tx_fifo [0:FIFO_DEPTH-1];
  reg [31:0] rx_fifo [0:FIFO_DEPTH-1];
  reg [3:0]  tx_wptr_q, tx_rptr_q, tx_cnt_q;
  reg [3:0]  rx_wptr_q, rx_rptr_q, rx_cnt_q;

  wire tx_full_c  = (tx_cnt_q == FIFO_DEPTH);
  wire tx_empty_c = (tx_cnt_q == 0);
  wire rx_full_c  = (rx_cnt_q == FIFO_DEPTH);
  wire rx_empty_c = (rx_cnt_q == 0);

  wire apb_write_c = PSEL && PENABLE && PWRITE;
  wire apb_read_c  = PSEL && PENABLE && !PWRITE;

  // -------------------------------------------------------------------------
  // Transfer engine states
  // -------------------------------------------------------------------------
  localparam [3:0] ST_IDLE = 4'd0;
  localparam [3:0] ST_CS   = 4'd1;   // CS setup, wait half period
  localparam [3:0] ST_NEXT = 4'd2;   // advance slot / end transfer
  localparam [3:0] ST_BIT  = 4'd3;   // bit engine
  localparam [3:0] ST_END  = 4'd4;   // EOT handling

  reg [3:0] state_q;

  localparam [1:0] SLOT_CMD  = 2'd0;
  localparam [1:0] SLOT_ADDR = 2'd1;
  localparam [1:0] SLOT_DUM  = 2'd2;
  localparam [1:0] SLOT_DATA = 2'd3;

  reg  [1:0] slot_q;
  reg [15:0] slot_bits_total_q;
  reg [15:0] slot_bits_done_q;
  reg [31:0] shift_q;                 // tx shift register
  reg [31:0] rx_word_q;               // rx accumulation (LSB first)
  reg [15:0] words_total_q;           // data words to transfer (from TX FIFO)
  reg [15:0] words_done_q;            // data words completed
  reg        is_read_q;
  reg        cs_active_q;

  // data word width, clamped to the 32-bit data path
  wire [15:0] data_len_eff_c = (data_len_q > 16'd32) ? 16'd32 : data_len_q;

  // first bit of the first slot (pre-driven on SDA at CS assert for CPHA=0)
  wire [31:0] first_data_c = (cmd_len_q != 6'd0) ? spi_cmd_q :
                             (addr_len_q != 6'd0) ? spi_addr_q :
                             (((is_read_q ? dummy_rd_q : dummy_wr_q)) != 16'd0) ? 32'h0 :
                             (wr_op ? tx_fifo[tx_rptr_q] : 32'h0);
  wire [15:0] first_len_c  = (cmd_len_q != 6'd0) ? {10'd0, cmd_len_q} :
                             (addr_len_q != 6'd0) ? {10'd0, addr_len_q} :
                             (((is_read_q ? dummy_rd_q : dummy_wr_q)) != 16'd0) ?
                               (is_read_q ? dummy_rd_q : dummy_wr_q) : data_len_eff_c;
  wire        first_bit_c  = (first_len_c != 16'd0) ? first_data_c[first_len_c - 16'd1] : 1'b0;

  // bit engine
  reg [8:0]  half_reg_q;
  reg        phase_q;                 // 0: sclk=CPOL, 1: sclk=~CPOL
  wire [8:0] half_c = (clk_div_q == 8'd0) ? 9'd1 : ({1'b0, clk_div_q} + 9'd1);
  wire       half_end_c = (state_q != ST_IDLE) && (half_reg_q == half_c - 9'd1);

  // -------------------------------------------------------------------------
  // CS outputs (bitmask in csreg_q, asserted while cs_active_q)
  // -------------------------------------------------------------------------
  assign spi_mode = mode_c;
  assign spi_csn0 = ~(cs_active_q && csreg_q[0]);
  assign spi_csn1 = ~(cs_active_q && csreg_q[1]);
  assign spi_csn2 = ~(cs_active_q && csreg_q[2]);
  assign spi_csn3 = ~(cs_active_q && csreg_q[3]);
  assign spi_sdo1 = 1'b0;
  assign spi_sdo2 = 1'b0;
  assign spi_sdo3 = 1'b0;

  // -------------------------------------------------------------------------
  // Bit engine datapath (combinational)
  // -------------------------------------------------------------------------
  reg        sdo_c;
  reg        sdo_next_c;
  reg [31:0] shift_next_c;
  reg [31:0] rx_word_next_c;
  reg        push_rx_c;
  reg        pop_tx_c;

  always @(*) begin
    sdo_c          = 1'b0;
    sdo_next_c     = 1'b0;
    shift_next_c   = shift_q;
    rx_word_next_c = rx_word_q;
    push_rx_c      = 1'b0;
    pop_tx_c       = 1'b0;

    case (slot_q)
      SLOT_CMD, SLOT_ADDR: begin
        sdo_c        = shift_q[slot_bits_total_q - 16'd1];
        shift_next_c = shift_q << 1;
        sdo_next_c   = shift_next_c[slot_bits_total_q - 16'd1];
      end
      SLOT_DUM: begin
        sdo_c = 1'b0;
      end
      SLOT_DATA: begin
        if (wr_op) begin
          sdo_c        = shift_q[data_len_eff_c - 16'd1];
          shift_next_c = shift_q << 1;
          sdo_next_c   = shift_next_c[data_len_eff_c - 16'd1];
        end else begin
          sdo_c = 1'b0;
        end
        // LSB-first accumulation: received bits land at [data_len-1:0]
        rx_word_next_c = {rx_word_q[30:0], spi_sdi0};
        push_rx_c      = is_read_q && (slot_bits_done_q == data_len_eff_c - 16'd1);
        // one FIFO word consumed per data slot (dummy words for reads)
        pop_tx_c       = (slot_bits_done_q == data_len_eff_c - 16'd1);
      end
      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Main control FSM
  // -------------------------------------------------------------------------
  always @(posedge HCLK) begin
    if (!HRESETn) begin
      state_q        <= ST_IDLE;
      slot_q         <= SLOT_CMD;
      slot_bits_total_q <= 16'd0;
      slot_bits_done_q  <= 16'd0;
      shift_q        <= 32'h0;
      rx_word_q      <= 32'h0;
      words_total_q  <= 16'd0;
      words_done_q   <= 16'd0;
      is_read_q      <= 1'b0;
      cs_active_q    <= 1'b0;
      half_reg_q     <= 9'd0;
      phase_q        <= 1'b0;
      spi_clk        <= 1'b0;
      spi_sdo0       <= 1'b0;
      eot_sticky_q   <= 1'b0;
      spi_op_q       <= 2'b00;
      tx_wptr_q      <= 4'd0;
      rx_rptr_q      <= 4'd0;
      // FIX: these were previously left un-reset, which makes them X in a
      // 4-state simulator.  int_en_q/thresholds feed events_o[0], so X there
      // poisons spi_evt_any and the INTC never sees a rising edge on source 1.
      clk_div_q      <= 8'h00;
      spi_cmd_q      <= 32'h0;
      spi_addr_q     <= 32'h0;
      cmd_len_q      <= 6'd0;
      addr_len_q     <= 6'd0;
      data_len_q     <= 16'd0;
      dummy_rd_q     <= 16'd0;
      dummy_wr_q     <= 16'd0;
      csreg_q        <= 4'd0;
      tx_thresh_q    <= 8'h00;
      rx_thresh_q    <= 8'h00;
      int_en_q       <= 1'b0;
    end else begin
      // ---- APB register access ----
      if (apb_write_c) begin
        case (a)
          4'h0: begin
            if (PWDATA[4]) begin
              tx_cnt_q <= 4'h0; rx_cnt_q <= 4'h0;
              cs_active_q <= 1'b0; state_q <= ST_IDLE;
            end else begin
              spi_op_q <= {PWDATA[1], PWDATA[0]};
              csreg_q  <= PWDATA[11:8];
              eot_sticky_q <= 1'b0;
            end
          end
          4'h1: clk_div_q <= PWDATA[7:0];
          4'h2: spi_cmd_q <= PWDATA;
          4'h3: spi_addr_q <= PWDATA;
          4'h4: begin
            cmd_len_q    <= PWDATA[5:0];
            addr_len_q   <= PWDATA[13:8];
            data_len_q[7:0]  <= PWDATA[23:16];
            data_len_q[15:8] <= PWDATA[31:24];
          end
          4'h5: begin
            dummy_rd_q[7:0]  <= PWDATA[7:0];
            dummy_rd_q[15:8] <= PWDATA[15:8];
            dummy_wr_q[7:0]  <= PWDATA[23:16];
            dummy_wr_q[15:8] <= PWDATA[31:24];
          end
          4'h6: begin
            if (!tx_full_c) begin
              tx_fifo[tx_wptr_q] <= PWDATA;
              tx_wptr_q <= tx_wptr_q + 4'd1;
            end
          end
          4'h9: begin
            tx_thresh_q <= PWDATA[7:0];
            rx_thresh_q <= PWDATA[15:8];
            int_en_q    <= PWDATA[31];
          end
          default: ;
        endcase
      end

      // RX FIFO pop on read
      if (apb_read_c && (a == 4'h8) && !rx_empty_c) begin
        rx_rptr_q <= rx_rptr_q + 4'd1;
      end

      // ---- transfer engine ----
      case (state_q)
        ST_IDLE: begin
          spi_clk <= cpol;               // idle level
          if (spi_op_q != 2'b00) begin
            is_read_q      <= rd_op;
            words_total_q  <= {12'h0, tx_cnt_q};
            words_done_q   <= 16'd0;
            slot_q         <= SLOT_CMD;
            slot_bits_done_q <= 16'd0;
            half_reg_q     <= 9'd0;
            phase_q        <= 1'b0;
            state_q        <= ST_CS;
          end
        end

        ST_CS: begin
          cs_active_q <= 1'b1;           // assert CS
          if (!cpha)
            spi_sdo0 <= first_bit_c;     // CPHA=0: present first bit now
          if (half_end_c) begin
            half_reg_q <= 9'd0;
            state_q    <= ST_NEXT;
          end else begin
            half_reg_q <= half_reg_q + 9'd1;
          end
        end

        ST_NEXT: begin
          slot_bits_done_q <= 16'd0;
          if (slot_q == SLOT_CMD) begin
            if (cmd_len_q != 6'd0) begin
              slot_bits_total_q <= {10'd0, cmd_len_q};
              shift_q <= spi_cmd_q;
              state_q <= ST_BIT;
            end else begin
              slot_q <= SLOT_ADDR;
              state_q <= ST_NEXT;
            end
          end else if (slot_q == SLOT_ADDR) begin
            if (addr_len_q != 6'd0) begin
              slot_bits_total_q <= {10'd0, addr_len_q};
              shift_q <= spi_addr_q;
              state_q <= ST_BIT;
            end else begin
              slot_q <= SLOT_DUM;
              state_q <= ST_NEXT;
            end
          end else if (slot_q == SLOT_DUM) begin
            if ((is_read_q ? dummy_rd_q : dummy_wr_q) != 16'd0) begin
              slot_bits_total_q <= is_read_q ? dummy_rd_q : dummy_wr_q;
              shift_q <= 32'h0;
              state_q <= ST_BIT;
            end else begin
              slot_q <= SLOT_DATA;
              state_q <= ST_NEXT;
            end
          end else begin // SLOT_DATA
            if ((data_len_eff_c != 16'd0) && (words_done_q < words_total_q)) begin
              slot_bits_total_q <= data_len_eff_c;
              shift_q <= wr_op ? tx_fifo[tx_rptr_q] : 32'h0;
              rx_word_q <= 32'h0;      // fresh accumulation per word
              if (!cpha)
                spi_sdo0 <= wr_op ? tx_fifo[tx_rptr_q][data_len_eff_c - 16'd1]
                                  : 1'b0;   // CPHA=0: present MSB while SCL low
              words_done_q <= words_done_q + 16'd1;
              state_q <= ST_BIT;
            end else begin
              state_q <= ST_END;
            end
          end
        end

        ST_BIT: begin
          if (half_end_c) begin
            half_reg_q <= 9'd0;
            if (phase_q == 1'b0) begin
              phase_q <= 1'b1;
              spi_clk <= ~cpol;          // first edge of the bit
            end else begin
              phase_q <= 1'b0;
              spi_clk <= cpol;           // second edge: bit completes
              slot_bits_done_q <= slot_bits_done_q + 16'd1;
              if (slot_bits_done_q == slot_bits_total_q - 16'd1) begin
                state_q <= ST_NEXT;      // slot finished
              end
            end
          end else begin
            half_reg_q <= half_reg_q + 9'd1;
          end
        end

        ST_END: begin
          if (eot_c) begin
            cs_active_q  <= 1'b0;
            eot_sticky_q <= 1'b1;
          end
          spi_op_q <= 2'b00;             // consume the command
          state_q  <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Datapath updates (shift out / sample in), inlined per edge
  // -------------------------------------------------------------------------
  always @(posedge HCLK) begin
    if (!HRESETn) begin
      shift_q   <= 32'h0;
      rx_word_q <= 32'h0;
      spi_sdo0  <= 1'b0;
    end else if ((state_q == ST_BIT) && half_end_c) begin
      if (phase_q == 1'b0) begin
        // first edge
        if (cpha) begin
          spi_sdo0 <= sdo_c;             // CPHA=1: change data here
          shift_q  <= shift_next_c;
        end else begin
          rx_word_q <= rx_word_next_c;   // CPHA=0: sample here
        end
      end else begin
        // second edge
        if (!cpha) begin
          spi_sdo0 <= sdo_next_c;        // CPHA=0: present the next bit after sampling
          shift_q  <= shift_next_c;
        end else begin
          rx_word_q <= rx_word_next_c;   // CPHA=1: sample here
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // FIFO counts (push/pop)
  // -------------------------------------------------------------------------
  always @(posedge HCLK) begin
    if (!HRESETn) begin
      tx_cnt_q  <= 4'h0;
      rx_cnt_q  <= 4'h0;
      tx_rptr_q <= 4'h0;
      rx_wptr_q <= 4'h0;
    end else begin
      if (apb_write_c && (a == 4'h6) && !tx_full_c) begin
        tx_cnt_q <= tx_cnt_q + 4'd1;
      end
      // FIFO word accounting happens once at the trailing edge of the last
      // bit.  slot_bits_done_q holds its terminal value across both SPI edges;
      // without the phase_q guard the same word is pushed/popped twice.
      if ((state_q == ST_BIT) && half_end_c && phase_q && pop_tx_c && !tx_empty_c) begin
        tx_cnt_q  <= tx_cnt_q - 4'd1;
        tx_rptr_q <= tx_rptr_q + 4'd1;
      end
      if ((state_q == ST_BIT) && half_end_c && phase_q && push_rx_c && !rx_full_c) begin
        rx_fifo[rx_wptr_q] <= cpha ? rx_word_next_c : rx_word_q;
        rx_wptr_q <= rx_wptr_q + 4'd1;
        rx_cnt_q  <= rx_cnt_q + 4'd1;
      end
      if (apb_read_c && (a == 4'h8) && !rx_empty_c) begin
        rx_cnt_q <= rx_cnt_q - 4'd1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Status / read mux / interrupts
  // -------------------------------------------------------------------------
  reg [31:0] status_c;
  always @(*) begin
    status_c = 32'h0;
    status_c[0]    = (state_q != ST_IDLE);
    status_c[1]    = eot_sticky_q;
    status_c[2]    = tx_full_c;
    status_c[3]    = rx_empty_c;
    status_c[15:8] = rx_cnt_q;
    status_c[23:16]= tx_cnt_q;
  end

  always @(*) begin
    PRDATA = 32'h0;
    case (a)
      4'h0: PRDATA = status_c;
      4'h1: PRDATA = {24'h0, clk_div_q};
      4'h2: PRDATA = spi_cmd_q;
      4'h3: PRDATA = spi_addr_q;
      4'h4: PRDATA = {data_len_q[15:8], data_len_q[7:0], 2'b00, addr_len_q, 2'b00, cmd_len_q};
      4'h5: PRDATA = {dummy_wr_q, dummy_rd_q};
      4'h8: PRDATA = rx_empty_c ? 32'h0 : rx_fifo[rx_rptr_q];
      4'h9: PRDATA = {int_en_q, 23'h0, rx_thresh_q[7:0], tx_thresh_q[7:0]};
      default: PRDATA = 32'h0;
    endcase
  end

  wire tx_int_c = int_en_q && (tx_cnt_q <= {1'b0, tx_thresh_q[3:0]});
  wire rx_int_c = int_en_q && (rx_cnt_q >= {1'b0, rx_thresh_q[3:0]});

  assign events_o[0] = tx_int_c || rx_int_c;
  assign events_o[1] = (state_q == ST_END) && eot_c;

  assign PREADY  = 1'b1;
  assign PSLVERR = 1'b0;

endmodule
