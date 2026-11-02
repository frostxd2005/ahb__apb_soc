//=============================================================================
// apb_uart.v
//------------------------------------------------------------------------------
// 16550-style UART on an APB3 slave port, written in plain Verilog
// (IEEE 1364-2001).  Register-compatible with the pulp-platform obi_uart /
// apb_uart programming model (the standard 16550 map):
//
//   Offset  DLAB=0              DLAB=1
//   0x00    RBR (R) / THR (W)   DLL
//   0x04    IER                 DLH
//   0x08    IIR (R) / FCR (W)
//   0x0C    LCR
//   0x10    MCR
//   0x14    LSR
//   0x18    MSR
//   0x1C    SCR
//
// Features: 5-8 data bits, parity (even/odd/stick), 1/2 stop bits, 16x
// oversampled RX with mid-bit sampling, TX holding register (THRE/TEMT),
// overrun/framing/parity/break detection, loopback (MCR[4]), modem lines,
// 16550 interrupt identification (IIR) with IER enables.
//
// PADDR: use PADDR[4:2] as the 3-bit register index (byte offsets above).
//=============================================================================
`timescale 1ns/1ps

module apb_uart (
  input  wire        CLK,
  input  wire        RSTN,

  // APB3 slave
  input  wire        PSEL,
  input  wire        PENABLE,
  input  wire        PWRITE,
  input  wire [11:0] PADDR,
  input  wire [31:0] PWDATA,
  output reg  [31:0] PRDATA,
  output wire        PREADY,
  output wire        PSLVERR,

  output reg         INT,

  // modem / optional outputs
  output reg         OUT1N,
  output reg         OUT2N,
  output reg         RTSN,
  output reg         DTRN,
  input  wire        CTSN,
  input  wire        DSRN,
  input  wire        DCDN,
  input  wire        RIN,

  // serial
  input  wire        SIN,
  output reg         SOUT
);

  // -------------------------------------------------------------------------
  // Register index
  // -------------------------------------------------------------------------
  wire [2:0] a = PADDR[4:2];

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  reg [7:0] rbr_q;      // receive buffer
  reg [7:0] thr_q;      // transmit holding
  reg [7:0] ier_q;      // interrupt enable
                        // IIR is combinational (iir_c), see below
  reg [7:0] fcr_q;      // FIFO control (FIFOs not implemented; accepted)
  reg [7:0] lcr_q;      // line control
  reg [7:0] mcr_q;      // modem control
  reg [7:0] lsr_q;      // line status
  reg [7:0] msr_q;      // modem status
  reg [7:0] scr_q;      // scratch
  reg [7:0] dll_q;      // divisor latch low
  reg [7:0] dlh_q;      // divisor latch high

  wire       dlab = lcr_q[7];
  wire [3:0] data_bits = lcr_q[1:0] + 4'd5;      // 5..8
  wire       stb  = lcr_q[2];                    // 1 -> 2 stop bits
  wire       pen  = lcr_q[3];
  wire       eps  = lcr_q[4];
  wire       sp   = lcr_q[5];
  wire       bc   = lcr_q[6];

  wire rbr_read_c = PSEL && PENABLE && !PWRITE && (a == 3'd0) && !dlab;
  wire lsr_read_c = PSEL && PENABLE && !PWRITE && (a == 3'd5);

  // -------------------------------------------------------------------------
  // Baud rate generator (16x oversampling)
  //
  // Two counters derive from the divisor, each phase-synced to its own frame
  // edge (a single free-running counter would randomise both sample phases):
  //   * tx_baud_cnt (TX section) - cleared while TX is idle, so a frame
  //     starts with fresh phase and the start bit is exactly one bit time;
  //   * baud16_cnt  (RX section) - cleared on start-edge detection, so the
  //     mid-bit sample points are exact.
  // -------------------------------------------------------------------------
  wire [19:0] div16 = {dlh_q, dll_q, 4'b0000};   // TX bit period = divisor x 16
  wire [19:0] div16_clamped = (div16 == 20'd0) ? 20'd16 : div16;
  wire [19:0] rx_divisor = {4'b0000, dlh_q, dll_q}; // one 16x RX oversample tick
  wire [19:0] rx_divisor_clamped = (rx_divisor == 20'd0) ? 20'd1 : rx_divisor;

  // -------------------------------------------------------------------------
  // Transmit
  // -------------------------------------------------------------------------
  localparam [1:0] TX_IDLE  = 2'd0;
  localparam [1:0] TX_START = 2'd1;
  localparam [1:0] TX_DATA  = 2'd2;
  localparam [1:0] TX_STOP  = 2'd3;

  reg  [1:0] tx_state_q;
  reg  [7:0] tsr_q;                    // transmit shift register
  reg  [3:0] tx_bit_q;                 // bits sent so far
  reg  [1:0] tx_stop_cnt_q;
  reg        tx_busy_q;
  reg        tx_par_q;                 // parity bit for the current byte
  reg        thr_empty_q;              // THRE

  wire tx_par_c = sp ? ~eps : ((^thr_q) ^ ~eps);

  // TX-dedicated baud counter: held at 0 while idle, so the start bit gets a
  // fresh phase and lasts exactly div16 cycles (one bit time).
  wire       tx_active_c = (tx_state_q != TX_IDLE);
  reg [19:0] tx_baud_cnt;
  wire       tx_tick_c   = (tx_baud_cnt == div16_clamped - 20'd1);

  always @(posedge CLK) begin
    if (!RSTN)             tx_baud_cnt <= 20'd0;
    else if (!tx_active_c) tx_baud_cnt <= 20'd0;
    else if (tx_tick_c)    tx_baud_cnt <= 20'd0;
    else                   tx_baud_cnt <= tx_baud_cnt + 20'd1;
  end

  always @(posedge CLK) begin
    if (!RSTN) begin
      tx_state_q    <= TX_IDLE;
      tsr_q         <= 8'h0;
      tx_bit_q      <= 4'h0;
      tx_stop_cnt_q <= 2'd0;
      tx_busy_q     <= 1'b0;
      tx_par_q      <= 1'b0;
      thr_empty_q   <= 1'b1;
      SOUT          <= 1'b1;
    end else begin
      // THR write (DLAB=0): only flag the holding register here (the byte
      // itself is stored in the APB write block, keeping a single driver on
      // thr_q)
      if (PSEL && PENABLE && PWRITE && (a == 3'd0) && !dlab) begin
        thr_empty_q <= 1'b0;
      end

      case (tx_state_q)
        TX_IDLE: begin
          SOUT <= 1'b1;
          if (!thr_empty_q) begin
            tsr_q       <= thr_q;
            tx_par_q    <= tx_par_c;
            thr_empty_q <= 1'b1;               // THRE re-asserts
            tx_bit_q    <= 4'd0;
            tx_busy_q   <= 1'b1;
            if (bc) begin
              SOUT       <= 1'b0;              // break: hold low
              tx_state_q <= TX_STOP;           // "stop" placeholder
              tx_stop_cnt_q <= 2'd3;           // long break window
            end else begin
              SOUT       <= 1'b0;              // start bit
              tx_state_q <= TX_START;
            end
          end else begin
            tx_busy_q <= 1'b0;
          end
        end
        TX_START: begin
          if (tx_tick_c) begin
            // The start bit has lasted exactly one bit time.  Present data
            // bit 0 now (not one extra baud interval later), then continue
            // with bits 1..N-1 in TX_DATA.
            SOUT       <= tsr_q[0];
            tsr_q      <= {1'b0, tsr_q[7:1]};
            tx_bit_q   <= 4'd1;
            tx_state_q <= TX_DATA;
          end
        end
        TX_DATA: begin
          if (tx_tick_c) begin
            if (tx_bit_q < data_bits) begin
              SOUT     <= tsr_q[0];            // LSB first
              tsr_q    <= {1'b0, tsr_q[7:1]};
              tx_bit_q <= tx_bit_q + 4'd1;
            end else if (pen && (tx_bit_q == data_bits)) begin
              SOUT     <= tx_par_q;
              tx_bit_q <= tx_bit_q + 4'd1;
            end else begin
              SOUT          <= 1'b1;           // stop bit
              tx_stop_cnt_q <= (stb ? 2'd2 : 2'd1);
              tx_state_q    <= TX_STOP;
            end
          end
        end
        TX_STOP: begin
          if (tx_tick_c) begin
            if (tx_stop_cnt_q > 2'd1) begin
              tx_stop_cnt_q <= tx_stop_cnt_q - 2'd1;
            end else begin
              tx_state_q <= TX_IDLE;
            end
          end
        end
        default: tx_state_q <= TX_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Receive (16x oversampled, mid-bit sampling)
  // -------------------------------------------------------------------------
  localparam [2:0] RX_IDLE  = 3'd0;
  localparam [2:0] RX_START = 3'd1;
  localparam [2:0] RX_DATA  = 3'd2;
  localparam [2:0] RX_PAR   = 3'd3;
  localparam [2:0] RX_STOP  = 3'd4;

  reg  [2:0] rx_state_q;
  reg  [7:0] rxb_q;                     // rx shift (LSB first)
  reg  [3:0] rx_bit_q;
  reg  [4:0] rx_tick_q;                 // tick position within a bit
  reg        rx_par_bit_q;              // received parity bit
  reg        rxd_q;                     // registered input
  reg        rx_done_pend_q;

  // RX 16x oversampling counter, phase-synced on start-edge detection so the
  // mid-bit sample points are exact (was the shared free-running counter).
  wire       rx_start_det_c = (rx_state_q == RX_IDLE) && !rxd_q;
  reg [19:0] baud16_cnt;
  wire       baud16_tick = (baud16_cnt == rx_divisor_clamped - 20'd1);

  always @(posedge CLK) begin
    if (!RSTN)               baud16_cnt <= 20'd0;
    else if (rx_start_det_c) baud16_cnt <= 20'd0;
    else if (baud16_tick)    baud16_cnt <= 20'd0;
    else                     baud16_cnt <= baud16_cnt + 20'd1;
  end

  // one-cycle pulse when a received byte is committed (end of stop bit)
  wire rx_commit_c = (rx_state_q == RX_STOP) && (rx_tick_q == 5'd15);

  // RX input mux (loopback)
  wire rxd_loop = (mcr_q[4]) ? SOUT : SIN;

  always @(posedge CLK) begin
    if (!RSTN) begin
      rx_state_q     <= RX_IDLE;
      rxb_q          <= 8'h0;
      rx_bit_q       <= 4'd0;
      rx_tick_q      <= 5'd0;
      rx_par_bit_q   <= 1'b0;
      rx_done_pend_q <= 1'b0;
      rxd_q          <= 1'b1;
    end else begin
      rxd_q <= rxd_loop;
      if (rbr_read_c) rx_done_pend_q <= 1'b0;   // RBR read frees the slot

      case (rx_state_q)
        RX_IDLE: begin
          if (!rxd_q) begin               // falling edge -> start candidate
            rx_state_q <= RX_START;
            rx_tick_q  <= 5'd0;
          end
        end
        RX_START: begin
          if (baud16_tick) begin
            if (rx_tick_q < 5'd8) begin
              rx_tick_q <= rx_tick_q + 5'd1;
            end else begin
              if (rxd_q) begin            // false start
                rx_state_q <= RX_IDLE;
              end else begin
                rx_state_q <= RX_DATA;
                rx_bit_q   <= 4'd0;
                rx_tick_q  <= 5'd0;
                rxb_q      <= 8'h0;
              end
            end
          end
        end
        RX_DATA: begin
          if (baud16_tick) begin
            if (rx_tick_q == 5'd15) begin // sample at mid of bit
              rxb_q     <= {rxd_q, rxb_q[7:1]};   // LSB first
              rx_tick_q <= 5'd0;
              if (rx_bit_q == data_bits - 4'd1) begin
                if (pen) begin
                  rx_state_q <= RX_PAR;
                end else begin
                  rx_state_q <= RX_STOP;
                end
              end else begin
                rx_bit_q <= rx_bit_q + 4'd1;
              end
            end else begin
              rx_tick_q <= rx_tick_q + 5'd1;
            end
          end
        end
        RX_PAR: begin
          if (baud16_tick) begin
            if (rx_tick_q == 5'd15) begin
              rx_par_bit_q <= rxd_q;
              rx_state_q   <= RX_STOP;
            end else begin
              rx_tick_q <= rx_tick_q + 5'd1;
            end
          end
        end
        RX_STOP: begin
          if (baud16_tick) begin
            if (rx_tick_q == 5'd15) begin
              // commit the received byte; LSR flags are applied by the LSR
              // block on the rx_commit_c pulse (single driver on lsr_q)
              if (!rx_done_pend_q)
                rbr_q <= rxb_q;                 // keep old RBR on overrun
              rx_done_pend_q <= 1'b1;
              rx_state_q     <= RX_IDLE;
            end else begin
              rx_tick_q <= rx_tick_q + 5'd1;
            end
          end
        end
        default: rx_state_q <= RX_IDLE;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Line status register
  // -------------------------------------------------------------------------
  always @(posedge CLK) begin
    if (!RSTN) begin
      lsr_q <= 8'h60;                     // THRE + TEMT set
    end else begin
      lsr_q[5] <= thr_empty_q;            // THRE
      lsr_q[6] <= thr_empty_q && !tx_busy_q;  // TEMT
      if (rbr_read_c) lsr_q[0] <= 1'b0;   // DR clears on RBR read
      if (lsr_read_c) lsr_q[4:1] <= 4'h0; // errors clear on LSR read
      if (rx_commit_c) begin              // byte committed by the RX engine
        if (rx_done_pend_q) lsr_q[1] <= 1'b1;  // overrun error
        lsr_q[0] <= 1'b1;                      // DR
        if (~rxd_q) begin                      // framing error (and break)
          lsr_q[3] <= 1'b1;
          if (rxb_q == 8'h0) lsr_q[4] <= 1'b1; // break
        end
        if (pen && (rx_par_bit_q != (sp ? ~eps : ((^rxb_q) ^ ~eps))))
          lsr_q[2] <= 1'b1;                    // parity error
      end
    end
  end

  // -------------------------------------------------------------------------
  // Modem status
  // -------------------------------------------------------------------------
  always @(posedge CLK) begin
    if (!RSTN) begin
      msr_q <= 8'h00;
    end else begin
      if (mcr_q[4]) begin
        msr_q[7:4] <= {mcr_q[3], mcr_q[2], mcr_q[0], mcr_q[1]};
      end else begin
        msr_q[4] <= ~CTSN;
        msr_q[5] <= ~DSRN;
        msr_q[6] <= ~RIN;
        msr_q[7] <= ~DCDN;
      end
      if (PSEL && PENABLE && !PWRITE && (a == 3'd6)) begin
        msr_q[3:0] <= 4'h0;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Interrupt identification (IIR) and INT output
  // -------------------------------------------------------------------------
  reg [7:0] iir_c;
  always @(*) begin
    if (|lsr_q[4:1] && ier_q[2])      iir_c = 8'h06;   // RX line status
    else if (lsr_q[0] && ier_q[0])    iir_c = 8'h04;   // RX data available
    else if (thr_empty_q && ier_q[1]) iir_c = 8'h0C;   // THRE
    else if (|msr_q[3:0] && ier_q[3]) iir_c = 8'h00;   // modem status
    else                              iir_c = 8'h01;   // none
  end

  always @(posedge CLK) begin
    if (!RSTN) INT <= 1'b0;
    else INT <= (iir_c[0] == 1'b0);
  end

  // -------------------------------------------------------------------------
  // APB register write
  // -------------------------------------------------------------------------
  always @(posedge CLK) begin
    if (!RSTN) begin
      ier_q <= 8'h00; fcr_q <= 8'h00; lcr_q <= 8'h00; mcr_q <= 8'h00;
      scr_q <= 8'h00; dll_q <= 8'h01; dlh_q <= 8'h00;
      OUT1N <= 1'b1; OUT2N <= 1'b1; RTSN <= 1'b1; DTRN <= 1'b1;
    end else if (PSEL && PENABLE && PWRITE) begin
      case (a)
        3'd0: if (dlab) dll_q <= PWDATA[7:0];
              else thr_q <= PWDATA[7:0];
        3'd1: if (dlab) dlh_q <= PWDATA[7:0];
              else ier_q <= PWDATA[7:0];
        3'd2: fcr_q  <= PWDATA[7:0];
        3'd3: lcr_q  <= PWDATA[7:0];
        3'd4: begin
                mcr_q <= PWDATA[7:0];
                DTRN  <= ~PWDATA[0];
                RTSN  <= ~PWDATA[1];
                OUT1N <= ~PWDATA[2];
                OUT2N <= ~PWDATA[3];
              end
        3'd7: scr_q <= PWDATA[7:0];
        default: ;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // APB register read
  // -------------------------------------------------------------------------
  always @(*) begin
    PRDATA = 32'h0;
    case (a)
      3'd0: PRDATA = dlab ? {24'h0, dll_q} : {24'h0, rbr_q};
      3'd1: PRDATA = dlab ? {24'h0, dlh_q} : {24'h0, ier_q};
      3'd2: PRDATA = {24'h0, iir_c};
      3'd3: PRDATA = {24'h0, lcr_q};
      3'd4: PRDATA = {24'h0, mcr_q};
      3'd5: PRDATA = {24'h0, lsr_q};
      3'd6: PRDATA = {24'h0, msr_q};
      3'd7: PRDATA = {24'h0, scr_q};
      default: PRDATA = 32'h0;
    endcase
  end

  assign PREADY  = 1'b1;
  assign PSLVERR = 1'b0;

endmodule
