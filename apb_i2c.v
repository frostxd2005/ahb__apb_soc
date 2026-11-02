//=============================================================================
// apb_i2c.v
//------------------------------------------------------------------------------
// I2C master on an APB3 slave port, written in plain Verilog (1364-2001).
// Register map and behavior match the pulp-platform apb_i2c / opencores
// i2c_master (Richard Herveille) cores:
//
//   0x00 PRE  [15:0] clock prescaler. SCL period = 4*(PRE+1) system clocks
//                    (each SCL bit uses 4 timer states of PRE+1 clocks).
//   0x04 CTRL [7] EN (core enable), [6] IEN (interrupt enable)
//   0x08 RX   (R) last received byte
//   0x0C STA  (R) [7] RxACK (0 = ack received), [6] BUSY, [5] AL,
//                 [1] TIP (transfer in progress), [0] IRQ flag
//   0x10 TX   (W) byte to transmit (address or data)
//   0x14 CMD  (W) [7] STA, [6] STO, [5] RD, [4] WR, [3] ACK, [0] IACK
//                 command bits [7:4] auto-clear when the command completes
//                 (or on arbitration loss); [0] clears the interrupt flag.
//
// Transaction flow (as in the original core):
//   * write : TX=addr|0, CMD=STA|WR (0x90) -> start + address + ACK
//             then per byte: TX=data, CMD=WR (0x10)
//             end: CMD=STO (0x40)
//   * read  : TX=addr|1, CMD=STA|WR (0x90) -> start + address + ACK
//             then per byte: CMD=RD|ACK (0x28), last byte CMD=RD (0x20)
//             end: CMD=STO (0x40)
//   * repeated start: CMD=STA|RD (0xA0) or CMD=STA|WR (0x90)
//
// The SCL/SDA pads are open-drain: scl_pad_o / sda_pad_o are constant 0 and
// scl_padoen_o / sda_padoen_o are the active-low output enables (0 = drive).
// Supports clock stretching (slave pulls SCL low) and multimaster
// arbitration-loss detection.
//=============================================================================
`timescale 1ns/1ps

module apb_i2c #(
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
  output reg                   interrupt_o,
  input  wire                  scl_pad_i,
  output wire                  scl_pad_o,
  output wire                  scl_padoen_o,
  input  wire                  sda_pad_i,
  output wire                  sda_pad_o,
  output wire                  sda_padoen_o
);

  wire [3:0] a = PADDR[5:2];               // word index

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------
  reg [15:0] r_pre_q;                      // clock prescaler
  reg  [7:0] r_ctrl_q;                     // control
  reg  [7:0] r_tx_q;                       // transmit
  reg  [7:0] r_cmd_q;                      // command

  wire s_core_en_w = r_ctrl_q[7];
  wire s_ien_w     = r_ctrl_q[6];

  wire sta_w    = r_cmd_q[7];
  wire sto_w    = r_cmd_q[6];
  wire rd_w     = r_cmd_q[5];
  wire wr_w     = r_cmd_q[4];
  wire ack_in_w = r_cmd_q[3];
  wire iack_w   = r_cmd_q[0];

  // -------------------------------------------------------------------------
  // Command codes (bit controller)
  // -------------------------------------------------------------------------
  localparam [3:0] I2C_CMD_NOP   = 4'b0000;
  localparam [3:0] I2C_CMD_START = 4'b0001;
  localparam [3:0] I2C_CMD_STOP  = 4'b0010;
  localparam [3:0] I2C_CMD_WRITE = 4'b0100;
  localparam [3:0] I2C_CMD_READ  = 4'b1000;

  // -------------------------------------------------------------------------
  // Byte controller signals
  // -------------------------------------------------------------------------
  localparam [2:0] B_IDLE  = 3'd0;
  localparam [2:0] B_START = 3'd1;
  localparam [2:0] B_READ  = 3'd2;
  localparam [2:0] B_WRITE = 3'd3;
  localparam [2:0] B_ACK   = 3'd4;
  localparam [2:0] B_STOP  = 3'd5;

  reg [2:0] b_state_q;
  reg [7:0] sr_q;                          // shift register
  reg       ld_r, shift_r;
  reg       byte_cmd_ack_r;                // command acknowledge (s_done)
  reg       ack_out_r;                     // rxack
  reg [2:0] dcnt_q;                        // bit counter (7..0)
  reg [3:0] core_cmd_r;                    // command to the bit controller
  reg       core_txd_r;                    // bit to transmit

  wire      go_w        = (rd_w | wr_w | sto_w) & ~byte_cmd_ack_r;
  wire      cnt_done_w  = ~|dcnt_q;
  wire      s_done_w    = byte_cmd_ack_r;

  // -------------------------------------------------------------------------
  // Bit controller signals
  // -------------------------------------------------------------------------
  localparam [4:0] BIT_IDLE    = 5'd0;
  localparam [4:0] BIT_START_A = 5'd1;
  localparam [4:0] BIT_START_B = 5'd2;
  localparam [4:0] BIT_START_C = 5'd3;
  localparam [4:0] BIT_START_D = 5'd4;
  localparam [4:0] BIT_START_E = 5'd5;
  localparam [4:0] BIT_STOP_A  = 5'd6;
  localparam [4:0] BIT_STOP_B  = 5'd7;
  localparam [4:0] BIT_STOP_C  = 5'd8;
  localparam [4:0] BIT_STOP_D  = 5'd9;
  localparam [4:0] BIT_RD_A    = 5'd10;
  localparam [4:0] BIT_RD_B    = 5'd11;
  localparam [4:0] BIT_RD_C    = 5'd12;
  localparam [4:0] BIT_RD_D    = 5'd13;
  localparam [4:0] BIT_WR_A    = 5'd14;
  localparam [4:0] BIT_WR_B    = 5'd15;
  localparam [4:0] BIT_WR_C    = 5'd16;
  localparam [4:0] BIT_WR_D    = 5'd17;

  reg [4:0]  bit_state_q;
  reg        core_ack_q;
  reg        scl_oen_q;                     // active low (1 = released)
  reg        sda_oen_q;                     // active low (1 = released)
  reg        sda_chk_q;                     // SDA arbitration check phase
  reg [15:0] cnt_q;                         // prescale counter
  reg        clk_en_q;                      // one tick per (PRE+1) clocks
  reg        dscl_oen_q;
  reg        slave_wait_q;                  // clock stretching
  reg [13:0] filter_cnt_q;
  reg [1:0]  cSCL_q, cSDA_q;                // sync stages
  reg [2:0]  fSCL_q, fSDA_q;                // glitch filters
  reg        sSCL_q, sSDA_q;                // filtered bus levels
  reg        dSCL_q, dSDA_q;                // delayed filtered levels
  reg        sta_cond_q, sto_cond_q;
  reg        busy_q;                        // i2c_busy
  reg        cmd_stop_q;
  reg        al_bit_q;                      // i2c_al
  reg        sda_sample_q;                  // sampled SDA bit

  wire       scl_sync_w = dSCL_q & ~sSCL_q & scl_oen_q;

  // -------------------------------------------------------------------------
  // Status block signals
  // -------------------------------------------------------------------------
  reg al_q, rxack_q, tip_q, irq_flag_q;

  // handshake wires
  wire core_ack_w = core_ack_q;
  wire core_rxd_w = sda_sample_q;
  wire i2c_al_w   = al_bit_q;
  wire i2c_busy_w = busy_q;

  wire [7:0] s_rx_w    = sr_q;                  // received byte (shift register)
  wire [7:0] s_status_w = {rxack_q, busy_q, al_q, 3'b000, tip_q, irq_flag_q};

  // =========================================================================
  // Byte controller
  // =========================================================================
  // shift register
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      sr_q <= 8'h0;
    else if (ld_r)
      sr_q <= r_tx_q;
    else if (shift_r)
      sr_q <= {sr_q[6:0], core_rxd_w};
  end

  // bit counter
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      dcnt_q <= 3'h0;
    else if (ld_r)
      dcnt_q <= 3'h7;
    else if (shift_r)
      dcnt_q <= dcnt_q - 3'h1;
  end

  // byte controller FSM
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      b_state_q       <= B_IDLE;
      core_cmd_r      <= I2C_CMD_NOP;
      core_txd_r      <= 1'b0;
      shift_r         <= 1'b0;
      ld_r            <= 1'b0;
      byte_cmd_ack_r  <= 1'b0;
      ack_out_r       <= 1'b0;
    end else if (i2c_al_w) begin
      b_state_q       <= B_IDLE;
      core_cmd_r      <= I2C_CMD_NOP;
      core_txd_r      <= 1'b0;
      shift_r         <= 1'b0;
      ld_r            <= 1'b0;
      byte_cmd_ack_r  <= 1'b0;
      ack_out_r       <= 1'b0;
    end else begin
      core_txd_r      <= sr_q[7];
      shift_r         <= 1'b0;
      ld_r            <= 1'b0;
      byte_cmd_ack_r  <= 1'b0;

      case (b_state_q)
        B_IDLE: begin
          if (go_w) begin
            if (sta_w) begin
              b_state_q  <= B_START;
              core_cmd_r <= I2C_CMD_START;
            end else if (rd_w) begin
              b_state_q  <= B_READ;
              core_cmd_r <= I2C_CMD_READ;
            end else if (wr_w) begin
              b_state_q  <= B_WRITE;
              core_cmd_r <= I2C_CMD_WRITE;
            end else begin
              b_state_q  <= B_STOP;
              core_cmd_r <= I2C_CMD_STOP;
            end
            ld_r <= 1'b1;
          end
        end

        B_START: begin
          if (core_ack_w) begin
            if (rd_w) begin
              b_state_q  <= B_READ;
              core_cmd_r <= I2C_CMD_READ;
            end else begin
              b_state_q  <= B_WRITE;
              core_cmd_r <= I2C_CMD_WRITE;
            end
            ld_r <= 1'b1;
          end
        end

        B_WRITE: begin
          if (core_ack_w) begin
            if (cnt_done_w) begin
              b_state_q  <= B_ACK;
              core_cmd_r <= I2C_CMD_READ;    // read the slave ACK bit
            end else begin
              b_state_q  <= B_WRITE;
              core_cmd_r <= I2C_CMD_WRITE;
              shift_r    <= 1'b1;
            end
          end
        end

        B_READ: begin
          if (core_ack_w) begin
            if (cnt_done_w) begin
              b_state_q  <= B_ACK;
              core_cmd_r <= I2C_CMD_WRITE;   // send our ACK/NACK bit
            end else begin
              b_state_q  <= B_READ;
              core_cmd_r <= I2C_CMD_READ;
            end
            shift_r    <= 1'b1;
            core_txd_r <= ack_in_w;
          end
        end

        B_ACK: begin
          if (core_ack_w) begin
            if (sto_w) begin
              b_state_q  <= B_STOP;
              core_cmd_r <= I2C_CMD_STOP;
            end else begin
              b_state_q      <= B_IDLE;
              core_cmd_r     <= I2C_CMD_NOP;
              byte_cmd_ack_r <= 1'b1;
            end
            ack_out_r  <= core_rxd_w;
            core_txd_r <= 1'b1;
          end else begin
            core_txd_r <= ack_in_w;
          end
        end

        B_STOP: begin
          if (core_ack_w) begin
            b_state_q      <= B_IDLE;
            core_cmd_r     <= I2C_CMD_NOP;
            byte_cmd_ack_r <= 1'b1;
          end
        end

        default: begin
          b_state_q  <= B_IDLE;
          core_cmd_r <= I2C_CMD_NOP;
        end
      endcase
    end
  end

  // =========================================================================
  // Bit controller
  // =========================================================================
  // input synchronizers
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      cSCL_q <= 2'b11;
      cSDA_q <= 2'b11;
    end else begin
      cSCL_q <= {cSCL_q[0], scl_pad_i};
      cSDA_q <= {cSDA_q[0], sda_pad_i};
    end
  end

  // glitch filter clock
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      filter_cnt_q <= 14'h0;
    else if (!s_core_en_w)
      filter_cnt_q <= 14'h0;
    else if (~|filter_cnt_q)
      filter_cnt_q <= {2'b00, r_pre_q[15:2]};   // reload to PRE>>2
    else
      filter_cnt_q <= filter_cnt_q - 14'd1;
  end

  // filter shift registers (majority of 3)
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      fSCL_q <= 3'b111;
      fSDA_q <= 3'b111;
    end else if (~|filter_cnt_q) begin
      fSCL_q <= {fSCL_q[1:0], cSCL_q[1]};
      fSDA_q <= {fSDA_q[1:0], cSDA_q[1]};
    end
  end

  // filtered levels and delayed copies
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      sSCL_q <= 1'b1;
      sSDA_q <= 1'b1;
      dSCL_q <= 1'b1;
      dSDA_q <= 1'b1;
    end else begin
      sSCL_q <= (fSCL_q[2] & fSCL_q[1]) | (fSCL_q[1] & fSCL_q[0]) |
                (fSCL_q[2] & fSCL_q[0]);
      sSDA_q <= (fSDA_q[2] & fSDA_q[1]) | (fSDA_q[1] & fSDA_q[0]) |
                (fSDA_q[2] & fSDA_q[0]);
      dSCL_q <= sSCL_q;
      dSDA_q <= sSDA_q;
    end
  end

  // start / stop condition detection
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      sta_cond_q <= 1'b0;
      sto_cond_q <= 1'b0;
    end else begin
      sta_cond_q <= ~sSDA_q & dSDA_q & sSCL_q;   // SDA falls while SCL high
      sto_cond_q <= sSDA_q & ~dSDA_q & sSCL_q;   // SDA rises while SCL high
    end
  end

  // bus busy
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      busy_q <= 1'b0;
    else
      busy_q <= (sta_cond_q | busy_q) & ~sto_cond_q;
  end

  // delayed output enable (edge detect for slave_wait)
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      dscl_oen_q <= 1'b1;
    else
      dscl_oen_q <= scl_oen_q;
  end

  // clock stretching: we release SCL but the slave holds it low
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      slave_wait_q <= 1'b0;
    else
      slave_wait_q <= (scl_oen_q & ~dscl_oen_q & ~sSCL_q) | (slave_wait_q & ~sSCL_q);
  end

  // timer: one clk_en tick every (PRE+1) clocks; resync on external SCL low
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      cnt_q    <= 16'h0;
      clk_en_q <= 1'b1;
    end else if (~|cnt_q || !s_core_en_w || scl_sync_w) begin
      cnt_q    <= r_pre_q;
      clk_en_q <= 1'b1;
    end else if (slave_wait_q) begin
      cnt_q    <= cnt_q;
      clk_en_q <= 1'b0;
    end else begin
      cnt_q    <= cnt_q - 16'd1;
      clk_en_q <= 1'b0;
    end
  end

  // remember whether the current command is a STOP (suppresses AL on it)
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      cmd_stop_q <= 1'b0;
    else if (clk_en_q)
      cmd_stop_q <= (core_cmd_r == I2C_CMD_STOP);
  end

  // arbitration lost
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      al_bit_q <= 1'b0;
    else
      al_bit_q <= (sda_chk_q & ~sSDA_q & sda_oen_q) |
                  ((bit_state_q != BIT_IDLE) & sto_cond_q & ~cmd_stop_q);
  end

  // sample SDA on the rising edge of SCL
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      sda_sample_q <= 1'b1;
    else if (sSCL_q & ~dSCL_q)
      sda_sample_q <= sSDA_q;
  end

  // bit controller FSM (one state per clk_en tick)
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      bit_state_q <= BIT_IDLE;
      scl_oen_q   <= 1'b1;                  // released
      sda_oen_q   <= 1'b1;                  // released
      sda_chk_q   <= 1'b0;
      core_ack_q  <= 1'b0;
    end else if (al_bit_q) begin
      bit_state_q <= BIT_IDLE;
      scl_oen_q   <= 1'b1;
      sda_oen_q   <= 1'b1;
      sda_chk_q   <= 1'b0;
      core_ack_q  <= 1'b0;
    end else begin
      core_ack_q <= 1'b0;                   // default: single-cycle pulse
      if (clk_en_q) begin
        case (bit_state_q)
          BIT_IDLE: begin
            case (core_cmd_r)
              I2C_CMD_START: bit_state_q <= BIT_START_A;
              I2C_CMD_STOP:  bit_state_q <= BIT_STOP_A;
              I2C_CMD_WRITE: bit_state_q <= BIT_WR_A;
              I2C_CMD_READ:  bit_state_q <= BIT_RD_A;
              default:       bit_state_q <= BIT_IDLE;
            endcase
          end
          BIT_START_A: begin
            bit_state_q <= BIT_START_B;
            sda_oen_q   <= 1'b1;            // SDA high
          end
          BIT_START_B: begin
            bit_state_q <= BIT_START_C;
            scl_oen_q   <= 1'b1;            // SCL high
          end
          BIT_START_C: begin
            bit_state_q <= BIT_START_D;
            sda_oen_q   <= 1'b0;            // SDA low -> START condition
          end
          BIT_START_D: begin
            bit_state_q <= BIT_START_E;
          end
          BIT_START_E: begin
            bit_state_q <= BIT_IDLE;
            core_ack_q  <= 1'b1;
            scl_oen_q   <= 1'b0;            // SCL low
            sda_oen_q   <= 1'b0;            // keep SDA low
          end
          BIT_STOP_A: begin
            bit_state_q <= BIT_STOP_B;
            sda_oen_q   <= 1'b0;            // SDA low
          end
          BIT_STOP_B: begin
            bit_state_q <= BIT_STOP_C;
            scl_oen_q   <= 1'b1;            // SCL high
          end
          BIT_STOP_C: begin
            bit_state_q <= BIT_STOP_D;
          end
          BIT_STOP_D: begin
            bit_state_q <= BIT_IDLE;
            core_ack_q  <= 1'b1;
            scl_oen_q   <= 1'b1;            // SCL high
            sda_oen_q   <= 1'b1;            // SDA high -> STOP condition
          end
          BIT_RD_A: begin
            bit_state_q <= BIT_RD_B;
            scl_oen_q   <= 1'b0;            // SCL low
            sda_oen_q   <= 1'b1;            // release SDA
          end
          BIT_RD_B: begin
            bit_state_q <= BIT_RD_C;
            scl_oen_q   <= 1'b1;            // SCL high (dout samples here)
          end
          BIT_RD_C: begin
            bit_state_q <= BIT_RD_D;
          end
          BIT_RD_D: begin
            bit_state_q <= BIT_IDLE;
            core_ack_q  <= 1'b1;
            scl_oen_q   <= 1'b0;            // SCL low
            sda_oen_q   <= 1'b1;            // keep SDA released
          end
          BIT_WR_A: begin
            bit_state_q <= BIT_WR_B;
            scl_oen_q   <= 1'b0;            // SCL low
            sda_oen_q   <= core_txd_r;      // drive SDA with the bit
          end
          BIT_WR_B: begin
            bit_state_q <= BIT_WR_C;
            scl_oen_q   <= 1'b1;            // SCL high
            sda_oen_q   <= core_txd_r;
          end
          BIT_WR_C: begin
            bit_state_q <= BIT_WR_D;
            sda_chk_q   <= 1'b1;            // arbitration check phase
            sda_oen_q   <= core_txd_r;
          end
          BIT_WR_D: begin
            bit_state_q <= BIT_IDLE;
            core_ack_q  <= 1'b1;
            scl_oen_q   <= 1'b0;            // SCL low
            sda_oen_q   <= core_txd_r;
            sda_chk_q   <= 1'b0;
          end
          default: bit_state_q <= BIT_IDLE;
        endcase
      end
    end
  end

  // =========================================================================
  // Status block + interrupt
  // =========================================================================
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      al_q       <= 1'b0;
      rxack_q    <= 1'b0;
      tip_q      <= 1'b0;
      irq_flag_q <= 1'b0;
    end else begin
      al_q       <= i2c_al_w | (al_q & ~sta_w);
      rxack_q    <= ack_out_r;
      tip_q      <= (rd_w | wr_w);
      irq_flag_q <= (s_done_w | i2c_al_w | irq_flag_q) & ~iack_w;
    end
  end

  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn)
      interrupt_o <= 1'b0;
    else
      interrupt_o <= irq_flag_q && s_ien_w;
  end

  // =========================================================================
  // APB register interface
  // =========================================================================
  always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
      r_pre_q  <= 16'h0;
      r_ctrl_q <= 8'h0;
      r_tx_q   <= 8'h0;
      r_cmd_q  <= 8'h0;
    end else if (PSEL && PENABLE && PWRITE) begin
      if (s_done_w | i2c_al_w)
        r_cmd_q[7:4] <= 4'h0;               // clear command bits
      r_cmd_q[2:1] <= 2'b0;
      r_cmd_q[0]   <= 1'b0;                 // clear IRQ_ACK bit
      case (a)
        4'h0: r_pre_q  <= PWDATA[15:0];     // PRE
        4'h1: r_ctrl_q <= PWDATA[7:0];      // CTRL
        4'h4: r_tx_q   <= PWDATA[7:0];      // TX
        4'h5: begin
          if (s_core_en_w)
            r_cmd_q <= PWDATA[7:0];         // CMD
        end
        default: ;
      endcase
    end else begin
      if (s_done_w | i2c_al_w)
        r_cmd_q[7:4] <= 4'h0;
      r_cmd_q[2:1] <= 2'b0;
      r_cmd_q[0]   <= 1'b0;
    end
  end

  // read mux
  always @(*) begin
    case (a)
      4'h0: PRDATA = {16'h0, r_pre_q};
      4'h1: PRDATA = {24'h0, r_ctrl_q};
      4'h2: PRDATA = {24'h0, s_rx_w};
      4'h3: PRDATA = {24'h0, s_status_w};
      4'h4: PRDATA = {24'h0, r_tx_q};
      4'h5: PRDATA = {24'h0, r_cmd_q};
      default: PRDATA = 32'h0;
    endcase
  end

  assign PREADY  = 1'b1;
  assign PSLVERR = 1'b0;

  // open-drain pads
  assign scl_pad_o   = 1'b0;
  assign sda_pad_o   = 1'b0;
  assign scl_padoen_o = scl_oen_q;
  assign sda_padoen_o = sda_oen_q;

endmodule
