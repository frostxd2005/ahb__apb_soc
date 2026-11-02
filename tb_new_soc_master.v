`timescale 1ns/1ps

// Master, self-checking integration regression for new_soc_v2.
// Runs SRAM boot (00), flash XIP (01), and flash-to-SRAM copy (10) in one
// simulation using the same position-independent firmware image.
module tb_new_soc_master;
  parameter integer CLK_PERIOD_NS = 10;
  parameter integer UART_DIV = 4;
  parameter integer BIT_NS = CLK_PERIOD_NS * 16 * UART_DIV;
  parameter integer MAX_CYCLES = 1200000; // 12 ms per boot mode

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg [1:0] boot_mode = 2'b00;
  reg uart_rx_i = 1'b1;
  wire uart_tx_o;
  wire uart_int_o;
  wire spi_clk_o, spi_csn0_o, spi_sdo0_o;
  wire [1:0] spi_events_o;
  wire i2c_scl_o, i2c_scl_oen_o, i2c_sda_o, i2c_sda_oen_o;
  wire i2c_irq_o;
  wire i2s_sck_o, i2s_ws_o, i2s_sd_o;
  wire wdg_reset_o, core_sleep_o, dbg_halted_o;
  wire [63:0] dbg_mcycle_o;

  // External SPI slave loopback: the test still traverses the physical MISO
  // input and MOSI output pins, and the TB checks CS/clock/data activity.
  wire spi_sdi_i = spi_sdo0_o;

  // Behavioral open-drain I2C slave.  The SoC drives low when OEN=0; the TB
  // also pulls SDA low for ACK and returns 0xC3 during an address-read.
  reg i2c_slave_sda_low = 1'b0;
  reg i2c_slave_scl_low = 1'b0;
  wire i2c_scl_i = i2c_scl_oen_o && !i2c_slave_scl_low;
  wire i2c_sda_i = i2c_sda_oen_o && !i2c_slave_sda_low;

  // I2S external RX source. The master test has a loopback-free RX phase;
  // holding SD high for a complete frame must be received as 0xFFFF_FFFF.
  wire i2s_sd_i = 1'b1;

  always #(CLK_PERIOD_NS/2) clk = ~clk;

  new_soc_top #(
    .BOOT_ADDR(32'h0000_0000),
    .MTVEC_ADDR(32'h0000_1000),
    .SRAM_HEX("sram.hex"),
    .FLASH_HEX("")
  ) dut (
    .clk_i(clk), .rst_ni(rst_n), .boot_mode_i(boot_mode),
    .uart_tx_o(uart_tx_o), .uart_rx_i(uart_rx_i), .uart_int_o(uart_int_o),
    .spi_clk_o(spi_clk_o), .spi_csn0_o(spi_csn0_o), .spi_sdo0_o(spi_sdo0_o),
    .spi_sdi0_i(spi_sdi_i), .spi_events_o(spi_events_o),
    .i2c_scl_i(i2c_scl_i), .i2c_scl_o(i2c_scl_o), .i2c_scl_oen_o(i2c_scl_oen_o),
    .i2c_sda_i(i2c_sda_i), .i2c_sda_o(i2c_sda_o), .i2c_sda_oen_o(i2c_sda_oen_o),
    .i2c_irq_o(i2c_irq_o),
    .i2s_sck_o(i2s_sck_o), .i2s_ws_o(i2s_ws_o), .i2s_sd_o(i2s_sd_o), .i2s_sd_i(i2s_sd_i),
    .wdg_reset_o(wdg_reset_o), .core_sleep_o(core_sleep_o),
    .dbg_halted_o(dbg_halted_o), .dbg_mcycle_o(dbg_mcycle_o)
  );

  // -------------------------------------------------------------------------
  // UART RX driver and UART TX decoder
  // -------------------------------------------------------------------------
  task send_uart_byte;
    input [7:0] value;
    integer b;
    begin
      uart_rx_i = 1'b0; #(BIT_NS);
      for (b = 0; b < 8; b = b + 1) begin
        uart_rx_i = value[b]; #(BIT_NS);
      end
      uart_rx_i = 1'b1; #(BIT_NS);
    end
  endtask

  reg [7:0] uart_byte = 8'h00;
  reg [31:0] uart_recent = 32'h0;
  integer uart_char_count = 0;
  reg ready_seen = 1'b0;
  reg rx_stim_sent = 1'b0;
  reg pass_seen = 1'b0;
  reg fail_seen = 1'b0;
  integer bit_index;
  integer tx_dma_seen = 0;
  reg [7:0] expected_tx [0:3];
  integer expected_tx_index = 0;

  initial begin
    expected_tx[0] = 8'h31;
    expected_tx[1] = 8'h32;
    expected_tx[2] = 8'h33;
    expected_tx[3] = 8'h34;
    forever begin
      @(negedge uart_tx_o);
      if (rst_n) begin
        #(BIT_NS + BIT_NS/2);
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
          if (bit_index != 0) #(BIT_NS);
          uart_byte[bit_index] = uart_tx_o;
        end
        #(BIT_NS); // stop bit
        uart_char_count = uart_char_count + 1;
        uart_recent = {uart_recent[23:0], uart_byte};
        if (uart_byte == 8'h52) ready_seen = 1'b1; // R: firmware armed UART RX DMA
        if (uart_byte == expected_tx[expected_tx_index] && expected_tx_index < 4) begin
          expected_tx_index = expected_tx_index + 1;
          tx_dma_seen = tx_dma_seen + 1;
        end
        if (uart_recent == {8'h50,8'h41,8'h53,8'h53}) pass_seen = 1'b1;
        if (uart_recent == {8'h46,8'h41,8'h49,8'h4C}) fail_seen = 1'b1;
      end
    end
  end

  // Send fresh UART RX bytes only after firmware has configured DMA0 stream 0.
  initial begin
    forever begin
      wait (ready_seen && !rx_stim_sent);
      rx_stim_sent = 1'b1;
      repeat (12) @(posedge clk);
      send_uart_byte(8'hA5);
      send_uart_byte(8'h5A);
      send_uart_byte(8'hC3);
      send_uart_byte(8'h3C);
    end
  end

  // -------------------------------------------------------------------------
  // I2C slave bit/byte model
  // -------------------------------------------------------------------------
  localparam [2:0] I2C_IDLE=3'd0, I2C_RX=3'd1, I2C_ACK_SETUP=3'd2,
                   I2C_ACK_HOLD=3'd3, I2C_ACK_RELEASE=3'd4,
                   I2C_TX=3'd5, I2C_MASTER_ACK=3'd6;
  reg [2:0] i2c_state = I2C_IDLE;
  integer i2c_bit_count = 0;
  reg [7:0] i2c_shift = 8'h00;
  reg [7:0] i2c_tx_byte = 8'hC3;
  reg i2c_first_byte = 1'b1;
  reg i2c_read_direction = 1'b0;
  integer i2c_address_w_count = 0;
  integer i2c_address_r_count = 0;
  integer i2c_data_w_count = 0;
  reg [7:0] i2c_last_write = 8'h00;
  integer i2c_read_bits = 0;

  always @(negedge rst_n) begin
    i2c_slave_sda_low = 1'b0;
    i2c_slave_scl_low = 1'b0;
    i2c_state = I2C_IDLE;
    i2c_bit_count = 0;
    i2c_shift = 8'h00;
    i2c_tx_byte = 8'hC3;
    i2c_first_byte = 1'b1;
    i2c_read_direction = 1'b0;
    i2c_address_w_count = 0;
    i2c_address_r_count = 0;
    i2c_data_w_count = 0;
    i2c_last_write = 8'h00;
    i2c_read_bits = 0;
  end

  // START/STOP are SDA transitions while SCL is high.
  always @(negedge i2c_sda_i) begin
    if (rst_n && i2c_scl_i) begin
      i2c_state = I2C_RX;
      i2c_bit_count = 0;
      i2c_shift = 8'h00;
      i2c_first_byte = 1'b1;
      i2c_read_direction = 1'b0;
      i2c_slave_sda_low = 1'b0;
    end
  end
  always @(posedge i2c_sda_i) begin
    if (rst_n && i2c_scl_i) begin
      i2c_state = I2C_IDLE;
      i2c_slave_sda_low = 1'b0;
      i2c_bit_count = 0;
    end
  end

  always @(posedge i2c_scl_i) begin
    if (rst_n) begin
      case (i2c_state)
        I2C_RX: begin
          i2c_shift = {i2c_shift[6:0], i2c_sda_i};
          if (i2c_bit_count == 7) begin
            i2c_bit_count = 0;
            i2c_state = I2C_ACK_SETUP;
            if (i2c_first_byte) begin
              i2c_first_byte = 1'b0;
              i2c_read_direction = i2c_sda_i;
              if (i2c_sda_i) i2c_address_r_count = i2c_address_r_count + 1;
              else            i2c_address_w_count = i2c_address_w_count + 1;
            end else begin
              i2c_last_write = i2c_shift;
              i2c_data_w_count = i2c_data_w_count + 1;
            end
          end else begin
            i2c_bit_count = i2c_bit_count + 1;
          end
        end
        I2C_ACK_HOLD: begin
          i2c_state = I2C_ACK_RELEASE;
        end
        I2C_TX: begin
          if (i2c_bit_count == 7) begin
            i2c_bit_count = 0;
            i2c_state = I2C_MASTER_ACK;
          end else begin
            i2c_bit_count = i2c_bit_count + 1;
          end
        end
        I2C_MASTER_ACK: begin
          i2c_state = I2C_IDLE;
          i2c_slave_sda_low = 1'b0;
        end
        default: ;
      endcase
    end
  end

  always @(negedge i2c_scl_i) begin
    if (rst_n) begin
      case (i2c_state)
        I2C_ACK_SETUP: begin
          i2c_slave_sda_low = 1'b1; // ACK low for ninth clock
          i2c_state = I2C_ACK_HOLD;
        end
        I2C_ACK_RELEASE: begin
          if (i2c_read_direction) begin
            i2c_state = I2C_TX;
            i2c_bit_count = 0;
            i2c_slave_sda_low = !i2c_tx_byte[7];
          end else begin
            i2c_state = I2C_RX;
            i2c_bit_count = 0;
            i2c_slave_sda_low = 1'b0;
          end
        end
        I2C_TX: begin
          i2c_slave_sda_low = !i2c_tx_byte[7-i2c_bit_count];
        end
        I2C_MASTER_ACK: begin
          i2c_slave_sda_low = 1'b0;
        end
        default: ;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Monitors for pin activity and core interrupt acknowledgement
  // -------------------------------------------------------------------------
  integer irq_ack_count [0:7];
  integer irq_unexpected_count = 0;
  integer spi_sck_edges = 0;
  integer spi_cs_count = 0;
  integer i2s_sck_edges = 0;
  integer i2s_ws_edges = 0;
  reg wdg_seen = 1'b0;
  integer j;

  always @(posedge clk) begin
    if (!rst_n) begin
      for (j = 0; j < 8; j = j + 1) irq_ack_count[j] = 0;
      irq_unexpected_count = 0;
      spi_sck_edges = 0;
      spi_cs_count = 0;
      i2s_sck_edges = 0;
      i2s_ws_edges = 0;
      wdg_seen = 1'b0;
    end else begin
      if (dut.irq_ack_w) begin
        if ((dut.irq_id_w >= 16) && (dut.irq_id_w < 24))
          irq_ack_count[dut.irq_id_w - 16] = irq_ack_count[dut.irq_id_w - 16] + 1;
        else
          irq_unexpected_count = irq_unexpected_count + 1;
      end
      if (wdg_reset_o) wdg_seen = 1'b1;
    end
  end

  reg [7:0] spi_capture = 8'h00;
  integer spi_bit_index = 0;
  integer spi_words_seen = 0;
  integer spi_bad_words = 0;
  always @(posedge spi_clk_o) if (rst_n) spi_sck_edges = spi_sck_edges + 1;
  always @(negedge spi_csn0_o) if (rst_n) begin
    spi_cs_count = spi_cs_count + 1;
    spi_bit_index = 0;
    spi_capture = 8'h00;
  end
  always @(posedge spi_clk_o) begin
    if (rst_n && !spi_csn0_o) begin
      spi_capture = {spi_capture[6:0], spi_sdi_i};
      spi_bit_index = spi_bit_index + 1;
      if (spi_bit_index == 8) begin
        spi_words_seen = spi_words_seen + 1;
        if (spi_capture !== 8'hA5) spi_bad_words = spi_bad_words + 1;
        spi_bit_index = 0;
      end
    end
  end

  reg [31:0] i2s_tx_shift = 32'h0;
  reg i2s_tx_pattern_seen = 1'b0;
  always @(negedge rst_n) begin
    spi_capture = 8'h00;
    spi_bit_index = 0;
    spi_words_seen = 0;
    spi_bad_words = 0;
    i2s_tx_shift = 32'h0;
    i2s_tx_pattern_seen = 1'b0;
  end
  always @(posedge i2s_sck_o) begin
    if (rst_n) begin
      i2s_sck_edges = i2s_sck_edges + 1;
      i2s_tx_shift = {i2s_tx_shift[30:0], i2s_sd_o};
      if (i2s_tx_shift == 32'h89ABCDEF) i2s_tx_pattern_seen = 1'b1;
    end
  end
  always @(posedge i2s_ws_o) if (rst_n) i2s_ws_edges = i2s_ws_edges + 1;

  // -------------------------------------------------------------------------
  // Multi-boot regression driver and watchdog
  // -------------------------------------------------------------------------
  integer mode_index;
  integer cycles;
  integer irq_i;
  integer dma_verify_i;
  reg timed_out = 1'b0;
  always @(posedge clk) begin
    if (!rst_n) begin
      cycles = 0;
      timed_out = 1'b0;
    end else begin
      cycles = cycles + 1;
      if (cycles >= MAX_CYCLES) timed_out = 1'b1;
    end
  end

  initial begin
    uart_rx_i = 1'b1;
    ready_seen = 1'b0;
    rx_stim_sent = 1'b0;
    pass_seen = 1'b0;
    fail_seen = 1'b0;
    uart_recent = 32'h0;
    tx_dma_seen = 0;
    expected_tx_index = 0;
    boot_mode = 2'b00;
    rst_n = 1'b0;

    for (mode_index = 0; mode_index < 3; mode_index = mode_index + 1) begin
      boot_mode = mode_index[1:0];
      ready_seen = 1'b0;
      rx_stim_sent = 1'b0;
      pass_seen = 1'b0;
      fail_seen = 1'b0;
      uart_recent = 32'h0;
      tx_dma_seen = 0;
      expected_tx_index = 0;
      uart_char_count = 0;
      rst_n = 1'b0;
      repeat (8) @(posedge clk);
      rst_n = 1'b1;
      $display("==================================================");
      $display(" MASTER TEST START: boot_mode=%0d (0=SRAM,1=XIP,2=copy)", mode_index);
      $display("==================================================");
      wait (pass_seen || fail_seen || timed_out);
      if (timed_out) begin
        $display("MASTER TEST FAILED: timeout in boot mode %0d; UART chars=%0d", mode_index, uart_char_count);
        $display("  PC=%08h MEPC=%08h MCAUSE=%02h MTVEC=%06h MIE=%b",
                 dut.u_cpu.core_i.pc_id, dut.u_cpu.core_i.mepc,
                 dut.u_cpu.core_i.cs_registers_i.mcause_q,
                 dut.u_cpu.core_i.mtvec,
                 dut.u_cpu.core_i.cs_registers_i.mstatus_q.mie);
        $display("  DMA0 CFG0..7=%08h %08h %08h %08h %08h %08h %08h %08h",
                 dut.u_dma0.u_apb_slave.reg_config[0], dut.u_dma0.u_apb_slave.reg_config[1],
                 dut.u_dma0.u_apb_slave.reg_config[2], dut.u_dma0.u_apb_slave.reg_config[3],
                 dut.u_dma0.u_apb_slave.reg_config[4], dut.u_dma0.u_apb_slave.reg_config[5],
                 dut.u_dma0.u_apb_slave.reg_config[6], dut.u_dma0.u_apb_slave.reg_config[7]);
        $display("  DMA1 CFG0..7=%08h %08h %08h %08h %08h %08h %08h %08h",
                 dut.u_dma1.u_apb_slave.reg_config[0], dut.u_dma1.u_apb_slave.reg_config[1],
                 dut.u_dma1.u_apb_slave.reg_config[2], dut.u_dma1.u_apb_slave.reg_config[3],
                 dut.u_dma1.u_apb_slave.reg_config[4], dut.u_dma1.u_apb_slave.reg_config[5],
                 dut.u_dma1.u_apb_slave.reg_config[6], dut.u_dma1.u_apb_slave.reg_config[7]);
        $display("  DMA states=%0d/%0d req=%02h/%02h req_lines=%02h irq=%b/%b",
                 dut.u_dma0.u_ahb_master.state_q, dut.u_dma1.u_ahb_master.state_q,
                 dut.u_dma0.w_req, dut.u_dma1.w_req, dut.dma_req_w, dut.dma0_irq_w, dut.dma1_irq_w);
        $display("  DMA0 src=%08h dst=%08h count=%08h haddr=%08h trans=%b grant=%b ready=%b busreq=%b",
                 dut.u_dma0.u_ahb_master.current_src_q, dut.u_dma0.u_ahb_master.current_dest_q,
                 dut.u_dma0.u_ahb_master.current_count_q, dut.dma0_haddr, dut.dma0_htrans,
                 dut.dma0_hgrant, dut.dma0_hready, dut.dma0_hbusreq);
        $display("  mem grant=%b ready=%b sel=%b sready=%b dsel=%b grantq=%0d CPU req=%b addr=%08h rvalid=%b SRAM_BSTATE=%0d bsel=%b bhreadyi=%b",
                 dut.mf_hgrant, dut.mf_hready, dut.mf_s_hsel, dut.mf_s_hready,
                 dut.u_mem_fabric.d_sel_q, dut.u_mem_fabric.grant_q,
                 dut.data_req, dut.data_addr, dut.data_rvalid,
                 dut.u_sram.b_state_q, dut.mf_s_hsel[0], dut.mf_hready);
        $display("  core pc_if=%08h pc_ex=%08h instr_req=%b gnt=%b rvalid=%b instr_addr=%08h",
                 dut.u_cpu.core_i.pc_if, dut.u_cpu.core_i.pc_ex,
                 dut.instr_req, dut.instr_gnt, dut.instr_rvalid, dut.instr_addr);
        $display("  UART ready=%b sent=%b rx=%b rxstate=%0d rxd=%b lsr=%02h ier=%02h dll=%02h int=%b",
                 ready_seen, rx_stim_sent, uart_rx_i, dut.u_uart.rx_state_q,
                 dut.u_uart.rxd_q, dut.u_uart.lsr_q, dut.u_uart.ier_q,
                 dut.u_uart.dll_q, dut.uart_int_w);
        $display("  PF grant=%b ready=%b sel=%b sready=%b dsel=%b APB state=%0d psel=%h pen=%b apbready=%b DMA0 steer sel=%b p_req=%b p_hsel=%b p_trans=%b p_addr=%08h p_hready=%b",
                 dut.pf_hgrant, dut.pf_hready, dut.pf_s_hsel, dut.pf_s_hready,
                 dut.u_periph_fabric.d_sel_q, dut.u_apb.state_q,
                 dut.p_psel, dut.p_penable, dut.apb_hready,
                 dut.u_dma0_steer.sel_q, dut.dma0_p_hbusreq, dut.dma0_p_hbusreq,
                 dut.dma0_p_htrans, dut.dma0_p_haddr, dut.pf_hready);
        $fatal(1, "master regression failure");
      end
      if (fail_seen) begin
        $display("MASTER TEST FAILED: firmware reported FAIL in boot mode %0d", mode_index);
        $display("  PC=%08h MEPC=%08h MCAUSE=%02h SPI_RXBUF=%08h I2C_RXBUF=%08h I2S_RXBUF=%08h",
                 dut.u_cpu.core_i.pc_id, dut.u_cpu.core_i.mepc,
                 dut.u_cpu.core_i.cs_registers_i.mcause_q,
                 dut.u_sram.mem[32'h14180 >> 2], dut.u_sram.mem[32'h141C0 >> 2],
                 dut.u_sram.mem[32'h14200 >> 2]);
        $display("  SPI RX FIFO0=%08h FIFO1=%08h DMA0 read_cap=%08h memwrite=%08h",
                 dut.u_spi.rx_fifo[0], dut.u_spi.rx_fifo[1],
                 dut.u_dma0.u_ahb_master.read_cap_q,
                 dut.u_sram.mem[32'h14180 >> 2]);
        $display("  SPI state=%0d sck=%b csn=%b rxempty=%b txempty=%b rxcount=%0d txcount=%0d events=%b DMA0 src=%08h dst=%08h count=%08h",
                 dut.u_spi.state_q, spi_clk_o, spi_csn0_o,
                 dut.u_spi.rx_empty_c, dut.u_spi.tx_empty_c,
                 dut.u_spi.rx_cnt_q, dut.u_spi.tx_cnt_q, spi_events_o,
                 dut.u_dma0.u_ahb_master.current_src_q,
                 dut.u_dma0.u_ahb_master.current_dest_q,
                 dut.u_dma0.u_ahb_master.current_count_q);
        $display("  I2S ctrl=%b div=%0d rxcount=%0d rxptr=%0d rxword0=%08h rxword1=%08h rxshift=%08h txcount=%0d bit=%0d sck=%b ws=%b sd_i=%b DMA0 ch=%0d cfg4=%08h",
                 {dut.u_i2s.rx_irq_en_q, dut.u_i2s.tx_irq_en_q, dut.u_i2s.loop_q,
                  dut.u_i2s.rx_en_q, dut.u_i2s.tx_en_q, dut.u_i2s.en_q},
                 dut.u_i2s.div_q, dut.u_i2s.rx_cnt_q, dut.u_i2s.rx_rd_ptr_q,
                 dut.u_i2s.rx_fifo[0], dut.u_i2s.rx_fifo[1], dut.u_i2s.rx_shift_q,
                 dut.u_i2s.tx_cnt_q, dut.u_i2s.bit_cnt_q, i2s_sck_o, i2s_ws_o, i2s_sd_i,
                 dut.u_dma0.u_ahb_master.active_ch_q, dut.u_dma0.u_apb_slave.reg_config[4]);
        $fatal(1, "master regression failure");
      end
      #1;
      if (tx_dma_seen != 4) begin
        $display("MASTER TEST FAILED: saw %0d/4 UART DMA TX bytes", tx_dma_seen);
        $fatal(1, "master regression failure");
      end
      if (dut.u_sram.mem[32'h14100 >> 2] !== 32'h000000A5 ||
          dut.u_sram.mem[32'h14104 >> 2] !== 32'h0000005A ||
          dut.u_sram.mem[32'h14108 >> 2] !== 32'h000000C3 ||
          dut.u_sram.mem[32'h1410C >> 2] !== 32'h0000003C ||
          dut.u_sram.mem[32'h14180 >> 2] !== 32'h000000A5 ||
          dut.u_sram.mem[32'h141C0 >> 2] !== 32'h000000C3 ||
          dut.u_sram.mem[32'h14200 >> 2] !== 32'hFFFFFFFF ||
          dut.u_sram.mem[32'h14220 >> 2] !== 32'h13579BDF) begin
        $display("MASTER TEST FAILED: physical SRAM DMA result buffer mismatch in boot mode %0d", mode_index);
        $display("  UART=%08h/%08h/%08h/%08h SPI=%08h I2C=%08h I2S=%08h WDG=%08h",
                 dut.u_sram.mem[32'h14100 >> 2], dut.u_sram.mem[32'h14104 >> 2],
                 dut.u_sram.mem[32'h14108 >> 2], dut.u_sram.mem[32'h1410C >> 2],
                 dut.u_sram.mem[32'h14180 >> 2], dut.u_sram.mem[32'h141C0 >> 2],
                 dut.u_sram.mem[32'h14200 >> 2], dut.u_sram.mem[32'h14220 >> 2]);
        $fatal(1, "master regression failure");
      end
      for (dma_verify_i = 0; dma_verify_i < 32; dma_verify_i = dma_verify_i + 1) begin
        if (dut.u_sram.mem[(32'h10000 >> 2) + dma_verify_i] !==
            dut.u_sram.mem[(32'h12000 >> 2) + dma_verify_i] ||
            dut.u_sram.mem[(32'h11000 >> 2) + dma_verify_i] !==
            dut.u_sram.mem[(32'h13000 >> 2) + dma_verify_i]) begin
          $display("MASTER TEST FAILED: DMA memory-copy mismatch at word %0d in boot mode %0d",
                   dma_verify_i, mode_index);
          $fatal(1, "master regression failure");
        end
      end
      for (irq_i = 0; irq_i < 7; irq_i = irq_i + 1) begin
        if (irq_ack_count[irq_i] == 0 ||
            dut.u_sram.mem[(32'h14000 >> 2) + irq_i] == 0) begin
          $display("MASTER TEST FAILED: CV32E40P did not service INTC source %0d in boot mode %0d",
                   irq_i, mode_index);
          $fatal(1, "master regression failure");
        end
      end
      if (irq_unexpected_count != 0) begin
        $display("MASTER TEST FAILED: unexpected interrupt claim count=%0d", irq_unexpected_count);
        $fatal(1, "master regression failure");
      end
      if (dut.u_sram.mem[32'h14040 >> 2] == 0 ||
          dut.u_sram.mem[32'h14044 >> 2] == 0 ||
          dut.u_sram.mem[32'h14048 >> 2] == 0) begin
        $display("MASTER TEST FAILED: ECALL/EBREAK/illegal-instruction trap counters missing");
        $fatal(1, "master regression failure");
      end
      if (spi_cs_count == 0 || spi_sck_edges < 8 ||
          spi_words_seen != 1 || spi_bad_words != 0) begin
        $display("MASTER TEST FAILED: SPI pin/loopback check failed (CS=%0d SCK=%0d words=%0d bad=%0d)",
                 spi_cs_count, spi_sck_edges, spi_words_seen, spi_bad_words);
        $fatal(1, "master regression failure");
      end
      if (i2s_sck_edges == 0 || i2s_ws_edges == 0 || !i2s_tx_pattern_seen) begin
        $display("MASTER TEST FAILED: I2S TX/RX pin check failed (SCK=%0d WS=%0d TXword=%b)",
                 i2s_sck_edges, i2s_ws_edges, i2s_tx_pattern_seen);
        $fatal(1, "master regression failure");
      end
      if (!wdg_seen) begin
        $display("MASTER TEST FAILED: watchdog reset request was not observed");
        $fatal(1, "master regression failure");
      end
      if (i2c_address_w_count == 0 || i2c_address_r_count == 0 ||
          i2c_data_w_count == 0 || i2c_last_write !== 8'h5A) begin
        $display("MASTER TEST FAILED: I2C transaction/data check failed (AW=%0d AR=%0d DW=%0d data=%02h)",
                 i2c_address_w_count, i2c_address_r_count, i2c_data_w_count, i2c_last_write);
        $fatal(1, "master regression failure");
      end
      $display("MASTER TEST PASS: boot_mode=%0d SRAM/XIP DMA, peripherals, IRQs, and traps checked", mode_index);
      if (mode_index != 2) begin
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
      end
    end
    $display("==================================================");
    $display(" NEW_SOC V2 MASTER REGRESSION PASSED");
    $display(" SRAM boot + flash XIP + flash-to-SRAM copy all passed");
    $display("==================================================");
    $finish;
  end
endmodule
