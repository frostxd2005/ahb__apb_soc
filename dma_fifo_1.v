`timescale 1ns/1ps

//=============================================================================
// dma_fifo_1.v
//------------------------------------------------------------------------------
// Synchronous buffer FIFO of the 4-channel DMA controller (default 4 x 32).
// From varun23-2004/-4-Channel-AHB-APB-DMA-Controller-RTL-Design- (unchanged
// logic, timescale added). Decouples the read and write phases of the AHB
// master.
//=============================================================================

module dma_fifo_1 #(
  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 2
)(
  input  wire                 clk,
  input  wire                 rst_n,
  input  wire                 w_en,
  input  wire                 r_en,
  input  wire [DATA_WIDTH-1:0] data_in,
  output wire [DATA_WIDTH-1:0] data_out,
  output wire                 full,
  output wire                 empty
);

  reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
  reg [ADDR_WIDTH-1:0] wr_ptr;
  reg [ADDR_WIDTH-1:0] rd_ptr;
  reg [ADDR_WIDTH:0]   count;

  // ---- write ----
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= 0;
    end else begin
      if (w_en && !full) begin
        mem[wr_ptr] <= data_in;
        wr_ptr <= wr_ptr + 1;
      end
    end
  end

  // ---- read ----
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_ptr <= 0;
    end else begin
      if (r_en && !empty) begin
        rd_ptr <= rd_ptr + 1;
      end
    end
  end

  assign data_out = mem[rd_ptr];

  // ---- occupancy counter ----
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      count <= 0;
    end else begin
      if ((w_en && !full) && (r_en && !empty)) begin
        count <= count;
      end else if (w_en && !full) begin
        count <= count + 1;
      end else if (r_en && !empty) begin
        count <= count - 1;
      end
    end
  end

  // ---- status flags ----
  assign full  = (count == (1 << ADDR_WIDTH));
  assign empty = (count == 0);

endmodule
