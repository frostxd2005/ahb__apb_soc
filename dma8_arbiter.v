`timescale 1ns/1ps

//=============================================================================
// dma8_arbiter.v
//-----------------------------------------------------------------------------
// Round-robin stream arbiter of the 8-stream DMA controller (the 8-channel
// counterpart of dma_arbiter).  Rotating pointer guarantees starvation-free
// stream selection; grant is one-hot, registered.
// Plain Verilog (IEEE 1364-2001).
//=============================================================================

module dma8_arbiter (
  input  wire       clk,
  input  wire       rst_n,
  input  wire [7:0] req,
  output reg  [7:0] grant
);

  reg [2:0] rotate_ptr_q;
  reg [7:0] next_grant;
  reg       found;

  integer i;
  integer j;

  // ---- decision matrix (rotating priority) ----
  always @(*) begin
    next_grant = 8'h00;
    found      = 1'b0;
    for (i = 0; i < 8; i = i + 1) begin
      j = (i + rotate_ptr_q) % 8;
      if (!found && req[j]) begin
        next_grant = 8'b0000_0001 << j;
        found      = 1'b1;
      end
    end
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) grant <= 8'h00;
    else        grant <= next_grant;
  end

  // ---- pointer update: continue after the granted stream ----
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rotate_ptr_q <= 3'd0;
    end else begin
      for (i = 0; i < 8; i = i + 1) begin
        if (next_grant[i]) rotate_ptr_q <= (i + 1) % 8;
      end
    end
  end

endmodule
