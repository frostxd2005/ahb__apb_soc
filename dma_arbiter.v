`timescale 1ns/1ps

//=============================================================================
// dma_arbiter.v
//------------------------------------------------------------------------------
// Round-robin channel arbiter of the 4-channel DMA controller. From
// varun23-2004/-4-Channel-AHB-APB-DMA-Controller-RTL-Design- (unchanged logic,
// timescale added). Rotating pointer guarantees starvation-free channel
// selection; grant is one-hot, registered.
//=============================================================================

module dma_arbiter #(
  parameter NUM_CHAN = 4
)(
  input  wire             clk,
  input  wire             rst_n,
  input  wire [NUM_CHAN-1:0] req,
  output reg  [NUM_CHAN-1:0] grant
);

  reg [1:0] rotate_ptr;
  reg [NUM_CHAN-1:0] next_grant;

  // ---- decision matrix (rotating priority) ----
  always @(*) begin
    next_grant = 4'b0000;
    case (rotate_ptr)
      2'd0: begin
        if      (req[0]) next_grant = 4'b0001;
        else if (req[1]) next_grant = 4'b0010;
        else if (req[2]) next_grant = 4'b0100;
        else if (req[3]) next_grant = 4'b1000;
      end
      2'd1: begin
        if      (req[1]) next_grant = 4'b0010;
        else if (req[2]) next_grant = 4'b0100;
        else if (req[3]) next_grant = 4'b1000;
        else if (req[0]) next_grant = 4'b0001;
      end
      2'd2: begin
        if      (req[2]) next_grant = 4'b0100;
        else if (req[3]) next_grant = 4'b1000;
        else if (req[0]) next_grant = 4'b0001;
        else if (req[1]) next_grant = 4'b0010;
      end
      2'd3: begin
        if      (req[3]) next_grant = 4'b1000;
        else if (req[0]) next_grant = 4'b0001;
        else if (req[1]) next_grant = 4'b0010;
        else if (req[2]) next_grant = 4'b0100;
      end
      default: next_grant = 4'b0000;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) grant <= 4'b0000;
    else        grant <= next_grant;
  end

  // ---- pointer update ----
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rotate_ptr <= 2'd0;
    end else begin
      if      (next_grant[0]) rotate_ptr <= 2'd1;
      else if (next_grant[1]) rotate_ptr <= 2'd2;
      else if (next_grant[2]) rotate_ptr <= 2'd3;
      else if (next_grant[3]) rotate_ptr <= 2'd0;
    end
  end

endmodule
