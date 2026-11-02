`timescale 1ns/1ps

//=============================================================================
// ahb_defines.v
//------------------------------------------------------------------------------
// Shared constants, encodings and helper macros for the AHB/APB SoC RTL
// package (plain Verilog edition - IEEE 1364-2001).
//
// Include this file at the top of every module:
//     `include "ahb_defines.v"
//
// (SystemVerilog has packages; plain Verilog uses `define macros instead.)
//=============================================================================

// ---------------------------------------------------------------------------
// AHB-Lite encodings (AMBA 2)
// ---------------------------------------------------------------------------
`define HTRANS_IDLE   2'b00
`define HTRANS_BUSY   2'b01
`define HTRANS_NONSEQ 2'b10
`define HTRANS_SEQ    2'b11

`define HSIZE_BYTE    3'b000
`define HSIZE_HALF    3'b001
`define HSIZE_WORD    3'b010

`define HBURST_SINGLE 3'b000
`define HBURST_INCR   3'b001

`define HRESP_OKAY    2'b00
`define HRESP_ERROR   2'b10

// ---------------------------------------------------------------------------
// AXI burst encodings
// ---------------------------------------------------------------------------
`define AXI_BURST_FIXED 2'b00
`define AXI_BURST_INCR  2'b01
`define AXI_BURST_WRAP  2'b10

// ---------------------------------------------------------------------------
// clog2 macro: ceiling of log base 2 (constant expression)
// ---------------------------------------------------------------------------
`define log2(VALUE) ((VALUE) < ( 1 ) ? 0 : (VALUE) < ( 2 ) ? 1 : (VALUE) < ( 4 ) ? 2 : (VALUE) < ( 8 ) ? 3 : (VALUE) < ( 16 ) ? 4 : (VALUE) < ( 32 ) ? 5 : (VALUE) < ( 64 ) ? 6 : (VALUE) < ( 128 ) ? 7 : (VALUE) < ( 256 ) ? 8 : (VALUE) < ( 512 ) ? 9 : (VALUE) < ( 1024 ) ? 10 : (VALUE) < ( 2048 ) ? 11 : (VALUE) < ( 4096 ) ? 12 : (VALUE) < ( 8192 ) ? 13 : (VALUE) < ( 16384 ) ? 14 : (VALUE) < ( 32768 ) ? 15 : (VALUE) < ( 65536 ) ? 16 : (VALUE) < ( 131072 ) ? 17 : (VALUE) < ( 262144 ) ? 18 : (VALUE) < ( 524288 ) ? 19 : (VALUE) < ( 1048576 ) ? 20 : (VALUE) < ( 2097152 ) ? 21 : (VALUE) < ( 4194304 ) ? 22 : (VALUE) < ( 8388608 ) ? 23 : (VALUE) < ( 16777216 ) ? 24 : (VALUE) < ( 33554432 ) ? 25 : 26)

// ---------------------------------------------------------------------------
// AHB byte-enables (4 bits) from HSIZE + HADDR[1:0]
// ---------------------------------------------------------------------------
`define AHB_BE_FROM_HSIZE(hsize, addr) \
  (({hsize, addr} == {3'b000, 2'b00}) ? 4'b0001 : \
   ({hsize, addr} == {3'b000, 2'b01}) ? 4'b0010 : \
   ({hsize, addr} == {3'b000, 2'b10}) ? 4'b0100 : \
   ({hsize, addr} == {3'b000, 2'b11}) ? 4'b1000 : \
   ({hsize, addr} == {3'b001, 2'b00}) ? 4'b0011 : \
   ({hsize, addr} == {3'b001, 2'b10}) ? 4'b1100 : \
   4'b1111)

// ---------------------------------------------------------------------------
// OBI byte-enable -> {HSIZE, HADDR[1:0]} (5-bit result)
// ---------------------------------------------------------------------------
`define OBI_BE_TO_HSIZE(be) \
  ((be) == 4'b0001 ? {3'b000, 2'b00} : \
   (be) == 4'b0010 ? {3'b000, 2'b01} : \
   (be) == 4'b0100 ? {3'b000, 2'b10} : \
   (be) == 4'b1000 ? {3'b000, 2'b11} : \
   (be) == 4'b0011 ? {3'b001, 2'b00} : \
   (be) == 4'b1100 ? {3'b001, 2'b10} : \
   {3'b010, 2'b00})
