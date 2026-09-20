/*
 * Copyright (c) 2026 Jet Meneses
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SEQ8 - a tiny programmable output sequencer
 *
 * An 8-instruction programmable state machine. You shift a program in
 * over a simple SPI-like port, raise RUN, and it drives the 8 output
 * pins according to that program.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_jet_seq8b (
    input  wire       VPWR,
    input  wire       VGND,
    input  wire [7:0] ui_in,    // dedicated inputs
    output wire [7:0] uo_out,   // dedicated outputs
    input  wire [7:0] uio_in,   // bidirectional: input path (unused)
    output wire [7:0] uio_out,  // bidirectional: output path
    output wire [7:0] uio_oe,   // bidirectional: 1 = drive as output
    input  wire       ena,      // always 1 when the design is selected
    input  wire       clk,      // clock
    input  wire       rst_n     // reset, active low
);

  // --------------------------------------------------------------------
  // Input synchronisers.
  // ui_in comes from the outside world and can change at any moment, so
  // every input passes through two flip-flops before it is used.
  // --------------------------------------------------------------------
  reg [7:0] sync0, sync1;

  always @(posedge clk) begin
    if (!rst_n) begin
      sync0 <= 8'b0000_0100;  // CS_N idles high
      sync1 <= 8'b0000_0100;
    end else begin
      sync0 <= ui_in;
      sync1 <= sync0;
    end
  end

  wire       sck    = sync1[0];    // program load clock
  wire       mosi   = sync1[1];    // program load data
  wire       csn    = sync1[2];    // program load select, active low
  wire       run    = sync1[3];    // 1 = execute, 0 = reset the core
  wire       in0    = sync1[4];    // branch input 0
  wire       in1    = sync1[5];    // branch input 1
  wire [1:0] ps_sel = sync1[7:6];  // tick speed select

  // Edge detectors for the loader
  reg sck_d, csn_d;
  always @(posedge clk) begin
    if (!rst_n) begin
      sck_d <= 1'b0;
      csn_d <= 1'b1;
    end else begin
      sck_d <= sck;
      csn_d <= csn;
    end
  end

  wire sck_rise = sck & ~sck_d;
  wire csn_fall = ~csn & csn_d;

  // --------------------------------------------------------------------
  // Program memory and the loader.
  //
  // 8 words x 10 bits. Pull CS_N low, then clock 10 bits per instruction
  // on the rising edge of SCK, most significant bit first. The write
  // address starts at 0 and auto-increments, so you shift in the whole
  // program back to back and raise CS_N when done.
  // --------------------------------------------------------------------
  reg [9:0] pmem [0:7];
  reg [8:0] shreg;    // the 9 bits shifted in so far; the 10th is MOSI
  reg [3:0] bitcnt;
  reg [2:0] ldaddr;

  wire [9:0] shreg_next = {shreg, mosi};
  wire       word_done  = (bitcnt == 4'd9);

  always @(posedge clk) begin
    if (!rst_n) begin
      shreg  <= 9'd0;
      bitcnt <= 4'd0;
      ldaddr <= 3'd0;
    end else if (csn_fall) begin
      // A new load session starts at address 0
      bitcnt <= 4'd0;
      ldaddr <= 3'd0;
    end else if (!csn && sck_rise) begin
      shreg <= shreg_next[8:0];
      if (word_done) begin
        pmem[ldaddr] <= shreg_next;
        ldaddr       <= ldaddr + 3'd1;
        bitcnt       <= 4'd0;
      end else begin
        bitcnt <= bitcnt + 4'd1;
      end
    end
  end

  // --------------------------------------------------------------------
  // Tick generator.
  //
  // WAIT counts "ticks". ui_in[7:6] picks how many clock cycles make one
  // tick: 00 = 1, 01 = 16, 10 = 256, 11 = 4096. The counter is held at
  // zero while RUN is low so timing always starts from the same point.
  // --------------------------------------------------------------------
  reg [11:0] ps_cnt;

  wire t0 = &ps_cnt[3:0];
  wire t1 = &ps_cnt[7:4];
  wire t2 = &ps_cnt[11:8];

  wire tick = (ps_sel == 2'd0)
            | ((ps_sel == 2'd1) & t0)
            | ((ps_sel == 2'd2) & t0 & t1)
            | ((ps_sel == 2'd3) & t0 & t1 & t2);

  always @(posedge clk) begin
    if (!rst_n || !run) ps_cnt <= 12'd0;
    else                ps_cnt <= ps_cnt + 12'd1;
  end

  // --------------------------------------------------------------------
  // The core.
  //
  // Instruction word: [9:8] opcode, [7:0] operand.
  //   00 OUT  imm         uo_out <= imm
  //   01 WAIT n           pause for n+1 ticks
  //   10 JMP  cond,addr   cond = imm[7:6], addr = imm[2:0]
  //                         00 always, 01 if IN0 high, 10 if IN1 high,
  //                         11 HALT (stop, outputs hold)
  //   11 NOP
  // The program counter is 3 bits, so it wraps from 7 back to 0 by
  // itself. A program that fills all 8 words loops without needing a JMP.
  // --------------------------------------------------------------------
  localparam OP_OUT  = 2'd0;
  localparam OP_WAIT = 2'd1;
  localparam OP_JMP  = 2'd2;
  localparam OP_NOP  = 2'd3;

  localparam C_ALWAYS = 2'd0;
  localparam C_IN0    = 2'd1;
  localparam C_IN1    = 2'd2;
  localparam C_HALT   = 2'd3;

  reg [2:0] pc;
  reg [7:0] wait_cnt;
  reg [7:0] outl;
  reg       waiting;
  reg       halted;

  wire [9:0] ir   = pmem[pc];
  wire [1:0] op   = ir[9:8];
  wire [7:0] imm  = ir[7:0];
  wire [1:0] cond = imm[7:6];

  wire take = (cond == C_ALWAYS)
            | ((cond == C_IN0) & in0)
            | ((cond == C_IN1) & in1);

  always @(posedge clk) begin
    if (!rst_n || !run) begin
      // RUN low holds the core in reset. Load your program while RUN is
      // low, then raise it to start from address 0.
      pc       <= 3'd0;
      wait_cnt <= 8'd0;
      outl     <= 8'd0;
      waiting  <= 1'b0;
      halted   <= 1'b0;
    end else if (halted) begin
      halted <= 1'b1;  // stay put, outputs hold their last value
    end else if (waiting) begin
      if (tick) begin
        if (wait_cnt == 8'd0) begin
          waiting <= 1'b0;
          pc      <= pc + 3'd1;
        end else begin
          wait_cnt <= wait_cnt - 8'd1;
        end
      end
    end else begin
      case (op)
        OP_OUT:  begin outl <= imm; pc <= pc + 3'd1; end
        OP_WAIT: begin wait_cnt <= imm; waiting <= 1'b1; end
        OP_JMP: begin
          if (cond == C_HALT) halted <= 1'b1;
          else if (take)      pc <= imm[2:0];
          else                pc <= pc + 3'd1;
        end
        OP_NOP:  pc <= pc + 3'd1;
      endcase
    end
  end

  // --------------------------------------------------------------------
  // Outputs
  // --------------------------------------------------------------------
  assign uo_out  = outl;
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;  // bidirectional pins are not used

  // Tie off unused inputs so the linter stays quiet
  wire _unused = &{ena, uio_in, VPWR, VGND, 1'b0};

endmodule
