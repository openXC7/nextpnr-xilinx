#!/usr/bin/env python3
# gen_tb.py -- generate an iverilog testbench for the physical Verilog netlist
# emitted by VerilogPhys (see json_drc-portable).  Drives a reset pulse then a
# 200 MHz clock, samples every flip-flop via the "// FF <name> Q=<wire>" probe
# comments, prints the PRBS/Johnson state each cycle and counts undriven (X)
# cycles -- the regression that catches the bypass-FFMUX defect.
#
# Usage: gen_tb.py phys.v tb.v

import re, sys
phys, out = sys.argv[1], sys.argv[2]
v = open(phys).read()
ff = {n: w for n, w in re.findall(r'// FF (\w+) Q=(\w+)', v)}

def idx(pfx):
    return sorted((int(n.split('_')[-2]), w) for n, w in ff.items() if n.startswith(pfx))

prbs, john = idx('prbs_reg_'), idx('johnson_reg_')
pw = '{' + ','.join('dut.' + w for _, w in reversed(prbs)) + '}'
jw = '{' + ','.join('dut.' + w for _, w in reversed(john)) + '}'

L = []
A = L.append
A('`timescale 1ns/1ps')
A('module tb;')
A('  reg sysclk_p=0, rst=1;')
A('  wire sysclk_n = ~sysclk_p;')
A('  wire led_0_,led_1_,led_2_,led_3_,led_4_,led_5_,led_6_,led_7_;')
A('  top dut(.sysclk_p(sysclk_p),.sysclk_n(sysclk_n),.rst(rst),')
A('    .led_0_(led_0_),.led_1_(led_1_),.led_2_(led_2_),.led_3_(led_3_),')
A('    .led_4_(led_4_),.led_5_(led_5_),.led_6_(led_6_),.led_7_(led_7_));')
A('  always #2.5 sysclk_p = ~sysclk_p;')      # 200 MHz
A('  wire [%d:0] PRBS = %s;' % (len(prbs)-1, pw))
A('  wire [%d:0] JOHN = %s;' % (len(john)-1, jw))
A('  integer i, xcnt=0;')
A('  initial begin')
A('    repeat(6) @(posedge sysclk_p);')        # hold reset
A('    @(negedge sysclk_p); rst=0;')
A('    for(i=0;i<40;i=i+1) begin')
A('      @(posedge sysclk_p); #0.1;')
A('      $display("cyc %0d PRBS=%h JOHN=%b", i, PRBS, JOHN);')
A("      if (^PRBS === 1'bx) begin $display(\"  XXX PRBS has X\"); xcnt=xcnt+1; end")
A('    end')
A('    $display("X-cycles=%0d", xcnt);')
A('    $finish; end')
A('endmodule')
open(out, 'w').write('\n'.join(L))
print("wrote", out, "(prbs=%d johnson=%d FFs)" % (len(prbs), len(john)))
