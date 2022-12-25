# CPU-Design
This project contains the design of the 5-stages pipelined CPU based on MIPS instruction set architecture, written in Verilog HDL


## Abstract
In this project, I implemented the design of the 5 stages pipelined CPU based on MIPS instruction set architecture, using the Xilinx design package for FPGAs. In addition to previous labs, I added another feature of the processor where it detects data hazards and resolves them by pipeline stalls and full forwarding.

## Introduction
The processor uses pipelining technique to execute a set of instructions much faster. Our processor, with 5-stage pipelined CPU, can execute at most 5 instructions at a time. This is done by saving temporary results in each pipeline stage into corresponding pipeline registers and use those stored data when the instruction moves to the next stage. Our processor has mainly 5 stages: Instruction Fetch (IF), Instruction Decode (ID), Execution (EXE), Memory Access (MEM), and Write Back (WB). The program counter register (pc) serves as the first pipeline register, and other 4 registers are named IF/ID, ID/EXE, EXE/MEM, and MEM/WB as shown in Figure 1.

For example, suppose that we have a sequence of instructions A – B – C – D, to be executed sequentially, and instruction A first goes through the IF stage. As soon as A finishes IF stage, the output from ID stage is stored in IF/ID register and the next instruction B enters the IF stage. Since we lose information from resetting/reusing the component, pipeline registers are required to hold information produced in previous cycle and feed it to the next stage. Similarly, if A is in MEM stage, B is in EXE, C is in ID, and D is in IF stage respectively. An instruction exits the CPU Datapath as it finishes the WB stage. This way, the CPU can execute 5 instructions at a single clock cycle. Under ideal conditions and with a large number of instructions, our CPU is approximately 5 times faster than a single cycled CPU. This is an advantage by parallelism.

In addition to pipelining, the designed processor also supports hazard detection, pipeline stalls, and full forwarding. When there are dependencies between several instructions where next instruction uses data before it is ready from previous instruction, there exists a data hazard. In this case, CPU needs to wait for previous instruction to complete its data read/write procedure by using result as soon as it is computed or stalling the pipeline. In our implementation, data hazards are resolved by additional input/output signals from the control unit and two multiplexors (fqaMux, fqbMux), as shown in Figure 2 schematics.

There are 3 types of data hazards.

### I. EX hazard
EX hazard happens when the destination register of the previous instruction is equal to either rs or rt of current instruction in ID stage. In this case, we forward from the output of ALU to the input of ALU to resolve this hazard, setting fwda and fwdb signal to 1.

### II. MEM hazard
MEM hazard happens when the destination register of the second previous instruction is equal to either rs or rt of current instruction in ID stage. In this case, we forward from the MEM/WB register to the input of ALU to resolve this hazard, setting fwda and fwdb signal to 2.

### III. lw-use hazard
lw-use hazard happens when the previous instruction is load word(lw) and its destination register (rt) is equal to either rs or rt of current instruction in ID stage. In this case, pipeline stall is necessary. Hence, we stall 1 time and forward from the output of memory to the input of ALU to resolve this hazard, setting fwda or fwdb signal to 3.

Given a stall signal from the control unit, pipeline stall is implemented by prohibiting the updates of the program counter and the content of IF/ID pipeline register and cancelling the first instruction. This stall signal becomes true only if either one of source registers, rs or rt, of an instruction right after the load word instruction is equivalent to the destination register rt of lw instruction.
If the processor needs to stall, the control unit sets the write enable signal (wpcir) to 0 to prevent the updates of pc and IF/ID Reg. Also, it sets all wreg, m2reg, wmem signals to 0 to prevent the instruction from updating the states of the registers and memory.
This forwarding architecture along with hazard detection is beneficial in terms of performance because the CPU does not have to wait until the data is finally ready at WB stage. Instead, it can use data in subsequent instructions as soon as it is available. This way, a CPU have a smaller number of stalls and finish the same number of instructions in fewer number of clock cycles even if the program has a lot of dependencies in between instructions.

## Verification
The implementation of this processor is verified by the following set of MIPS instructions. 

1 	lw $9, 4($1)

2 	add $8, $8, $9

3 	sub $4, $9, $8

4 	or $5, $8, $9

5 	xor $6, $8, $9

6 	and $7, $8, $9
