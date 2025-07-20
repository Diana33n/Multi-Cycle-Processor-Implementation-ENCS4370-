# Multi-Cycle-Processor-Implementation-ENCS4370-

This repository contains the Verilog implementation and documentation of a multi-cycle RISC processor capable of handling 16-bit instructions. It was developed as part of **Computer Architecture (ENCS4370)** at the Faculty of Engineering & Technology, Electrical & Computer Engineering Department.

## Overview

This project demonstrates the design and implementation of a multi-cycle processor that supports a custom Instruction Set Architecture (ISA). The processor features:

* 16-bit word size
* Separate instruction and data memory
* 8 general-purpose registers
* Support for R-type, I-type, and J-type instructions
* Return register for subroutine management

The processor is designed to execute a wide range of operations efficiently, including arithmetic, logic, branching, and memory access instructions.

---

## Project Structure

| File                 | Description                                        |
| -------------------- | -------------------------------------------------- |
| `FinalCode_ARCH.txt` | Verilog source code for processor modules          |
| `FinalCode2.txt`     | Alternative version of processor Verilog code      |
| `FUL_LCode.v`        | Full processor Verilog code (Multi-cycle design)   |
| `Mux2-1.v`           | Multiplexer module (2:1 MUX)                       |
| `ARCH2_Final.pdf`    | Project report detailing design and implementation |

---

## Features

* Multi-cycle execution: each instruction completes in multiple clock cycles
* ISA support: implements R-type, I-type, and J-type instructions with the following operations:

  * Arithmetic: ADD, SUB, ADDI
  * Logical: AND, ANDI
  * Branch: BEQ, BNE, FOR
  * Jump: JMP, CALL, RET
  * Memory: LW, SW
* Control Units: includes PC Control, Main Controller, and ALU Control
* Separate instruction and data memory: avoids fetch-execute conflicts

---

## How to Run

### Prerequisites

* ModelSim, Vivado, or Quartus Prime
* Verilog simulation environment
* Basic knowledge of RTL simulation

### Steps

1. Open the Verilog files (`FinalCode_ARCH.txt` or `FinalCode2.txt`) in your simulator.
2. Compile all modules, including:

   * extender
   * reg\_file
   * dataMemory
   * flop
   * mux2x1, mux4x1, mux8x1
   * ALU, ALUcontrol, PCcontrol
   * mainController
   * datapath
   * CPU
3. Run the simulation using provided testbenches in the code.
4. Observe outputs for different instruction executions and verify results against the project report.

---

## Documentation

Detailed documentation includes:

* Instruction formats and encoding (R-Type, I-Type, J-Type)
* Datapath components (Registers, ALU, Multiplexers, Memory)
* Control path design (State machines and truth tables)
* Full-state diagrams and Boolean equations for control signals

Refer to `ARCH2_Final.pdf` for architecture diagrams, datapath, and control flow.

---

## Authors

| Name          | Student ID |
| ------------- | ---------- |
| Diana Naseer  | 1210363    |
| Lana Musaffer | 1210455    |
| Roaa Sroor    | 1221727    | 

Instructor: Dr. Ayman Hroub
Course: ENCS4370 – Computer Architecture
Date: 1/7/2025

---

## References

* David A. Patterson and John L. Hennessy, *Computer Organization and Design*
* Verilog HDL documentation
* ModelSim User Guide
