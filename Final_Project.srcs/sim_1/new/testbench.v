`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Penn State College of Engineering
// Engineer: Dohyoung Ko
// Create Date: 12/17/2022 05:26:34 AM
// Design Name: Final Project
// Module Name: testbench
// Project Name: CMPEN-331 FA22
//////////////////////////////////////////////////////////////////////////////////

module testbench();
    reg clk_tb;                 // clock signal
    wire [31:0] pc_tb;          // IF stage
    wire [31:0] inst_tb;    // IF/ID stage
    wire [31:0] ealu_tb;           // ID/EXE stage
    wire [31:0] malu_tb;          // EXE/MEM stage
    wire [31:0] wdi_tb;      // MEM/WB stage
    
    datapath datapath_tb (
        .clk(clk_tb),
        .pc(pc_tb),
        .dinstOut(inst_tb),
        .r(ealu_tb),
        .mr(malu_tb),
        .wbData(wdi_tb)
    );
    
    initial begin
        clk_tb = 0;             // initial clock is 0
    end

    always begin
        #5;
        clk_tb = ~clk_tb;       // alternate clock in every 10ns
    end
endmodule