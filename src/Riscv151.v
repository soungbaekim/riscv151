`include "const.vh"

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output [31:0] dcache_addr,
    output [31:0] icache_addr,
    output [3:0] dcache_we,
    output dcache_re,
    output icache_re,
    output [31:0] dcache_din,
    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall,
    output [31:0] csr

);

    // TODO: Your code
    // Please use REGFILE_1W2R for your register file
    // (two async read ports, 1 sync write port)
    // Please use REGISTER* modules for all sequential logic
localparam integer WIDTH = 32;
localparam integer LOGDEPTH = 5;
localparam DEPTH = (1 << LOGDEPTH);


//Program count
wire [31:0] s0_PC, s0_next_pc, s0_next_pcPlus4;
wire PCsel; //from controller, 1 from ALU, 0 for PC+4
assign s0_next_pc = (PCsel) ? s1_ALUout : s0_next_pcPlus4;
assign s0_next_pcPlus4 = s0_PC + 4;
REGISTER_R #(.N(WIDTH)) pc_reg(.q(s0_PC), .d(s0_next_pc), .rst(reset), .clk(clk));

//IMEM
wire [31:0] s0_inst;
assign icache_addr = s0_pc;
assign icache_re = reset; //not sure what this does
assign s0_inst = icache_dout;

//s0 to s1 registers
wire [31:0] s1_inst;
REGISTER_R #(.N(WIDTH)) s12_reg0(.q(s1_inst), .d(s0_inst), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg1(.q(s1_PC), .d(s0_PC), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg2(.q(s1_pcPlus4), .d(s0_next_pcPlus4), .rst(reset), .clk(clk));

//Register file
wire [31:0] s1_reg_SrcA, s1_reg_SrcB, s2_WB;
wire [5:0] s1_A0, s1_A1, s1_A2;
wire RegWEn;
assign RegWEn = 1'b1;

assign s1_A0 = s1_inst[11:7];
assign s1_A1 = s1_inst[19:15];
assign s1_A2 = s1_inst[24:20];

//name is inccorectly listed in source code as REGFILE_1R2W
REGFILE_1W2R #(.AWIDTH(LOGDEPTH), .DWIDTH(WIDTH), .DEPTH(DEPTH)) regFile (
    .d0(s2_WB), .addr0(s1_A0), .we0(RegWEn),
    .q1(s1_reg_SrcA), .addr1(s1_A1),
    .q2(s1_reg_SrcB), .addr2(s1_A2),
    .clk(clk) );

//Immediate generator
wire ImmSel; //from controller, 0 if I type, 1 if S type
wire [4:0] imm_low5bits;
assign imm_low5bits = (ImmSel) ? s1_inst[4:0] : s1_inst[24:20];
assign s1_imm = {21{s1_inst[31]},s1_inst[30:25],imm_low5bits};

//Branch comp and associated signals
wire BrEq, BrLT; //to controller
wire BrUn; //from controller
assign BrEq = (s1_reg_SrcA==s1_reg_SrcB) ? 1'b1 : 1'b0;
assign BrLT = ((BrUn==0 && s1_reg_SrcA<s1_reg_SrcB) || (BrUn==1 && $signed(s1_reg_SrcA)<$signed(s1_reg_SrcB))) ? 1 : 0;

//input MUXes to ALU
wire [31:0] s1_SrcA, s1_SrcB;
wire [31:0] s1_imm, s1_PC;
wire Asel, Bsel; //from controller
assign s1_SrcA = (ASel) ? s1_pc : s1_reg_SrcA;
assign s1_SrcB = (BSel) ? s1_imm : s1_reg_SrcB;

//ALU
wire [3:0] ALUop; //from controller
ALU myALU (
	.A(s1_SrcA), .B(s1_SrcB),
	.ALUop(ALUop), .Out(s1_ALUout));

//s1 to s2 registers
wire [31:0] s2_ALUout, s2_WD;
wire [31:0] s1_pcPlus4, s2_pcPlus4; //not positive about this length
REGISTER_R #(.N(WIDTH)) s12_reg0(.q(s2_ALUout), .d(s1_ALUout), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg1(.q(s2_WD), .d(s1_reg_SrcB), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg2(.q(s2_pcPlus4), .d(s1_pcPlus4), .rst(reset), .clk(clk)); //not sure about this reg size

//DMEM assignments
wire MemRW; //from controller
wire [31:0] s2_ReadData;
assign dcache_addr = s2_ALUout;
assign dcache_we = MemRW;
assign dcahce_re = reset; //is this correct?
assign dcache_din = s2_WD;
assign s2_ReadData = dcache_dout;
 
//WB mux
wire [1:0] WBSel; //from controller
always@(*) begin
	case(WBSel)
		2'd0: s2_WB=s2_ReadData;
		2'd1: s2_WB=s2_ALUout;
		2'd2: s2_WB=s2_pcPlus4;
		default: s2_WB=s2_ReadData;
	endcase
end
		








endmodule
