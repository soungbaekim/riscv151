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
wire [31:0] s0_PC; s1_PC, s1_PCplus4
wire PCsel; //from controller, 0 for PC+4, 1 from ALU, 2 for PC
assign s1_PCplus4 = s1_PC + 4;
REGISTER_R #(.N(WIDTH)) pc_reg(.q(s1_PC), .d(s0_PC), .rst(reset), .clk(clk));
always@(*) begin
	case(PCsel)
		2'd0: s0_PC=s1_PCplus4;
		2'd1: s0_PC=s2_ALUout;
		2'd2: s0_PC=s1_PC;
		default: s0_PC = s1_PCplus4;
	endcase
end
	
//ICache
wire [31:0] s1_inst_read;
//icache_re comes from controller
assign icache_addr = s0_PC;
assign s1_inst_read = icache_dout;

//Kill inst mux
wire [31:0] No_OP;
assign No_OP = 32'b000000000000_00000_000_00000_0010011; //not sure this is the best way to write this
wire [31:0] s1_inst;
wire inst_kill; //from controller
assign s1_inst = (inst_kill) ? No_OP : s1_inst_read;

//Register file
wire [31:0] s1_reg_SrcA, s1_reg_SrcB, s3_WB;
wire [4:0] s3_A0, s1_A1, s1_A2;
wire RegFile_WE; //from controller
//name is inccorectly listed in source code as REGFILE_1R2W
REGFILE_1W2R #(.AWIDTH(LOGDEPTH), .DWIDTH(WIDTH), .DEPTH(DEPTH)) regFile (
    .d0(s3_WB), .addr0(s1_A0), .we0(RegFile_WE),
    .q1(s1_reg_SrcA), .addr1(s1_A1),
    .q2(s1_reg_SrcB), .addr2(s1_A2),
    .clk(clk) );

//Breaking up insruction
wire [4:0] s1_CSR_imm;
wire [31:0] s1_A0;
assign s1_A0 = s1_inst[11:7];
assign s1_A1 = s1_inst[19:15];
assign s1_A2 = s1_inst[24:20];
assign s1_CSR_imm = s1_inst[19:15];


//Immediate generator
wire ImmSel; //from controller, 0 if I type, 1 if S type
wire [4:0] imm_low5bits;
wire [31:0] s1_imm;
assign imm_low5bits = (ImmSel) ? s1_inst[4:0] : s1_inst[24:20];
assign s1_imm = {21{s1_inst[31]},s1_inst[30:25],imm_low5bits};


//s1 to s2 registers
wire [31:0] s2_PC, s2_reg_SrcA, s2_reg_SrcB, s2_imm, s2_PCplus4;
wire [4:0] s2_CSR_imm, s2_A0;
REGISTER_R #(.N(WIDTH)) s12_reg0(.q(s2_PC), .d(s1_PC), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg1(.q(s2_reg_SrcA), .d(s1_reg_SrcA), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg2(.q(s2_reg_SrcB), .d(s1_reg_SrcB), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg3(.q(s2_imm), .d(s1_imm), .rst(reset), .clk(clk));
REGISTER_R #(.N(5)) s12_reg4(.q(s2_CSR_imm), .d(s1_CSR_imm), .rst(reset), .clk(clk));
REGISTER_R #(.N(5)) s12_reg5(.q(s2_A0), .d(s1_A0), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s12_reg6(.q(s2_PCplus4), .d(s1_PCplus4), .rst(reset), .clk(clk));


//Bypass muxes
wire [31:0] s2_SrcA, s2_SrcB;
wire [31:0] s2_bypass_value;
wire bypass_A, bypass_B;
assign s2_SrcA = (bypass_A) ? s2_bypass_value : s2_reg_SrcA;
assign s2_SrcB= (bypass_B) ? s2_bypass_value : s2_reg_SrcB;


//Branch compare
wire BrEq, BrLT; //to controller
wire BrUn; //from controller
assign BrEq = (s2_SrcA==s2_SrcB) ? 1'b1 : 1'b0;
assign BrLT = ((BrUn==0 && s2_SrcA<s2_SrcB) || (BrUn==1 && $signed(s2_SrcA)<$signed(s2_SrcB))) ? 1'b1 : 1'b0;


//input MUXes to ALU
wire Asel, Bsel; //from controller
wire [31:0] s2_ALUin_A, s2_ALUin_B;
assign s2_ALUin_A = (ASel) ? s2_PC : s2_SrcA;
assign s2_ALUin_B = (BSel) ? s2_imm : s2_SrcB;


//ALU
wire [3:0] ALUop; //from controller
wire [31:0] s2_ALUout;
ALU myALU (
	.A(s2_ALUin_A), .B(s2_ALUin_B),
	.ALUop(ALUop), .Out(s2_ALUout));

//Store mask
wire [1:0] st_size;
wire [31:0] s2_WD;
//very unsure on this one
always@(*) begin
	case(st_size) //0=word, 1=half, 2=byte
		2'd0: s2_WD = s2_SrcB;
		2'd1: s2_WD = {16{s2_SrcB[15]},s2_SrcB[15:0]};
		2'd2: s2_WD = {24{s2_SrcB[7]},s2_SrcB[7:0]};
		default: s2_WD = s2_SrcB; 
	endcase
end


//CSR imm zero extend
wire [31:0] s2_CSR_imm_ext;
assign s2_CSR_imm_ext = {27'd0,s2_CSR_imm};

//CSR mux
wire [31:0] s2_CSR_WD;
wire CSR_sel; //from controller
assign s2_CSR_WD = (CSR_sel) ? s2_CSR_imm_ext : s2_SrcA;


//s1 to s2 registers
wire [31:0] s3_ALUout, s3_CSR_WD, s3_PCplus4;
REGISTER_R #(.N(WIDTH)) s23_reg0(.q(s3_ALUout), .d(s2_ALUout), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s23_reg1(.q(s3_CSR_WD), .d(s2_CSR_WD), .rst(reset), .clk(clk));
REGISTER_R #(.N(5)) s23_reg2(.q(s3_A0), .d(s2_A0), .rst(reset), .clk(clk));
REGISTER_R #(.N(WIDTH)) s23_reg3(.q(s3_PCplus4), .d(s2_PCplus4), .rst(reset), .clk(clk)); 


//DCache
//dcache_we and dcache_re coming from controller
wire [31:0] s3_ReadData;
assign dcache_addr = s2_ALUout;
assign dcache_din = s2_WD;
assign s3_ReadData = dcache_dout;


//Load mask
wire [1:0] ld_size; //from controller
wire ld_sign; //from controller, 1 for signed, 0 for unsigned
wire [31:0] s3_LoadData;
always@(*) begin
	case(ld_size) //0=word, 1=half, 2=byte
		2'd0: s3_LoadData = s3_ReadData;
		2'd1: s3_LoadData = (ld_sign) ? {16{s3_ReadData[15]},s3_ReadData[15:0]} : {16'd0, s3_ReadData[15:0]};
		2'd2: s3_LoadData = (ld_sign) ? {24{s3_ReadData[7]},s3_ReadData[7:0]}   : {24'd0, s3_ReadData[7:0]};
		default: s3_LoadData = s3_ReadData; 
	endcase
end


 
//WB mux
wire [1:0] WBSel; //from controller
always@(*) begin
	case(WBSel)
		2'd0: s3_WB=s3_LoadData;
		2'd1: s3_WB=s3_ALUout;
		2'd2: s3_WB=s3_PCplus4;
		default: s3_WB=s3_ALUout;
	endcase
end


//Bypass
wire [31:0] s3_WB_delay;
wire bypass_delay; //from controller
REGISTER_R #(.N(WIDTH)) bypass_reg(.q(s3_WB_dealy), .d(s3_WB), .rst(reset), .clk(clk)); 
assign s2_bypass_value = (bypass_delay) ? s3_WB_delay : s3_WB;

		








endmodule
