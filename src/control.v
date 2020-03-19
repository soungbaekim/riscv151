// Module: ALUdecoder
// Desc:   Sets the ALU operation
// Inputs: opcode: the top 6 bits of the instruction
//         funct: the funct, in the case of r-type instructions
//         add_rshift_type: selects whether an ADD vs SUB, or an SRA vs SRL
// Outputs: ALUop: Selects the ALU's operation
//

`include "Opcode.vh"
`include "ALUop.vh"

module control(
  input [31:0] inst,
  output reg PCsel,
  output reg  ImmSel, //0 if I type, 1 if S type
  output reg BrUn,
  output reg Asel, Bsel,
  input BrEq, BrLT,
  output reg [3:0] ALUop,
  output reg MemRW,
  output reg WBSel
);

// Implement your ALU decoder here, then delete this comment
wire [6:0] opcode;
wire [2:0] funct;
wire add_rshift_type

assign opcode = inst[6:0];
assign funct = inst[14:12];
assign add_rshift_type = inst[30];


always@(*) begin
	case(opcode)
		//special immediate
		`OPC_LUI: ALUop=`ALU_COPY_B; 
		`OPC_AUIPC: ALUop=`ALU_ADD; //adds 20bit upper immediate to the PC
		//jump
		`OPC_JAL: ALUop=`ALU_ADD; //needs to add PC+offset
		`OPC_JALR: ALUop=`ALU_ADD; //need to add rs1+offset
		//branch
		`OPC_BRANCH: ALUop=`ALU_ADD; //adding an offset to pc, p40 on lecture
		//load and store
		`OPC_STORE, `OPC_LOAD:  begin
			case(funct)//total guesses
				`FNC_SB: ALUop=`ALU_ADD;
				`FNC_SH: ALUop=`ALU_ADD;
				`FNC_SW: ALUop=`ALU_ADD;
				default: ALUop=`ALU_ADD;
			endcase
		end
		`OPC_LOAD: begin
			case(funct) //not sure about any of these
				`FNC_LB: ALUop=`ALU_ADD;
				`FNC_LH: ALUop=`ALU_ADD;
				`FNC_LW: ALUop=`ALU_ADD;
				`FNC_LBU: ALUop=`ALU_ADD;
				`FNC_LHU: ALUop = `ALU_ADD;
				default: ALUop=`ALU_ADD; //`ALU_XXX;
			endcase
		end 
		//arithmetic
		`OPC_ARI_RTYPE: begin
			case(funct)
				`FNC_ADD_SUB: begin
					if(add_rshift_type) ALUop=`ALU_SUB; 			
					else ALUop=`ALU_ADD;
				end
				`FNC_SLL: ALUop=`ALU_SLL;
				`FNC_SLT: ALUop=`ALU_SLT;
				`FNC_SLTU: ALUop=`ALU_SLTU;
				`FNC_XOR: ALUop=`ALU_XOR;
				`FNC_OR: ALUop=`ALU_OR;
				`FNC_AND: ALUop=`ALU_AND;
				`FNC_SRL_SRA: begin
					if(add_rshift_type) ALUop=`ALU_SRA; 			
					else ALUop=`ALU_SRL;
				end
				default: ALUop=`ALU_XXX;
			endcase	
		end
		`OPC_ARI_ITYPE: begin
			case(funct)
				`FNC_ADD_SUB: ALUop=`ALU_ADD;
				`FNC_SLL: ALUop=`ALU_SLL;
				`FNC_SLT: ALUop=`ALU_SLT;
				`FNC_SLTU: ALUop=`ALU_SLTU;
				`FNC_XOR: ALUop=`ALU_XOR;
				`FNC_OR: ALUop=`ALU_OR;
				`FNC_AND: ALUop=`ALU_AND;
				`FNC_SRL_SRA: begin
					if(add_rshift_type) ALUop=`ALU_SRA; 			
					else ALUop=`ALU_SRL;
				end
				default: ALUop=`ALU_XXX;
			endcase	
		end

		default: ALUop=`ALU_XXX;
	endcase
end




endmodule
