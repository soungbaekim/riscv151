// Module: ALU.v
// Desc:   32-bit ALU for the MIPS150 Processor
// Inputs: 
//    A: 32-bit value
//    B: 32-bit value
//    ALUop: Selects the ALU's operation 
// 						
// Outputs:
//    Out: The chosen function mapped to A and B.

`include "Opcode.vh"
`include "ALUop.vh"

module ALU(
    input [31:0] A,B,
    input [3:0] ALUop,
    output reg [31:0] Out //changed to output reg instead of reg
);

    // Implement your ALU here, then delete this comment
always@(*) begin
	case(ALUop)
		`ALU_ADD: Out=A+B;
		`ALU_SUB: Out=A-B;
		`ALU_AND: Out=A&B;
		`ALU_OR:  Out=A|B;
		`ALU_XOR: Out=A^B;
		`ALU_SLT: begin
				if($signed(A)<$signed(B)) Out=1;
				else Out=0;
			end
		`ALU_SLTU: begin
				if(A<B) Out=1;
				else Out=0;
			end
		`ALU_SLL: Out = A << B[4:0]; //empty bits filled with 0
		`ALU_SRA: Out = $signed(A) >>> B[4:0]; //empty bits filled with sign bit
		`ALU_SRL: Out = A >> B[4:0]; //empty bits filled with 0;
		`ALU_COPY_B: Out=B;
		`ALU_XXX: Out=0; //not sure
		default: Out=0;    //not sure
	endcase
end

endmodule
