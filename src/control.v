// Module: control.v
// Controller for Riscv151.v

`include "Opcode.vh"
`include "ALUop.vh"
`include "const.vh"

module control(
  input             clk,
  input             reset,
  input [31:0]      inst,

  // Stage I
  output [1:0]  PC_Sel,
  output            ICache_RE, // UNUSED?
  output reg [2:0]  ImmSel, // 0 if I type, 1 if S type: 5 types
  output	    Inst_Kill,

  // Stage X
  input             BrEq, BrLT,
  output            BrUn,

  output [3:0]      ALUop,
  output            A_Sel, B_Sel,
  output            CSR_Sel,
  output [1:0]      ST_Size, // 3 types

  output            Bypass_A,
  output            Bypass_B,
 // output            Bypass_Sel,
  output 	    Bypass_Delay_A,
  output 	    Bypass_Delay_B,

  // Stage M
  output            DCache_WE,
  output            RegFile_WE,
  output            CSR_we,
  output [1:0]      WB_Sel,
  output [2:0]      LD_Size
);
  localparam WIDTH = 32;
  localparam ALU_WIDTH = 4;

  wire [6:0] opcode;
  wire [4:0] rd, rs1, rs2;
  wire [2:0] func3;
  wire [6:0] func7;

  // Spilt up inst
  assign opcode = inst[6:0];
  assign rd = inst[11:7];
  assign rs1 = inst[19:15];
  assign rs2 = inst[24:20];
  assign func3 = inst[14:12];
  assign func7 = inst[31:25];

  // ALU Decoder
  wire [3:0] aluop_next;
  ALUdec aludecoder(.opcode(opcode), .funct(func3), .add_rshift_type(inst[30]), .ALUop(aluop_next));

  // Control Registers
  wire [2:0] func3_X, func3_M;
  REGISTER_R #(.N(3)) func3_X_reg(.q(func3_X), .d(func3), .rst(reset), .clk(clk));
  REGISTER_R #(.N(3)) func3_M_reg(.q(func3_M), .d(func3_X), .rst(reset), .clk(clk));

  wire [4:0] rd_X, rd_M, rs1_X, rs1_M, rs2_X, rs2_M;
  REGISTER_R #(.N(5)) rd_X_reg(.q(rd_X), .d(rd), .rst(reset), .clk(clk));
  REGISTER_R #(.N(5)) rd_M_reg(.q(rd_M), .d(rd_X), .rst(reset), .clk(clk));

  REGISTER_R #(.N(5)) rs1_X_reg(.q(rs1_X), .d(rs1), .rst(reset), .clk(clk));
  REGISTER_R #(.N(5)) rs1_M_reg(.q(rs1_M), .d(rs1_X), .rst(reset), .clk(clk));

  REGISTER_R #(.N(5)) rs2_X_reg(.q(rs2_X), .d(rs2), .rst(reset), .clk(clk));
  REGISTER_R #(.N(5)) rs2_M_reg(.q(rs2_M), .d(rs2_X), .rst(reset), .clk(clk));

  // nop handler
  wire nop_I, nop_X, nop_M;
  REGISTER_R nop_IX_reg(.q(nop_X), .d(Inst_Kill), .rst(reset), .clk(clk));
  REGISTER_R nop_XM_reg(.q(nop_M), .d(nop_X), .rst(reset), .clk(clk));

  //assign nop_I = (Inst_Kill == 1'b1 || (rd == 5'd0)) ? 1'b1 : 1'b0;

  reg inst_kill_next;
  wire inst_kill_value;
  REGISTER_R inst_kill_reg(.q(inst_kill_value), .d(inst_kill_next), .rst(reset), .clk(clk));

  // TODO: Branches
  wire in_b_val;
  reg in_b_next, will_branch;
  REGISTER_R is_b_reg (.q(in_b_val), .d(in_b_next), .rst(reset), .clk(clk));

  reg BrUn_next;
  REGISTER_R br_un_reg (.q(BrUn), .d(BrUn_next), .rst(reset), .clk(clk));


  // Stage X Registers
  reg csr_sel_next, a_sel_next, b_sel_next;
  wire bypass_a_next, bypass_b_next;
  wire  bypass_delay_next_a, bypass_delay_next_b;
  //wire reg bypass_sel_next;
  REGISTER_R #(.N(ALU_WIDTH)) aluop_reg(.q(ALUop), .d(aluop_next), .rst(reset), .clk(clk));
  REGISTER_R csr_sel_reg(.q(CSR_Sel), .d(csr_sel_next), .rst(reset), .clk(clk));




  REGISTER_R bypass_a_reg(.q(Bypass_A), .d(bypass_a_next), .rst(reset), .clk(clk));
  REGISTER_R bypass_b_reg(.q(Bypass_B), .d(bypass_b_next), .rst(reset), .clk(clk));
 // REGISTER_R bypass_sel_reg(.q(Bypass_Sel), .d(bypass_sel_next), .rst(reset), .clk(clk));
  REGISTER_R bypass_delay_reg_a(.q(Bypass_Delay_A), .d(bypass_delay_next_a), .rst(reset), .clk(clk));
  REGISTER_R bypass_delay_reg_b(.q(Bypass_Delay_B), .d(bypass_delay_next_b), .rst(reset), .clk(clk));


  REGISTER_R a_sel_reg(.q(A_Sel), .d(a_sel_next), .rst(reset), .clk(clk));
  REGISTER_R b_sel_reg(.q(B_Sel), .d(b_sel_next), .rst(reset), .clk(clk));

  // Stage M registers
  reg  dcache_we_next;   /* See Notes below */
  wire dcache_we_value;
  //REGISTER_R dcache_we_reg1(.q(dcache_we_imm), .d(dcache_we_next), .rst(reset), .clk(clk));
  REGISTER_R #(.N(1)) dcache_we_reg2(.q(dcache_we_value), .d(dcache_we_next), .rst(reset), .clk(clk));
  assign DCache_WE = dcache_we_value && ~nop_X;

  reg csr_we_next;
  wire csr_we_imm;
  wire csr_we_imm2;
  REGISTER_R csr_we_reg1(.q(csr_we_imm), .d(csr_we_next), .rst(reset), .clk(clk));
  REGISTER_R csr_we_reg2(.q(csr_we_imm2), .d(csr_we_imm), .rst(reset), .clk(clk));
  assign CSR_we = csr_we_imm2 && ~nop_M;

  reg [1:0] wb_sel_next;
  wire [1:0] wb_sel_imm;
  REGISTER_R #(.N(2)) wb_sel_reg1(.q(wb_sel_imm), .d(wb_sel_next), .rst(reset), .clk(clk));
  REGISTER_R #(.N(2)) wb_sel_reg2(.q(WB_Sel), .d(wb_sel_imm), .rst(reset), .clk(clk));

  reg regfile_we_next; /* See Notes below */
  wire regfile_we_imm1;
  wire regfile_we_imm2;
  // Another register for delay since technically the one after?
  REGISTER_R regfile_we_reg1(.q(regfile_we_imm1), .d(regfile_we_next), .rst(reset), .clk(clk));
  //REGISTER_R regfile_we_reg1(.q(regfile_we_imm2), .d(regfile_we_imm1), .rst(reset), .clk(clk));s
  REGISTER_R regfile_we_reg2(.q(regfile_we_imm2), .d(regfile_we_imm1), .rst(reset), .clk(clk));
  assign RegFile_WE = regfile_we_imm2 && ~nop_M;

  assign PC_Sel = (Inst_Kill == 1'b1) ? `PCSEL_ALU : `PCSEL_PLUS4;
  assign Inst_Kill = inst_kill_value || will_branch;

  assign ICache_RE = 1'b1; // ALWAYS ON?
  assign ST_Size = func3_X[1:0];
  assign LD_Size = func3_M;

  /* if it gets more complicated, move it to another module? */
//  assign bypass_a_next = ((regfile_we_imm1 && (rs1 == rd_X) && ~nop_X) || (RegFile_WE && (rs1 == rd_M)))&& (rs1!=5'd0) ? `BYPASS_TRUE : `BYPASS_FALSE;
//  assign bypass_b_next = ((regfile_we_imm1 && (rs2 == rd_X) && ~nop_X) || (RegFile_WE && (rs2 == rd_M)))&&(rs1!=5'd0) ? `BYPASS_TRUE : `BYPASS_FALSE;
  assign bypass_a_next = ((regfile_we_imm1 && (rs1 == rd_X) && ~nop_X) || (RegFile_WE && (rs1 == rd_M)))&& (rs1!=5'd0) ? `BYPASS_TRUE : `BYPASS_FALSE;
  assign bypass_b_next = ((regfile_we_imm1 && (rs2 == rd_X) && ~nop_X) || (RegFile_WE && (rs2 == rd_M)))&&(rs2!=5'd0) ? `BYPASS_TRUE : `BYPASS_FALSE;

//  assign bypass_sel_next = ((rs1 == rd_X) || (rs2 == rd_X)) ? `BYPASS_CURR : `BYPASS_DELAY;
  assign bypass_delay_next_a = (regfile_we_imm1 && (rs1 == rd_X) && ~nop_X) ? `BYPASS_CURR : `BYPASS_DELAY;
  assign bypass_delay_next_b = (regfile_we_imm1 && (rs2 == rd_X) && ~nop_X)  ? `BYPASS_CURR : `BYPASS_DELAY;


//braeden's version (bad)
//  assign Bypass_A = ((RegFile_WE && (rs1_X == rd_M)) || (RegFile_WE && (rs1 == rd_M))) ? `BYPASS_TRUE : `BYPASS_FALSE;
//  assign Bypass_B = ((RegFile_WE && (rs2_X == rd_M)) || (RegFile_WE && (rs2 == rd_M))) ? `BYPASS_TRUE : `BYPASS_FALSE;
// assign Bypass_Delay_A = (rs1 == rd_M) ? `BYPASS_DELAY : `BYPASS_CURR;
//  assign Bypass_Delay_B = (rs2 == rd_M) ? `BYPASS_DELAY : `BYPASS_CURR;


  always @(*) begin
    //nops
    inst_kill_next = 1'b0;

    // default is nop
    csr_sel_next = `CSRSEL_IMM;
    csr_we_next = `WRITE_DISABLE;

    // TODO: BRANCHES
    in_b_next = 1'b0;

    if (in_b_val == 1'b1) begin
      case (func3_X)
        `FNC_BEQ: will_branch = (BrEq == 1'b1) ? 1'b1 : 1'b0;
        `FNC_BNE: will_branch = (BrEq == 1'b0) ? 1'b1 : 1'b0;
        `FNC_BLT, `FNC_BLTU: will_branch = (BrLT == 1'b1) ? 1'b1 : 1'b0;
        `FNC_BGE, `FNC_BGEU: will_branch = ((BrEq == 1'b1) || (BrLT == 1'b0)) ? 1'b1 : 1'b0;
      endcase
    end
    else begin
      will_branch = 1'b0;
    end

    /*
    bypass_a_next = `BYPASS_FALSE;
    bypass_b_next = `BYPASS_FALSE;
    bypass_sel_next = `BYPASS_CURR;
    */
    /*
    if (Inst_Kill == 1'b1) begin
      PC_Sel = `PCSEL_ALU;
    end else begin
      PC_Sel = `PCSEL_PLUS4;
    end
    */
    /*
    if (will_branch == 1'b1) begin

      //PC_Sel = `PCSEL_ALU;
      ImmSel = `IMMSEL_U;

      a_sel_next = `ASEL_REG;
      b_sel_next = `BSEL_IMM;

      dcache_we_next = `WRITE_DISABLE;
      regfile_we_next = `WRITE_DISABLE;

      wb_sel_next = `WBSEL_ALU;


    end

    else begin
    */
      case (opcode)
      `OPC_LUI: begin

        ImmSel = `IMMSEL_U;

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_ALU;


      end
      `OPC_AUIPC: begin

        ImmSel = `IMMSEL_U;

        a_sel_next = `ASEL_PC;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_ALU;


      end

      // Jump instructions
     `OPC_JAL: begin
        ImmSel = `IMMSEL_UJ;

        a_sel_next = `ASEL_PC;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_PC4;

        // TODO:nop
        inst_kill_next = ~Inst_Kill;
      end
      `OPC_JALR: begin
        ImmSel = `IMMSEL_I;

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_PC4;
        // TODO: nop?
        inst_kill_next = ~Inst_Kill;
      end

      // Branch instructions
      `OPC_BRANCH: begin

        ImmSel = `IMMSEL_SB;

        a_sel_next = `ASEL_PC;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_DISABLE;

        wb_sel_next = `WBSEL_ALU;


        in_b_next = ~Inst_Kill;
        case (func3)
          `FNC_BLTU, `FNC_BGEU: BrUn_next = 1'b1;
          default: BrUn_next = 1'b0;
        endcase
      end

      // Load and store instructions
      `OPC_STORE: begin

        ImmSel = `IMMSEL_S;

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_ENABLE;
        regfile_we_next = `WRITE_DISABLE;

        wb_sel_next = `WBSEL_ALU;


      end
      `OPC_LOAD: begin

        ImmSel = `IMMSEL_I;

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_DATA;


      end

      // Arithmetic instructions
      `OPC_ARI_RTYPE: begin

        ImmSel = `IMMSEL_I; //we won't be using this so shouldn't matter

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_REG;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_ALU;


      end
      `OPC_ARI_ITYPE: begin

        ImmSel = `IMMSEL_I;

        a_sel_next = `ASEL_REG;
        b_sel_next = `BSEL_IMM;

        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_ENABLE;

        wb_sel_next = `WBSEL_ALU;


      end

	    //For CSR
      `OPC_SYSTEM: begin

        ImmSel = `IMMSEL_U; //doesn't matter
        a_sel_next = `ASEL_REG; //doesn't matter
        b_sel_next = `BSEL_REG; //doesn't matter
        dcache_we_next = `WRITE_DISABLE;
        regfile_we_next = `WRITE_DISABLE;
        wb_sel_next = `WBSEL_ALU; //doesn't matter




	      csr_we_next = ~Inst_Kill;

    	  if(func3 == `FNC_CSRRW) begin
    		  csr_sel_next = `CSRSEL_A;
	      end else if(func3 == `FNC_CSRRWI) begin
    		  csr_sel_next = `CSRSEL_IMM;
      	end
	     end


      default: begin
        ImmSel = 0;

        a_sel_next = 0;
        b_sel_next = 0;

        dcache_we_next = 0;
        regfile_we_next = 0;

        wb_sel_next = 0;
      end


      endcase

    //end // else
  end // always block

endmodule



/*
  NOTES:
    LOAD HAZARD:
      1. Control knows by seeing that regfile_we (one ahead) is on and rd matches any of the rs
      2. turns on the appropiate bypass (no delay)
      3. HAS TO BE DELAYED TWICE? SINCE THE SECOND INSTRUCTION WOULD HAVE PASSED THE FIRST STAGE BY THEN?
      4. a nop should not have its regfile_we enabled so we should never read from a nop

    WRITE ENABLES:
      I think they are delay one lest than it would seem meaning:
        - dcache_we should be set such that when its respective instruction reaches the end of stage X, the write_enable bit
          should be set before the clock rises again (going into stage M) since the write is expected to happen on that clock
          edge.
        - similar case for regfile_we


*/
