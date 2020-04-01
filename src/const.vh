`ifndef CONST
`define CONST

`define MEM_DATA_BITS 128
`define MEM_TAG_BITS 5
`define MEM_ADDR_BITS 28
`define MEM_DATA_CYCLES 4

`define CPU_ADDR_BITS 32
`define CPU_INST_BITS 32
`define CPU_DATA_BITS 32
`define CPU_OP_BITS 4
`define CPU_WMASK_BITS 16
`define CPU_TAG_BITS 15

// PC address on reset
`define PC_RESET 32'h00002000

// The NOP instruction
`define INSTR_NOP {12'd0, 5'd0, `FNC_ADD_SUB, 5'd0, `OPC_ARI_ITYPE}

`define CSR_TOHOST 12'h51E
`define CSR_HARTID 12'h50B
`define CSR_STATUS 12'h50A


// Controller Constants
`define PCSEL_PLUS4 2'b00
`define PCSEL_ALU 2'b01
`define PCSEL_SAME 2'b10

`define IMMSEL_I 3'b000
`define IMMSEL_S 3'b001
`define IMMSEL_SB 3'b010
`define IMMSEL_U 3'b011
`define IMMSEL_UJ 3'b100

`define BYPASS_FALSE 1'b0
`define BYPASS_TRUE 1'b1
`define BYPASS_CURR 1'b0
`define BYPASS_DELAY 1'b1

`define ASEL_REG 1'b0
`define ASEL_PC 1'b1
`define BSEL_REG 1'b0
`define BSEL_IMM 1'b1

`define CSRSEL_A 1'b0
`define CSRSEL_IMM 1'b1

// FOR BOTH STORE AND LOAD
/* UNUSED
`define DATA_SIZE_B 2b'00
`define DATA_SIZE_H 2b'01
`define DATA_SIZE_W 2b'10
`define DATA_UNSIGNED 1'b0
`define DATA_SIGNED 1'b1
*/

`define WBSEL_DATA 2'b00
`define WBSEL_ALU 2'b01
`define WBSEL_PC4 2'b10

`define WRITE_DISABLE 1'b0
`define WRITE_ENABLE 1'b1


`define WRITE_DISABLE4 4'b0000
`define WRITE_ENABLE4 4'b1111


`endif //CONST
