`include "const.vh"

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8)
)
(
  input clk,
  input reset,

  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output                      cpu_resp_valid,
  output [CPU_WIDTH-1:0]      cpu_resp_data,

  output                      mem_req_valid,
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr,
  output                           mem_req_rw,
  output                           mem_req_data_valid,
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);


//SRAMS
wire data_web;
wire [7:0] data_addr;
wire [127:0] data_write;
wire [127:0] data_read;
wire [15:0] data_bytemask;

wire tag_web;
wire [7:0] tag_addr;
wire [31:0] tag_write; 
wire [31:0] tag_read;

/*********************************************************************
*  Filename      : SRAM1RW256x128.v                                 *
*  *  SRAM name     : SRAM1RW256x128                                   *
*  *  Word width    : 128   bits                                        *
*  *  Word number   : 256                                               *
*  *  Adress width  : 8     bits                                        *
*  **********************************************************************/
SRAM1RW256x128 dataSRAM(
	.CE(clk),
	.OEB(1'b0),
	.WEB(data_web),
	.CSB(1'b0),
	.BYTEMASK(data_bytemask),
	.A(data_addr),
	.I(data_write),
	.O(data_read)
)

/*********************************************************************
 * *  Filename      : SRAM1RW256x32.v                                  *
 * *  SRAM name     : SRAM1RW256x32                                    *
 * *  Word width    : 32    bits                                        *
 * *  Word number   : 256                                               *
 * *  Adress width  : 8     bits                                        *
 * **********************************************************************/
SRAM1RW256x32 tagSRAM(
	.CE(clk),
	.OEB(1'b0),
	.WEB(tag_web),
	.CSB(1'b0),
	.A(tag_addr),
	.I(tag_write),
	.O(tag_read)
)


wire [22:0] cache_tag, cpu_tag_next, cpu_tag;
wire valid;
assign cache_tag = tag_read[22:0]
assign valid = tag_read[23];
assign cpu_tag_next = cpu_reg_data[31:9];
REGISTER_R #(.N(23)) cpu_tag_reg(.q(cpu_tag), .d(cpu_tag_next), .rst(reset), .clk(clk))


//Controller/state machine
//Define state bits
parameter WEB_READ = 1'b1;
parameter WEB_WRITE = 1'b0;
parameter IDLE = 3'b000;
parameter READ = 3'b001;
parameter WRITE = 3'b10;

wire [2:0] state;
reg [2:0] next_state;
REGISTER_R #(.N(3)) state_reg(.q(state), .d(next_state), .rst(reset), .clk(clk));


always@(*) begin
	case(state)
		IDLE: begin
			cpu_req_ready = 1;
			data_addr = cpu_req_addr;
			tag_addr = cpu_req_addr;

			if(cpu_req_valid) begin
				if(cpu_req_write==4'b0000) begin //read
					next_state=READ;
					data_web=WEB_READ;
					tag_web = WEB_READ;										
				end else begin //write
					next_state=WRITE;
					data_web=WEB_WRITE;
					tag_web=WEB_READ;
				end
			else next_state = IDLE;		
		end

		READ: begin
			if((cpu_tag == cache_tag)&&valid) begin
				cpu_resp_valid=1;
				
			end else begin	

		end

		WRITE: begin

		end


	endcase
end



endmodule
