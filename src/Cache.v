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
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr, //[29:2]
  output                           mem_req_rw,
  output                           mem_req_data_valid,
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);

//Define state bits
parameter WEB_READ = 1'b1;
parameter WEB_WRITE = 1'b0;
parameter MEMORY_READ = 1'b0;
parameter MEMORY_WRITE = 1'b1; 
parameter INIT = 3'b000;
parameter READ = 3'b001;
parameter WRITE = 3'b010;
parameter READ_MEM = 3'b011;
parameter WRITE_HIT = 3'b100;
parameter READ_MEM_WAIT = 3'b101;
parameter WRITE_WAIT = 3'b110;


//SRAM signals
reg data_web;
reg [127:0] data_write;
wire [127:0] data_read;
reg [15:0] data_bytemask;
reg [7:0] data_addr_input;

reg tag_web;
reg [31:0] tag_write; 
wire [31:0] tag_read;
reg [7:0] tag_addr_input;



wire [7:0] cpu_data_addr_new;
wire [7:0] cpu_tag_addr_new;
wire [22:0] cpu_tag_new;
wire [1:0] cpu_word_offset_new;
wire [27:0] cpu_mem_addr_new;

assign cpu_data_addr_new = {{3'b000},cpu_req_addr[6:2]};
assign cpu_tag_addr_new = {{5'd0},cpu_req_addr[6:4]};
assign cpu_tag_new = cpu_req_addr[31:9];
assign cpu_word_offset_new = cpu_req_addr[1:0];
assign cpu_mem_addr_new = cpu_req_addr[31:4];

wire [22:0] cpu_tag;
reg [22:0] cpu_tag_next;
wire [7:0] tag_addr;
reg [7:0] tag_addr_next;
wire [7:0] data_addr;
reg [7:0] data_addr_next;
wire [1:0] word_offset;
reg [1:0] word_offset_next;
wire [27:0] mem_addr;
reg [27:0] mem_addr_next;
wire[31:0] cpu_write_data;
reg[31:0] cpu_write_data_next;
reg [15:0] data_bytemask_new;

REGISTER_R #(.N(23)) cpu_tag_reg(.q(cpu_tag), .d(cpu_tag_next), .rst(reset), .clk(clk))
REGISTER_R #(.N(8)) tag_addr_reg(.q(tag_addr), .d(tag_addr_next), .rst(reset), .clk(clk))
REGISTER_R #(.N(8)) data_addr_reg(.q(data_addr), .d(data_addr_next), .rst(reset), .clk(clk))
REGISTER_R #(.N(2)) word_offset_reg(.q(word_offset), .d(word_offset_next), .rst(reset), .clk(clk))
REGISTER_R #(.N(28)) med_addr_reg(.q(mem_addr), .d(med_addr_next), .rst(reset), .clk(clk))
REGISTER_R #(.N(32)) cpu_write_data_reg(.q(cpu_write_data), .d(cpu_write_data_next), .rst(reset), .clk(clk))


wire[1:0] low2_mem_bits, low2_mem_bits_increment;
assign low2_mem_bits = mem_addr[1:0];
assign low2_mem_bits_increment = low2_mem_bits + 2'b01;

reg [31:0] data_read_word;
reg [127:0] shifted_data, shifted_data_new;
always@(*) begin
	case(cpu_word_offset_new)
		2'b00: begin
			//data_read_word = data_read[31:0];
			data_bytemask_new = 16'h000f;
			shifted_data_new = {{96'd0},cpu_req_data};
		end
		2'b01: begin
			//data_read_word  = data_read[63:32];
			data_bytemask_new = 16'h00f0;
			shifted_data_new = {{64'd0},cpu_req_data,{32'd0}};	
		end
		2'b10: begin
			//data_read_word  = data_read[95:64];
			data_bytemask_new = 16'h0f00;
			shifted_data_new = {{32'd0},cpu_req_data,{64'd0}};
		end
		2'b11: begin
			//data_read_word = data_read[127:96];
			data_bytemask_new = 16'hf000;
			shifted_data_new = {cpu_req_data,{96'd0}};
		end
	endcase
end

always@(*) begin
	case(word_offset)
		2'b00: begin
			data_read_word = data_read[31:0];
			data_bytemask = 16'h000f;
			shifted_data = {{96'd0},cpu_write_data};
		end
		2'b01: begin
			data_read_word  = data_read[63:32];
			data_bytemask = 16'h00f0;
			shifted_data = {{64'd0},cpu_write_data,{32'd0}};	
		end
		2'b10: begin
			data_read_word  = data_read[95:64];
			data_bytemask = 16'h0f00;
			shifted_data = {{32'd0},cpu_write_data,{64'd0}};
		end
		2'b11: begin
			data_read_word = data_read[127:96];
			data_bytemask = 16'hf000;
			shifted_data = {cpu_write_data,{96'd0}};
		end
	endcase
end


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
	.A(data_addr_input),
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
	.A(tag_addr_input),
	.I(tag_write),
	.O(tag_read)
)


wire [22:0] cache_tag;
wire valid;
assign cache_tag = tag_read[22:0]
assign valid = tag_read[23];

wire [2:0] state;
reg [2:0] next_state;
REGISTER_R #(.N(3)) state_reg(.q(state), .d(next_state), .rst(reset), .clk(clk));

wire [1:0] count;
reg [1:0] next_count;
REGISTER_R #(.N(2)) count_reg(.q(count), .d(next_count), .rst(reset), .clk(clk));

wire [31:0] temp_word;
reg [31:0] temp_word_next;
REGISTER_R #(.N(32)) count_reg(.q(temp_word), .d(temp_word_next), .rst(reset), .clk(clk));


always@(*) begin
	//Defaults
	mem_req_valid=0;
	next_count = 2'd0;
	cpu_req_ready = 0;
	tag_web = WEB_READ;
	data_web = WEB_READ;
	data_bytemask = 16'hffff;
	cpu_tag_next = cpu_tag;
	tag_addr_next = tag_addr;
	data_addr_next = data_addr;
	word_offset_next = word_offset;
	data_addr_input = data_addr;
	tag_addr_input = tag_addr;
	mem_addr_next = mem_addr;
	temp_word_next = temp_word;
	cpu_write_data_next = cpu_write_data;

	case(state)
		INIT: begin
			cpu_req_ready = 1;

			data_addr_next = cpu_data_addr_new;
			data_addr_input = cpu_data_addr_new;
			tag_addr_next = cpu_tag_addr_new;
			tag_addr = cpu_tag_addr_new;
			cpu_tag_next = cpu_tag_new;
			word_offset_next = cpu_word_offset_new;
			mem_addr_next = cpu_mem_addr_new;

			data_web=WEB_READ;
			tag_web=WEB_READ;

			if(cpu_req_valid) begin
				if(cpu_req_write==4'b0000) begin
					next_state=READ;				
				end else begin					
					if(mem_req_ready & mem_req_data_ready) begin
						next_state=WRITE;
						mem_req_valid = 1;
						mem_req_data_valid=1;
						mem_req_addr = cpu_mem_addr_new; 
						mem_req_data_bits = shifted_data_new;
						mem_req_data_mask = data_bytemask_new;
					end else begin
						next_state=WRITE_WAIT;
					end
				end
			else next_state = INIT;		
		end

		READ: begin
			if((cpu_tag == cache_tag)&&valid) begin //read hit
				cpu_resp_valid=1;
				cpu_req_ready=1; //still an issue here
			 	cpu_resp_data = data_read_word;
				next_state=INIT; //issues here			
			end else begin	//read miss
				cpu_req_ready=0;
				mem_req_rw=MEMORY_READ;
				next_count = 2'd0;
				tag_web = WEB_WRITE;
				tag_addr_input = tag_addr; 
				tag_write = {{8'd0},{1'b1},cpu_tag}; //zeros, valid=1, updated tag
				mem_addr_next={mem_addr[27:2],data_addr[1:0]}; 
				mem_req_addr = {mem_addr[27:2],data_addr[1:0]};
				if(mem_req_ready) begin
					next_state = READ_MEM;
					mem_req_valid = 1'b1;
				end else begin
					next_state = READ_MEM_WAIT;
					mem_req_valid = 1'b0;
				end
			end
		end

		READ_MEM_WAIT: begin
			mem_req_rw=MEMORY_READ;
			next_count = count;
			if(mem_req_ready) begin
				next_state = READ_MEM;
				mem_req_addr = mem_addr;
				mem_req_valid = 1'b1;
			end else begin
				next_state = READ_MEM_WAIT;
				mem_req_addr = mem_addr;
				mem_req_valid = 1'b0;
			end
		end

		
		READ_MEM: begin
			if(mem_resp_valid) begin
				next_count = count + 1;
			
				mem_req_valid = 1'b1;
				mem_req_rw = MEMORY_READ;
				mem_addr_next={mem_addr[27:2],low2_mem_bits_increment};
				
				data_web = WEB_WRITE;			
				data_write = mem_resp_data;			
				data_addr_input = {data_addr[7:2],low2_mem_bits}
				data_addr_next = {data_addr[7:2], low2_mem_bits_increment};
				
				if(data_addr[1:0]==mem_addr[1:0]) begin //not 100% sure
					temp_word_next = mem_resp_data[
					case(word_offset)
						2'b00: temp_word_next = mem_resp_data[31:0];
						2'b01: temp_word_next  = mem_resp_data[63:32];
						2'b10: temp_word_next  = mem_resp_data[95:64];
						2'b11: temp_word_next = mem_resp_data[127:96];
					endcase
				end



				if(count==2'b11) begin
					next_state = INIT; //still questions here
					cpu_resp_valid = 1;
					cpu_req_ready = 1; //not sure
					cpu_resp_data = temp_word;
				end else if(mem_req_ready) begin
					next_state = READ_MEM;
				else begin
					next_state - READ_MEM_WAIT;
				end		
			end else begin
				next_state = READ_MEM;
			end
		end


		WRITE: begin
			if((cpu_tag == cache_tag)&&valid) begin //write hit
				next_state = WRITE_HIT;
				data_web = WEB_WRITE;
				data_addr_input = data_addr;
				data_write = shifted_data;
			end else begin //write miss
				next_state=INIT;
				cpu_req_ready = 1; //not sure		
			end
		end

		WRITE_HIT: begin
			next_state=INIT;
			cpu_req_ready = 1; //not sure

		end


		WRITE_WAIT: begin
			if(mem_req_ready & mem_req_data_ready) begin
				next_state=WRITE;
				mem_req_valid = 1;
				mem_req_data_valid=1;
				mem_req_addr = cpu_mem_addr_new; 
				mem_req_data_bits = shifted_data_new;
				mem_req_data_mask = data_bytemask_new;
			end else begin
				next_state=WRITE_WAIT;
			end

		end
	endcase
end


endmodule
