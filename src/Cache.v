`include "const.vh"
`include "util.vh"

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8),
  parameter WAYS = 2
)
(
  input clk,
  input reset,
  input                       cpu_req_valid,
  output reg                  cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output reg                  cpu_resp_valid,
  output reg [CPU_WIDTH-1:0]      cpu_resp_data,

  output reg                      mem_req_valid,
  input                       mem_req_ready,
  output reg [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr, //[29:2]
  output reg                          mem_req_rw,
  output reg                          mem_req_data_valid,
  input                            mem_req_data_ready,
  output reg [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output reg [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

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
parameter WRITE_DONE = 3'b111;
parameter READ_MEM = 3'b010;
parameter READ_MEM_WAIT = 3'b011;
parameter WRITE_WAIT = 3'b101;
parameter READ_MEM_DONE = 3'b100;
parameter WRITE_DATA_WAIT = 3'b110;

//CPU signals (same for each SRAM)
wire [7:0] cpu_data_addr_new;
wire [5:0] cpu_tag_addr_new;
wire [22:0] cpu_tag_new;
wire [1:0] cpu_word_offset_new;
wire [27:0] cpu_mem_addr_new;
wire [3:0] cpu_write_mask_new;

assign cpu_data_addr_new = {{3'b000},cpu_req_addr[6:2]};
assign cpu_tag_addr_new = {{3'd0},cpu_req_addr[6:4]};
assign cpu_tag_new = cpu_req_addr[29:7];
assign cpu_word_offset_new = cpu_req_addr[1:0];
assign cpu_mem_addr_new = cpu_req_addr[29:2];
assign cpu_write_mask_new = cpu_req_write;

wire [22:0] cpu_tag;
reg [22:0] cpu_tag_next;
wire [5:0] tag_addr;
reg [5:0] tag_addr_next;
wire [7:0] data_addr;
reg [7:0] data_addr_next;
wire [1:0] word_offset;
reg [1:0] word_offset_next;
wire [27:0] mem_addr;
reg [27:0] mem_addr_next;
wire[31:0] cpu_write_data;
reg[31:0] cpu_write_data_next;
wire [3:0] cpu_write_mask;
reg [3:0] cpu_write_mask_next;

REGISTER_R #(.N(23)) cpu_tag_reg(.q(cpu_tag), .d(cpu_tag_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(6)) tag_addr_reg(.q(tag_addr), .d(tag_addr_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(8)) data_addr_reg(.q(data_addr), .d(data_addr_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(2)) word_offset_reg(.q(word_offset), .d(word_offset_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(28)) mem_addr_reg(.q(mem_addr), .d(mem_addr_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(32)) cpu_write_data_reg(.q(cpu_write_data), .d(cpu_write_data_next), .rst(reset), .clk(clk));
REGISTER_R #(.N(4)) cpu_write_mask_reg(.q(cpu_write_mask), .d(cpu_write_mask_next), .rst(reset), .clk(clk));

wire[1:0] low2_mem_bits_increment, low2_data_bits_increment;
assign low2_mem_bits_increment = mem_addr[1:0] + 2'b01;
assign low2_data_bits_increment = data_addr[1:0] + 2'b01;

wire [2:0] state;
reg [2:0] next_state;
REGISTER_R #(.N(3)) state_reg(.q(state), .d(next_state), .rst(reset), .clk(clk));

wire [1:0] count;
reg [1:0] next_count;
REGISTER_R #(.N(2)) count_reg(.q(count), .d(next_count), .rst(reset), .clk(clk));

wire [31:0] temp_word;
reg [31:0] temp_word_next;
REGISTER_R #(.N(32)) temp_word_reg(.q(temp_word), .d(temp_word_next), .rst(reset), .clk(clk));

reg [15:0] data_bytemask;
reg [5:0] tag_addr_input;
reg [7:0] data_addr_input;

reg [31:0] data_read_word;
reg [127:0] shifted_data;


generate
if(WAYS==1) begin

//SRAM specific signals
reg data_web;
reg [127:0] data_write;
wire [127:0] data_read;


reg tag_web;
reg [31:0] tag_write; 
wire [31:0] tag_read;

wire [31:0] cache_output;
reg [31:0] cache_output_next;

REGISTER_R #(.N(32)) cache_output_reg(.q(cache_output), .d(cache_output_next), .rst(reset), .clk(clk));

//Word width: 128bits, word number: 256, address width: 8 bits
SRAM1RW256x128 dataSRAM(
	.CE(clk),
	.OEB(1'b0),
	.WEB(data_web),
	.CSB(1'b0),
	.BYTEMASK(data_bytemask),
	.A(data_addr_input),
	.I(data_write),
	.O(data_read)
);

//Word width: 32bits, Word number: 64, Address width: 6 bits
SRAM1RW64x32 tagSRAM(
	.CE(clk),
	.OEB(1'b0),
	.WEB(tag_web),
	.CSB(1'b0),
	.A(tag_addr_input),
	.I(tag_write),
	.O(tag_read)
);

wire [22:0] cache_tag;
wire valid;
assign cache_tag = tag_read[22:0];
assign valid = tag_read[23];


always@(*) begin
	case(word_offset)
		2'b00: begin
			data_read_word = data_read[31:0];
			data_bytemask = {{12'h000},cpu_write_mask};
			shifted_data = {{96'd0},cpu_write_data};
		end
		2'b01: begin
			data_read_word  = data_read[63:32];
			data_bytemask = {{8'h00},cpu_write_mask,{4'h0}};
			shifted_data = {{64'd0},cpu_write_data,{32'd0}};	
		end
		2'b10: begin
			data_read_word  = data_read[95:64];
			data_bytemask = {{4'h0},cpu_write_mask,{8'h00}};
			shifted_data = {{32'd0},cpu_write_data,{64'd0}};
		end
		2'b11: begin
			data_read_word = data_read[127:96];
			data_bytemask = {cpu_write_mask,{12'h000}};
			shifted_data = {cpu_write_data,{96'd0}};
		end
	endcase

	//Defaults
	mem_req_valid=0;
	next_count = 2'd0;
	cpu_req_ready = 0;
	tag_web = WEB_READ;
	data_web = WEB_READ;
	data_write = 32'd0;
	tag_write = 32'd0;
	cpu_tag_next = cpu_tag;
	tag_addr_next = tag_addr;
	data_addr_next = data_addr;
	word_offset_next = word_offset;
	data_addr_input = data_addr;
	tag_addr_input = tag_addr;
	mem_addr_next = mem_addr;
	temp_word_next = temp_word;
	cpu_write_data_next = cpu_write_data;
	cpu_write_mask_next = cpu_write_mask;
	cpu_req_ready = 1'b0;
	cpu_resp_valid = 1'b0;
	cpu_resp_data = cache_output;
	mem_req_valid = 1'b0;
	mem_req_addr = 0;
	mem_req_rw = MEMORY_READ;
	mem_req_data_valid = 1'b0;
	mem_req_data_bits = 128'd0;
	mem_req_data_mask = 16'hffff;		
	next_state = INIT;
	cache_output_next = cache_output;


	case(state)
		INIT: begin
			cpu_req_ready = 1;
			data_addr_next = cpu_data_addr_new;
			data_addr_input = cpu_data_addr_new;
			tag_addr_next = cpu_tag_addr_new;
			tag_addr_input = cpu_tag_addr_new;
			cpu_tag_next = cpu_tag_new;
			word_offset_next = cpu_word_offset_new;
			mem_addr_next = cpu_mem_addr_new;
			cpu_write_mask_next = cpu_write_mask_new;
			cpu_write_data_next = cpu_req_data;
			if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new;
							mem_req_rw = MEMORY_WRITE; 			 
						end else begin
							next_state=WRITE_WAIT;
						end
					end
			end else next_state = INIT;		
		end

		READ: begin
			if((cpu_tag == cache_tag)&&valid) begin //read hit
				cpu_resp_valid=1;
			 	cpu_resp_data = data_read_word;
				cache_output_next = data_read_word;

				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE;				
						end else begin
							next_state=WRITE_WAIT;
						end
					end
				end else next_state = INIT;			
				//END INIT BLOCK

			end else begin	//read miss
				cpu_req_ready=0;
				mem_req_rw=MEMORY_READ;
				next_count = 2'd0;
				tag_web = WEB_WRITE;
				tag_addr_input = tag_addr; 
				tag_write = {{8'd0},{1'b1},cpu_tag}; //zeros, valid=1, updated tag
				mem_addr_next = mem_addr;
				mem_req_addr = mem_addr;
				data_addr_next = {data_addr[7:2],{2'b00}}; 
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
			mem_req_addr = mem_addr;
			if(mem_req_ready) begin
				next_state = READ_MEM;
				mem_req_valid = 1'b1;
			end else begin
				next_state = READ_MEM_WAIT;
				mem_req_valid = 1'b0;
			end
		end

		
		READ_MEM: begin
			if(mem_resp_valid) begin
				next_count = count + 1;
				
				//Write memory data just retrieved to cache
				data_web = WEB_WRITE;			
				data_write = mem_resp_data;			
				data_addr_input = data_addr;
				data_addr_next = {data_addr[7:2], low2_data_bits_increment};
				data_bytemask = 16'hffff;
	
				//Identify the word we need to return to CPU
				if(data_addr[1:0]==mem_addr[1:0]) begin //not 100% sure
					case(word_offset)
						2'b00: temp_word_next = mem_resp_data[31:0];
						2'b01: temp_word_next  = mem_resp_data[63:32];
						2'b10: temp_word_next  = mem_resp_data[95:64];
						2'b11: temp_word_next = mem_resp_data[127:96];
					endcase
				end
				
				if(count==2'b11) begin
					cpu_req_ready=0; //not ready since still writing to cache, will require a full cycle more
					next_state = READ_MEM_DONE;
				end else begin
					mem_req_rw = MEMORY_READ;			
					next_state=READ_MEM;
				end		
			end else begin
				next_state = READ_MEM;
			end
		end

		READ_MEM_DONE: begin
			cpu_resp_data=temp_word;
			cache_output_next = temp_word;
			cpu_resp_valid=1;
			cpu_req_ready=1;

				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE; 
						end else begin
							next_state=WRITE_WAIT;
						end
					end

				end else next_state = INIT;			
				//END INIT BLOCK
		end


		WRITE_DONE: begin
				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE; 
						end else begin
							next_state=WRITE_WAIT;
						end
					end
				end else next_state = INIT;			
				//END INIT block
		end

		WRITE_DATA_WAIT: begin
			if(mem_req_data_ready) begin
				mem_req_data_valid=1;
				mem_req_data_bits = shifted_data;
				mem_req_data_mask = data_bytemask;
				next_state = WRITE_DONE;

				if((cpu_tag == cache_tag)&&valid) begin //write hit
					data_web = WEB_WRITE;
					data_addr_input = data_addr;
					data_write = shifted_data;
					cpu_req_ready = 0;		
				end	

			end else begin
				next_state=WRITE_DATA_WAIT;
			end
		end


		WRITE_WAIT: begin
			//Need to keep reading the cache_tag while we wait
			tag_addr_next = tag_addr;
			tag_addr_input = tag_addr;
			tag_web=WEB_READ;
	
			if(mem_req_ready) begin
				next_state=WRITE_DATA_WAIT;
				mem_req_valid = 1;
				mem_req_addr = mem_addr;
				mem_req_rw = MEMORY_WRITE; 
			end else begin
				next_state=WRITE_WAIT;
			end

		end
	endcase
end


end else begin
//SRAM specific signals
reg data_web1,data_web2;
reg [127:0] data_write;
wire [127:0] data_read1,data_read2;
reg [127:0] data_read;
reg tag_web1, tag_web2;
reg [31:0] tag_write;
wire [31:0] tag_read1, tag_read2;

wire [31:0] cache_output;
reg [31:0] cache_output_next;

REGISTER_R #(.N(32)) cache_output_reg(.q(cache_output), .d(cache_output_next), .rst(reset), .clk(clk));

//Word width: 128bits, word number: 256, address width: 8 bits
SRAM1RW256x128 dataSRAM1(
	.CE(clk),
	.OEB(1'b0),
	.WEB(data_web1),
	.CSB(1'b0),
	.BYTEMASK(data_bytemask),
	.A(data_addr_input),
	.I(data_write),
	.O(data_read1)
);
SRAM1RW256x128 dataSRAM2(
	.CE(clk),
	.OEB(1'b0),
	.WEB(data_web2),
	.CSB(1'b0),
	.BYTEMASK(data_bytemask),
	.A(data_addr_input),
	.I(data_write),
	.O(data_read2)
);

//Word width: 32bits, Word number: 64, Address width: 6 bits
SRAM1RW64x32 tagSRAM1(
	.CE(clk),
	.OEB(1'b0),
	.WEB(tag_web1),
	.CSB(1'b0),
	.A(tag_addr_input),
	.I(tag_write),
	.O(tag_read1)
);
SRAM1RW64x32 tagSRAM2(
	.CE(clk),
	.OEB(1'b0),
	.WEB(tag_web2),
	.CSB(1'b0),
	.A(tag_addr_input),
	.I(tag_write),
	.O(tag_read2)
);


wire [22:0] cache_tag1, cache_tag2;
wire valid1, valid2;
assign cache_tag1 = tag_read1[22:0];
assign valid1 = tag_read1[23];
assign cache_tag2 = tag_read2[22:0];
assign valid2 = tag_read2[23];

wire random;
reg random_next;
REGISTER_R #(.N(1)) random_reg(.q(random), .d(random_next), .rst(reset), .clk(clk));
wire active_way;
reg active_way_next;
REGISTER_R #(.N(1)) active_way_reg(.q(active_way), .d(active_way_next), .rst(reset), .clk(clk));



always@(*) begin
	case(word_offset)
		2'b00: begin
			data_read_word = data_read[31:0];
			data_bytemask = {{12'h000},cpu_write_mask};
			shifted_data = {{96'd0},cpu_write_data};
		end
		2'b01: begin
			data_read_word  = data_read[63:32];
			data_bytemask = {{8'h00},cpu_write_mask,{4'h0}};
			shifted_data = {{64'd0},cpu_write_data,{32'd0}};	
		end
		2'b10: begin
			data_read_word  = data_read[95:64];
			data_bytemask = {{4'h0},cpu_write_mask,{8'h00}};
			shifted_data = {{32'd0},cpu_write_data,{64'd0}};
		end
		2'b11: begin
			data_read_word = data_read[127:96];
			data_bytemask = {cpu_write_mask,{12'h000}};
			shifted_data = {cpu_write_data,{96'd0}};
		end
	endcase

	//Defaults
	mem_req_valid=0;
	next_count = 2'd0;
	cpu_req_ready = 0;
	tag_web1 = WEB_READ;
	data_web1 = WEB_READ;
	tag_web2 = WEB_READ;
	data_web2 = WEB_READ;
	data_write = 32'd0;
	data_read = 128'd0;
	tag_write = 32'd0;
	cpu_tag_next = cpu_tag;
	tag_addr_next = tag_addr;
	data_addr_next = data_addr;
	word_offset_next = word_offset;
	data_addr_input = data_addr;
	tag_addr_input = tag_addr;
	mem_addr_next = mem_addr;
	temp_word_next = temp_word;
	cpu_write_data_next = cpu_write_data;
	cpu_write_mask_next = cpu_write_mask;
	cpu_req_ready = 1'b0;
	cpu_resp_valid = 1'b0;
	cpu_resp_data = cache_output;
	mem_req_valid = 1'b0;
	mem_req_addr = 0;
	mem_req_rw = MEMORY_READ;
	mem_req_data_valid = 1'b0;
	mem_req_data_bits = 128'd0;
	mem_req_data_mask = 16'hffff;		
	next_state = INIT;
	cache_output_next = cache_output;
	random_next = random;
	active_way_next = active_way;

	case(state)
		INIT: begin
			cpu_req_ready = 1;
			data_addr_next = cpu_data_addr_new;
			data_addr_input = cpu_data_addr_new;
			tag_addr_next = cpu_tag_addr_new;
			tag_addr_input = cpu_tag_addr_new;
			cpu_tag_next = cpu_tag_new;
			word_offset_next = cpu_word_offset_new;
			mem_addr_next = cpu_mem_addr_new;
			cpu_write_mask_next = cpu_write_mask_new;
			cpu_write_data_next = cpu_req_data;
			if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new;
							mem_req_rw = MEMORY_WRITE; 			 
						end else begin
							next_state=WRITE_WAIT;
						end
					end
			end else next_state = INIT;		
		end

		READ: begin
			if( ((cpu_tag==cache_tag1)&&valid1) || ((cpu_tag==cache_tag2)&&valid2) ) begin //read hit
				if( ((cpu_tag==cache_tag1)&&valid1)) begin
					data_read = data_read1;
				end else begin
					data_read = data_read2;
				end

				cpu_resp_valid=1;
			 	cpu_resp_data = data_read_word;
				cache_output_next = data_read_word;

				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE;				
						end else begin
							next_state=WRITE_WAIT;
						end
					end
				end else next_state = INIT;			
				//END INIT BLOCK

			end else begin	//read miss
				cpu_req_ready=0;
				mem_req_rw=MEMORY_READ;
				next_count = 2'd0;
				tag_addr_input = tag_addr; 
				tag_write = {{8'd0},{1'b1},cpu_tag}; //zeros, valid=1, updated tag
				
				if(!valid1&valid2) begin
					tag_web1=WEB_WRITE;
					active_way_next=1'b0;
				end else if(valid1&!valid2) begin
					tag_web2=WEB_WRITE;
					active_way_next=1'b1;
				end else if(random==0) begin
					tag_web1=WEB_WRITE;
					active_way_next=1'b0;
				end else begin
					tag_web2 = WEB_WRITE;	
					active_way_next=1'b1;
				end

				mem_addr_next = mem_addr;
				mem_req_addr = mem_addr;
				data_addr_next = {data_addr[7:2],{2'b00}}; 
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
			mem_req_addr = mem_addr;
			if(mem_req_ready) begin
				next_state = READ_MEM;
				mem_req_valid = 1'b1;
			end else begin
				next_state = READ_MEM_WAIT;
				mem_req_valid = 1'b0;
			end
		end

		
		READ_MEM: begin
			if(mem_resp_valid) begin
				next_count = count + 1;
				
				//Write memory data just retrieved to cache
				data_addr_input = data_addr;
				data_addr_next = {data_addr[7:2], low2_data_bits_increment};
				data_bytemask = 16'hffff;
				data_write = mem_resp_data;

				if(active_way==0) begin
					data_web1 = WEB_WRITE;						
				end else begin
					data_web2 = WEB_WRITE;
				end
					

				//Identify the word we need to return to CPU
				if(data_addr[1:0]==mem_addr[1:0]) begin //not 100% sure
					case(word_offset)
						2'b00: temp_word_next = mem_resp_data[31:0];
						2'b01: temp_word_next  = mem_resp_data[63:32];
						2'b10: temp_word_next  = mem_resp_data[95:64];
						2'b11: temp_word_next = mem_resp_data[127:96];
					endcase
				end
				
				if(count==2'b11) begin
					cpu_req_ready=0; //not ready since still writing to cache, will require a full cycle more
					next_state = READ_MEM_DONE;
				end else begin
					mem_req_rw = MEMORY_READ;			
					next_state=READ_MEM;
				end		
			end else begin
				next_state = READ_MEM;
			end
		end

		READ_MEM_DONE: begin
			cpu_resp_data=temp_word;
			cache_output_next = temp_word;
			cpu_resp_valid=1;
			cpu_req_ready=1;
			random_next = random+1;

				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
			
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE; 
						end else begin
							next_state=WRITE_WAIT;
						end
					end

				end else next_state = INIT;			
				//END INIT BLOCK
		end


		WRITE_DONE: begin
				//BEGIN INIT block
				cpu_req_ready=1;
				data_addr_next = cpu_data_addr_new;
				data_addr_input = cpu_data_addr_new;
				tag_addr_next = cpu_tag_addr_new;
				tag_addr_input = cpu_tag_addr_new;
				cpu_tag_next = cpu_tag_new;
				word_offset_next = cpu_word_offset_new;
				mem_addr_next = cpu_mem_addr_new;
				cpu_write_mask_next = cpu_write_mask_new;
				cpu_write_data_next = cpu_req_data;
				if(cpu_req_valid) begin
					if(cpu_req_write==4'b0000) begin
						next_state=READ;				
					end else begin					
						if(mem_req_ready) begin
							next_state=WRITE_DATA_WAIT;
							mem_req_valid = 1;
							mem_req_addr = cpu_mem_addr_new; 
							mem_req_rw = MEMORY_WRITE; 
						end else begin
							next_state=WRITE_WAIT;
						end
					end
				end else next_state = INIT;			
				//END INIT block
		end

		WRITE_DATA_WAIT: begin
			if(mem_req_data_ready) begin
				mem_req_data_valid=1;
				mem_req_data_bits = shifted_data;
				mem_req_data_mask = data_bytemask;
				next_state = WRITE_DONE;

				if((cpu_tag == cache_tag1)&&valid1) begin //write hit
					data_web1 = WEB_WRITE;
					data_addr_input = data_addr;
					data_write = shifted_data;		
				end else if((cpu_tag==cache_tag2)&&valid2) begin
					data_web2 = WEB_WRITE;
					data_addr_input = data_addr;
					data_write = shifted_data;		
				end		
			end else begin
				next_state=WRITE_DATA_WAIT;
			end
		end


		WRITE_WAIT: begin
			//Need to keep reading the cache_tag while we wait
			tag_addr_next = tag_addr;
			tag_addr_input = tag_addr;
			tag_web1=WEB_READ;
			tag_web2=WEB_READ;
	
			if(mem_req_ready) begin
				next_state=WRITE_DATA_WAIT;
				mem_req_valid = 1;
				mem_req_addr = mem_addr;
				mem_req_rw = MEMORY_WRITE; 
			end else begin
				next_state=WRITE_WAIT;
			end

		end
	endcase
end //always block


end //if-else statement in generate

endgenerate


endmodule
