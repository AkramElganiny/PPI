module control_word_reg (in , out, en);

	input [7:0]in;
	output [0:2]out;
	input wire en;

	reg [0:2] word;

	assign out = word;

	always @(in,en) begin
		if(en) begin
			
			if(in[7]) begin
				word[0] <= in[4];
				word[1] <= in[1];
				word[2] <= in[0] | in[3];
			end else begin
				word[2] <= 1'b1;
			end

		end
	end

endmodule

module control_word_reg2 (in , out, en);

	input [7:0]in;	output [7:0]out;	input wire en;

	assign out = (en) ? in : out;

endmodule

module control_word_reg_tb ();

	reg [7:0]in;	wire [7:0]out;	reg en;

	// control_word_reg b(in , out, en);
	control_word_reg2 b(in , out, en);

	initial begin

		$monitor("en %b in %b out %b",en,in,out);

		in = 8'b1000_0000;
		en = 1;

	end

endmodule

