module control_word_reg (in , out, en);

	input [0:7]in;
	output [0:7]out;
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

