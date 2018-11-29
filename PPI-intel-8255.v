module control_word_reg (in , out, en);

	input [0:7]in;
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
module control_unit (CS,RD,WR,A0,A1,control, ic_mode);

	input CS,RD,WR,A0,A1,ic_mode;
	output [0:3]control;
	reg [0:3]control_reg;

	assign control = control_reg;

	always @(CS,RD,WR,A0,A1) begin
		
		if(~CS)begin 
			if(~A0 && ~A1) begin // p||t A
				if(~RD || ~WR) begin
					control_reg[0] <= 1'b1;
					control_reg[3] <= 1'b1;

					control_reg[1] <= 1'b0;
					control_reg[2] <= 1'b0;
				end // if(RD || WR)end
			end
			if(A0 && ~A1) begin // p||t B
				if(~RD || ~WR) begin
					control_reg[1] <= 1'b1;
					control_reg[3] <= 1'b1;

					control_reg[0] <= 1'b0;
					control_reg[2] <= 1'b0;
				end // if(RD || WR)end
			end
			if(~A0 && A1) begin // p||t C
				if(~RD || ~WR) begin
					control_reg[2] <= 1'b1;
					control_reg[3] <= 1'b1;

					control_reg[0] <= 1'b0;
					control_reg[1] <= 1'b0;
				end // if(RD || WR)end
			end
			if(A0 && A1) begin // p||t D to ctrl-w||d-reg
				if(~RD || ~WR) begin
					control_reg[3] <= 1'b1;

					control_reg[0] <= 1'b0;
					control_reg[1] <= 1'b0;
					control_reg[2] <= 1'b0;
					if(~ic_mode) begin
						control_reg[2] <= 1'b1;
					end
				end // if(RD || WR)end
			end
		end
		else begin
			control_reg = 4'b0000;
		end
	end // always @(CS,RD,WR,A0,A1,RESET)


endmodule

module Tri_State_Buffer (in, out, enable);

	input [0:7]in ;
	input enable;
	output [0:7]out;

	assign out = enable? in : 8'bzzzzzzzz;

endmodule


module port8 (port , databus, mode , enable);

	inout [0:7]port;
	inout [0:7]databus;
	input mode , enable , en1 , en2;
	

	assign en1 = enable & (~mode);
	assign en2 = enable & (mode);

	Tri_State_Buffer t2(port , databus , en1);
	Tri_State_Buffer t1(databus , port , en2);

	

endmodule


module Top_module (PA,PB,PC,PD,CS,A,RST,RD,WR);

	inout [0:7] PA;
	inout [0:7] PB;
	inout [0:7] PC;
	inout [0:7] PD;

	input wire CS,RST,RD,WR;
	input wire [0:1] A;

	wire [0:3] port_enable;
	wire [0:2] ctrl_word_reg_out;
	wire [0:7] databus;

	assign PD_mode = RD ? 1'b1 : WR ? 1'b0 : 1'b0;
	assign ctrl_word_reg_enable = A[0] & A[1];

	port8 PortA(PA,databus, ctrl_word_reg_out[0]  	,port_enable[0]);
	port8 PortB(PB,databus, ctrl_word_reg_out[1]  	,port_enable[1]);
	port8 portC(PC,databus, ctrl_word_reg_out[2]  	,port_enable[2]);
	port8 portD(PD,databus, PD_mode  				,port_enable[3]);

	control_word_reg ctrl_word_reg (PD  , ctrl_word_reg_out, ctrl_word_reg_enable);

	control_unit ctrl(CS,RD,WR,A[0],A[1],port_enable, PD[7]);

	assign ctrl_word_reg_out = (~RST) ? 3'b0 : ctrl_word_reg_out;

endmodule