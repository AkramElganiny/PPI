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


module control_tb ();


	reg CS,RD,WR,A0,A1;
	wire [0:3] out;

	control_unit A (CS,RD,WR,A0,A1,out);

	initial begin 

		$monitor("CS:%b RD:%b WR:%b A0:%b A1:%b out:%b ",CS,RD,WR,A0,A1,out);

		#5
		CS=1;
		RD=0;
		A0=0;
		A1=0;
		WR=1;
		#5
		CS=0;
		RD=0;
		A0=1;
		A1=1;

	end
	

endmodule