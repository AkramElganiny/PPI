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

module control_word_reg2 (in , out, en);

	input [0:7]in;
	output [0:7]out;
	input wire en;

	reg [0:2] word;

	assign out = (en) ? in : out;

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

module decoder();


endmodule

module decoder3to8(Data_in,Data_out,SetReset);

    input [0:2] Data_in;
    output [0:7] Data_out;
    reg [0:7] Data_out;
	input wire SetReset;

    always @(Data_in)
    case (Data_in) 
        3'b000 : Data_out[0] = SetReset;
        3'b001 : Data_out[1] = SetReset;
        3'b010 : Data_out[2] = SetReset;
        3'b011 : Data_out[3] = SetReset;
        3'b100 : Data_out[4] = SetReset;
        3'b101 : Data_out[5] = SetReset;
        3'b110 : Data_out[6] = SetReset;
        3'b111 : Data_out[7] = SetReset;
    endcase
endmodule

module Top_module (PA,PB,PC,PD,CS,A,RST,RD,WR , PD, ctrl_word_reg_out, ctrl_word_reg_enable);

	inout [0:7] PA;
	inout [0:7] PB;
	inout [0:7] PC;
	inout [0:7] PD;

	input wire CS,RST,RD,WR;
	input wire [0:1] A;

	wire [0:3] port_enable;
	// output wire [0:2] ctrl_word_reg_out;
	output wire [0:7] ctrl_word_reg_out;
	output [0:7] databus;
	wire [0:7] databusOrBsr;
	wire [0:7] BSR_Out;

	output wire ctrl_word_reg_enable;

	
	assign PD_mode = (~RD) ? 1'b1 : (~WR) ? 1'b0 : 1'b0;

	assign ctrl_word_reg_enable = (A[0] && A[1]);

	assign databusOrBsr = (~PD[7] && A[0] && A[1]) ? BSR_Out : databus;

	port8 PortA(PA,databus		, ctrl_word_reg_out[4]  	,port_enable[0]);
	port8 PortB(PB,databus		, ctrl_word_reg_out[1]  	,port_enable[1]);
	port8 portC(PC,databusOrBsr	, ctrl_word_reg_out[0]  	,port_enable[2]);
	port8 portD(PD,databus		, PD_mode  					,port_enable[3]);
	decoder3to8 BSR_Decoder(PD[1:3],BSR_Out,PD[0]);

	// control_word_reg ctrl_word_reg (PD  , ctrl_word_reg_out, ctrl_word_reg_enable);
	control_word_reg2 ctrl_word_reg (PD  , ctrl_word_reg_out, ctrl_word_reg_enable);

	control_unit ctrl(CS,RD,WR,A[0],A[1],port_enable, PD[7]);

	assign ctrl_word_reg_out = (~RST) ? 3'b0 : ctrl_word_reg_out;

endmodule




module top_module_tb();

	reg [0:7] DeviceA;	reg [0:7] DeviceB;	reg [0:7] DeviceC;	reg [0:7] DeviceD;

	wire [0:7] PA;	wire [0:7] PB;	wire [0:7] PC;	wire [0:7] PD;

	assign PA =(conected)	?	DeviceA	:8'bzzzzzzzz;
	assign PB =(conected)	?	DeviceB	:8'bzzzzzzzz;
	assign PC =(conected)	?	DeviceC	:8'bzzzzzzzz;
	assign PD =(~conected)	?	DeviceD	:8'bzzzzzzzz;

	reg cs,rd,wr,rst; 	reg [0:1]a;


	reg conected;
	wire [0:7] temp1;
	wire [0:7] temp2;
	wire  temp3;

	Top_module tpmd (PA,PB,PC,PD,cs,a,rst,rd,wr , temp1,temp2,temp3);

	initial begin
	
		conected <= 1'b1;
		cs = 1;
		rd = 1;
		wr = 1;
		rst = 1;
		a[0] = 1'bz;
		a[1] = 1'bz;
		
		$monitor(" DevA %b DevB %b DevC %b DevD %b | PA %b PB %b PC %b PD %b | temp1 %b temp2 %b temp3 %b | a0 %b a1 %b connected %b",DeviceA,DeviceB,DeviceC,DeviceD,PA,PB,PC,PD,temp1,temp2,temp3,a[0],a[1],conected);

		#5
		conected <= 1'b0;
		cs <= 0;
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0000_0001;
		wr <= 0;
		
		#5
		wr <= 1;

		#5
		a[0] <= 0;
		a[1] <= 1;
		DeviceD <= 8'b0101_0101;
		wr <= 0;

		#5
		conected <= 1'b1;


	end

endmodule



