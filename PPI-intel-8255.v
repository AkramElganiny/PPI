/**
	# NOTES
	
	## TODO
	--- BSR TB
	--- Reset pin not working 
**/


module control_word_reg (in , out, en);

	input [7:0]in;	output [3:0]out;	input wire en;

	reg [3:0] word;
	assign out = word;

	always @(in,en) begin
		if(en) begin
			if(in[7]) begin
				word[0] <= ~in[4];
				word[1] <= ~in[1];
				word[2] <= ~in[0] | ~in[3];
				word[3] <= in[7];
			end else begin 
				word[2] <= 1'b1;
				word[0] <= 1'b0;
				word[1] <= 1'b0;


				word[3] <= in[7];
			end
		end
	end
endmodule

/** ** ** ** ** ** ** **/

module control_unit (CS,RD,WR,A0,A1,control, ic_mode);

	input CS,RD,WR,A0,A1,ic_mode;	output [3:0]control;
	
	reg [3:0]control_reg;
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
					if(ic_mode == 0) begin
						control_reg[2] <= 1'b1;
						control_reg[3] <= 1'b1;

						
					end else begin 
						control_reg[3] <= 1'b1;

						control_reg[0] <= 1'b0;
						control_reg[1] <= 1'b0;
						control_reg[2] <= 1'b0;
					end
				end // if(RD || WR)end
			end
		end
		else begin
			control_reg = 4'b0000;
		end
	end // always @(CS,RD,WR,A0,A1,RESET)
endmodule

/** ** ** ** ** ** ** **/

module Tri_State_Buffer (in, out, enable);

	input [7:0]in ;
	input enable;
	output [7:0]out;

	assign out = enable? in : 8'bzzzzzzzz;

endmodule

/** ** ** ** ** ** ** **/

module port8 (port , databus, mode , enable);

	inout [7:0]port;	inout [7:0]databus;		input mode , enable , en1 , en2;

	assign en1 = enable & (~mode);
	assign en2 = enable & (mode);

	Tri_State_Buffer t2(port , databus , en1);
	Tri_State_Buffer t1(databus , port , en2);
endmodule

/** ** ** ** ** ** ** **/

module decoder3to8(Data_in,Data_out,SetReset);

    input [2:0] Data_in;
    output[7:0] Data_out;
	input wire SetReset;

	assign Data_out[0] = (Data_in == 3'b000)? SetReset : Data_out[0];
	assign Data_out[1] = (Data_in == 3'b001)? SetReset : Data_out[1];
	assign Data_out[2] = (Data_in == 3'b010)? SetReset : Data_out[2];
	assign Data_out[3] = (Data_in == 3'b011)? SetReset : Data_out[3];
	assign Data_out[4] = (Data_in == 3'b100)? SetReset : Data_out[4];
	assign Data_out[5] = (Data_in == 3'b101)? SetReset : Data_out[5];
	assign Data_out[6] = (Data_in == 3'b110)? SetReset : Data_out[6];
	assign Data_out[7] = (Data_in == 3'b111)? SetReset : Data_out[7];


endmodule

module Top_module (PA,PB,PC,PD,CS,A,RST,RD,WR);

	inout [7:0] PA;	inout [7:0] PB;	inout [7:0] PC;	inout [7:0] PD;	input wire CS,RST,RD,WR;	input wire [1:0] A; /* IC interface */

	wire [7:0] databus; /* internal bus between Port A,B,C,D in mode 0*/
	wire [7:0] BSR_Out; /* value of port c in BSR mode */
	wire [7:0] databusOrBsr; /* carry port c data bus for each mode BSR or Mode 0 */
	assign databusOrBsr = ((ctrl_word_reg_out[3]) && A[0] && A[1]) ? BSR_Out : databus  ;
	
	output wire [3:0] port_enable; /* each bit connected to enable pin of each port A,B,C */ 
	assign PD_mode = (~RD) ? 1'b1 : (~WR) ? 1'b0 : 1'b0; /* Port D enable as read or write*/

	output wire [3:0] ctrl_word_reg_out; /* carry control word for port mode select */
	output wire ctrl_word_reg_enable; /* Enable control word reg when A0 = 1 A1 = 1 For control */
	assign ctrl_word_reg_enable = (A[0] && A[1]);

	
	port8 PortA(PA,databus		,ctrl_word_reg_out[0]  	,port_enable[0]);
	port8 PortB(PB,databus		,ctrl_word_reg_out[1]  	,port_enable[1]);
	port8 portC(PC,databusOrBsr	,ctrl_word_reg_out[2]  	,port_enable[2]);
	port8 portD(PD,databus		,PD_mode  				,port_enable[3]);
	
	decoder3to8 BSR_Decoder(PD[3:1],BSR_Out,PD[0]); /* setting or reseting port c pins */

	control_word_reg ctrl_word_reg (PD  , ctrl_word_reg_out, ctrl_word_reg_enable);

	control_unit ctrl(CS,RD,WR,A[0],A[1],port_enable, PD[7]);

	//assign ctrl_word_reg_out = (~RST) ? 3'b0 : ctrl_word_reg_out;

endmodule

/** ** ** ** ** ** ** **/

module top_module_tb();

	reg [7:0] DeviceA;	reg [7:0] DeviceB;	reg [7:0] DeviceC;	reg [7:0] DeviceD;

	wire [7:0] PA;	wire [7:0] PB;	wire [7:0] PC;	wire [7:0] PD;
	reg cs,rd,wr,rst; 	reg [1:0]a;

	assign PA = (~rd && wr) ? DeviceA : 8'bzzzzzzzz;
	assign PB = (~rd && wr) ? DeviceB : 8'bzzzzzzzz;
	assign PC = (~rd && wr) ? DeviceC : 8'bzzzzzzzz;
	assign PD = (~wr && rd) ? DeviceD : 8'bzzzzzzzz;

	wire [7:0] temp;

	Top_module tpmd (PA,PB,PC,PD,cs,a,rst,rd,wr);

	initial begin
	
		cs = 1;		rd = 1;		wr = 1;		rst = 1;
		a[0] = 1'bz;
		a[1] = 1'bz;
		
		$monitor("rd %b wr %b ||| a0 %b a1 %b |||PD %b PA %b PB %b PC %b ",rd,wr,a[0],a[1],PD,PA,PB,PC);


		$display("********************WRITE*************************");
		#5
		cs <= 0;
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b1000_0000;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 0;
		a[1] <= 0;
		DeviceD <= 8'b0101_0101;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 0;
		DeviceD <= 8'b1001_1001;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 0;
		a[1] <= 1;
		DeviceD <= 8'b1111_1001;
		wr <= 0;
		rd <= 1;

		#5
		$display("********************READ*************************");
		
		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b1001_1011;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 0;
		a[1] <= 0;
		DeviceA <= 8'b1000_1000;
		DeviceB <= 8'b1001_1001;
		DeviceC <= 8'b1011_1101;
		rd <= 0;
		wr <= 1;

		#5
		a[0] <= 1;
		a[1] <= 0;
		DeviceA <= 8'b1000_1000;
		DeviceB <= 8'b1001_1001;
		DeviceC <= 8'b1011_1101;
		rd <= 0;
		wr <= 1;

		#5
		a[0] <= 0;
		a[1] <= 1;
		DeviceA <= 8'b1000_1000;
		DeviceB <= 8'b1001_1001;
		DeviceC <= 8'b1011_1101;
		rd <= 0;
		wr <= 1;
		

		#5
		$display("**********************BSR***********************");

		#5
		cs <= 0;
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0000_0000;
		wr <= 0;
		rd <= 1;

		#5
		cs <= 0;
		a[0] <= 0;
		a[1] <= 0;
		DeviceD <= 8'b0000_1111;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0000_1111;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0xxx_1111;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0xxx_0011;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0xxx_0001;
		wr <= 0;
		rd <= 1;

		#5
		a[0] <= 1;
		a[1] <= 1;
		DeviceD <= 8'b0xxx_1110;
		wr <= 0;
		rd <= 1;


	end

endmodule



