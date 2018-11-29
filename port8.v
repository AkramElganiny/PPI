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


module port8_tb ();

	wire[0:7]port;
	wire [0:7]databus;
	reg mode , enable;
	reg [0:7]a;

	assign port =(~enable)? 8'bzzzzzzzz:(~mode)?a:8'bzzzzzzzz;
	assign databus = (~enable)? 8'bzzzzzzzz : (mode)?a : 8'bzzzzzzzz;
	



	port8 A (port , databus, mode , enable);

	initial begin 

		$monitor("port:%b databus:%b mode:%b enable:%b",port, databus, mode, enable);
		#5
		enable = 0;
		#5
		mode =1;
		#5
		mode = 1;
		a = 5;
		#5
		enable =1;
		mode = 0;
		#5
		a=6;
		#5
		mode = 1;
		#5
		enable=0;
		#5
		mode = 0;
		a=16;
		
		
		
	end




endmodule