module Mux2 #(parameter N=8)(input [N-1:0] i0,i1 ,input sel ,output reg [N-1:0] out);

always @ (i0,i1,sel)
	case(sel)
		1'b0 : out = i0;
		1'b1 : out = i1;
		default : out = 0;
	endcase
endmodule