module decodificador(input [3:0] chaves ,input reset, output reg [6:0] segmentos);

always @(*)
	if (reset)
		segmentos = 7'b1111111;
	else
	case(chaves)
		4'h0 : segmentos = 7'b0000001;
		4'h1 : segmentos = 7'b1001111;
		4'h2 : segmentos = 7'b0010010;
		4'h3 : segmentos = 7'b0000110;
		4'h4 : segmentos = 7'b1001100;
		4'h5 : segmentos = 7'b0100100;
		4'h6 : segmentos = 7'b0100000;
		4'h7 : segmentos = 7'b0001111;
		4'h8 : segmentos = 7'b0000000;
		4'h9 : segmentos = 7'b0000100;
		4'hA : segmentos = 7'b0001000;
		4'hB : segmentos = 7'b1100000;
		4'hC : segmentos = 7'b0110001;
		4'hD : segmentos = 7'b1000010;
		4'hE : segmentos = 7'b0110000;
		default : segmentos = 7'b0111000;
	endcase
endmodule