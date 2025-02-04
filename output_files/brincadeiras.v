module brincadeira(input clock6hz,reset, output reg [27:0] out);

always @ (posedge clock6hz, negedge reset)
begin
	if (!reset)
		out <= {7'b0111111,7'b1111111,7'b1111111,7'b1111111};
		//out2 <= {out1[27],out1[20],out1[13],out1[6],out1[5],out1[4],out1[3], out1[10],out1[17],out1[24],out1[23],out1[22]};
	else
		//out2 <= {1'b1,out2[12:1]};	
		//out3 <= {out2[12],out1[25:21],out2[20],out1[19:14],out2[13],out1[12:7],out2[6:2],out1[1:0],out2[10],
		{out[27],out[20],out[13],out[6],out[5],out[4],out[3], out[10],out[17],out[24],out[23],out[22]} = {out[22],out[27],out[20],out[13],out[6],out[5],out[4],out[3], out[10],out[17],out[24],out[23]};
end
		
		
endmodule