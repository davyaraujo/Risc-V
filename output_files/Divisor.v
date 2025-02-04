module FreqClockDivider #(parameter frequencia_input = 26'd50_000_000)(input clk1 , output reg clk2);

reg  [25:0] contador = 26'd0;

always @ (posedge clk1)
begin 
	contador = contador + 1'd1;
	if(contador >= frequencia_input/2)
	begin
		if(clk2)
			clk2 <= 0;
		else
			clk2 <=1;
		contador <= 0;
	end
end

endmodule