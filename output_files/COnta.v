module Contador(input clock1,reset, output reg [3:0]saida);

always @ (posedge clock1,negedge reset)
begin

	if(!reset)
	begin
		saida <= 4'b0;
	end
	else
		if(saida >= 4'h9)
			saida <= 4'b0;
		else
			saida <= saida + 1'd1;
end

endmodule