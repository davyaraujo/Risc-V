module ULA(input [7:0] SrcA,SrcB, 
           input [2:0] ULAcontrol,
           output reg [7:0] ULAresult,
           output reg FlagZ);
  
  always @ (*)
    begin
      case(ULAcontrol)
        3'b000 : ULAresult = SrcA + SrcB;
        3'b001 : ULAresult = SrcA - SrcB;
        3'b010 : ULAresult = SrcA & SrcB;
        3'b011 : ULAresult = SrcA | SrcB;
        3'b101 : ULAresult = (SrcA < SrcB) ? 1 : 0;
		  default : ULAresult = 8'd0;
      endcase
      if(ULAresult == 8'd0)
        FlagZ <= 1'b1;
      else
        FlagZ <= 1'b0;
    end
  
endmodule