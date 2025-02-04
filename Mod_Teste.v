`default_nettype none //Comando para desabilitar declaração automática de wires
module Mod_Teste (
//Clocks
input CLOCK_27, CLOCK_50,
//Chaves e Botoes
input [3:0] KEY,
input [17:0] SW,
//Displays de 7 seg e LEDs
output [0:6] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
output [8:0] LEDG,
output [17:0] LEDR,
//Serial
output UART_TXD,
input UART_RXD,
inout [7:0] LCD_DATA,
output LCD_ON, LCD_BLON, LCD_RW, LCD_EN, LCD_RS,
//GPIO
inout [35:0] GPIO_0, GPIO_1
);
assign GPIO_1 = 36'hzzzzzzzzz;
assign GPIO_0 = 36'hzzzzzzzzz;
assign LCD_ON = 1'b1;
assign LCD_BLON = 1'b1;
wire [7:0] w_d0x0, w_d0x1, w_d0x2, w_d0x3, w_d0x4, w_d0x5,
w_d1x0, w_d1x1, w_d1x2, w_d1x3, w_d1x4, w_d1x5;
LCD_TEST MyLCD (
.iCLK ( CLOCK_50 ),
.iRST_N ( KEY[0] ),
.d0x0(w_d0x0),.d0x1(w_d0x1),.d0x2(w_d0x2),.d0x3(w_d0x3),.d0x4(w_d0x4),.d0x5(w_d0x5),
.d1x0(w_d1x0),.d1x1(w_d1x1),.d1x2(w_d1x2),.d1x3(w_d1x3),.d1x4(w_d1x4),.d1x5(w_d1x5),
.LCD_DATA( LCD_DATA ),
.LCD_RW ( LCD_RW ),
.LCD_EN ( LCD_EN ),
.LCD_RS ( LCD_RS )
);
//---------- modifique a partir daqui --------

wire [7:0] w_rd1SrcA, w_rd2, w_SrcB, w_ULAResult,w_RegData;
wire [7:0] w_PCp4, w_PC, w_RData, w_Wd3, w_Imm, w_ImmPC, w_PCn;
wire [31:0] w_Inst;
wire w_RegWrite, w_ULASrc, w_MemWrite, 
w_Branch, w_Zero, w_PCSrc, w_Jal, w_JalR;
wire [2:0] w_ULAControl;
wire [1:0] w_ImmSrc, w_ResultSrc;
wire Clk_1, Clk_2;
wire [3:0] Count_;
wire w_SaidaBranch;

assign w_PCSrc = w_Branch && w_Zero || w_Jal;

registrador regi(.clk(Clk_1), .we3(w_RegWrite), .wa3(w_Inst[11:7]), .ra1(w_Inst[19:15]),
.ra2(w_Inst[24:20]), .wd3(w_Wd3), .rst(KEY[2]),.rd1(w_rd1SrcA), .rd2(w_rd2),
.x0(w_d0x0), .x1(w_d0x1), .x2(w_d0x2), .x3(w_d0x3), .x4(w_d1x0),
.x5(w_d1x1), .x6(w_d1x2), .x7(w_d1x3));


ProgamCounter teste_PC(.clk(Clk_1), .rst(KEY[2]), .PCin(w_PCn), .PC(w_PC[7:0]));

Memoria_instrucao teste_MI(.entrada(w_PC), .RD1(w_Inst[31:0]));

Unidade_controle teste_UC(.Op(w_Inst[6:0]), .Funct3(w_Inst[14:12]), .Funct7(w_Inst[31:25]), .ULAControl(w_ULAControl[2:0]),
.ULASrc(w_ULASrc), .RegWrite(w_RegWrite), .MemWrite(w_MemWrite), .ResultSrc(w_ResultSrc), .ImmSrc(w_ImmSrc), .Branch(w_Branch), .Jal(w_Jal), .JalR(w_JalR));

hex_to_7seg teste_hex1(.hex(w_Inst[31:28]),.seg(HEX7));
hex_to_7seg teste_hex2(.hex(w_Inst[27:24]),.seg(HEX6));
hex_to_7seg teste_hex3(.hex(w_Inst[23:20]),.seg(HEX5));
hex_to_7seg teste_hex4(.hex(w_Inst[19:16]),.seg(HEX4));
hex_to_7seg teste_hex5(.hex(w_Inst[15:12]),.seg(HEX3));
hex_to_7seg teste_hex6(.hex(w_Inst[11:8]),.seg(HEX2));
hex_to_7seg teste_hex7(.hex(w_Inst[7:4]),.seg(HEX1));
hex_to_7seg teste_hex8(.hex(w_Inst[3:0]),.seg(HEX0));


MUX2x1 muxinhoULArc(.in0(w_rd2), .in1(w_Imm), .sel(w_ULASrc), .saida(w_SrcB));
MUX4x1 muxinhoPCSrc(.in0(w_PCp4), .in1(w_ImmPC), .in2(w_ULAResult), .in3(w_ULAResult), .sel({w_JalR, w_PCSrc}), .saida(w_PCn)); //na 2 e na 3 pra n dar erro
MUX4x1 muxinhoMuxResSrc(.in0(w_ULAResult), .in1(w_RegData), .in2(w_PCp4), .in3('b0), .sel(w_ResultSrc), .saida(w_Wd3));
MUX4x1 muxinhoMuxImmSrc(.in0(w_Inst[31:20]), .in1({w_Inst[31:25],w_Inst[11:7]}), 
.in2({ w_Inst[7], w_Inst[30:25], w_Inst[11:8], 1'b0}), 
.in3({w_Inst[31], w_Inst[19:12], w_Inst[20], w_Inst[30:21], 1'b0}), .sel(w_ImmSrc), .saida(w_Imm));

ULA ulinha(.SrcA(w_rd1SrcA), .SrcB(w_SrcB), .ULAControl(w_ULAControl[2:0]), 
.FlagZ(w_Zero), .ULAResult(w_ULAResult));

somador4bits teste_somador(.A(w_PC), .B(3'h4), .soma(w_PCp4));
somador4bits somador_w_ImmpC(.A(w_Imm), .B(w_PC), .soma(w_ImmPC));

assign w_d0x4[7:0] = w_PC;
assign LEDR[5:3] = w_ULAControl[2:0];
assign LEDR[8] = w_RegWrite;
assign LEDR[7] = w_ImmSrc;
assign LEDR[6] = w_ULASrc;
assign LEDR[2] = w_MemWrite;
assign LEDR[1] = w_ResultSrc;
assign LEDR[0] = w_Branch;

/*assign LEDG[8] = ~KEY[1];
assign w_rd1SrcA = w_d0x0[7:0];
assign w_rd2 = w_d1x0[7:0];
assign w_SrcB = w_d1x1[7:0]; 
assign w_ULAResultWd3 = w_d0x4[7:0];
*/
RAM ranzinha(.A(w_ULAResult), .WD(w_rd2), .WE(w_MemWrite), .rst(KEY[2]), .clk(Clk_1), 
.RD(w_RData));

parrallel_in in(.Address(w_ULAResult),.MemData(w_RData),.DataIn(SW[7:0]),.RegData(w_RegData));

parallel_out out(.EN(w_MemWrite),.clk(Clk_1),.reset(KEY[2]),.RegData(w_rd2),.Address(w_ULAResult),.DataOut(w_d1x4));
			

//assign Clk_1 = KEY[1];
clock_divider teste_clock(.clk(CLOCK_50), .clk_1hz(Clk_1));
endmodule

module registrador(input [7:0] wd3,
                   input [2:0] wa3, ra1, ra2,
                   input we3, clk, rst,
                   output reg [7:0] rd1, rd2, x0,x1,x2,x3,x4,x5,x6,x7);
  reg [7:0] registradores [7:0];
  integer count, count1; 
  //registradores [0] = 0;
  
  always @ (posedge clk, negedge rst)
    begin
      if (!rst)
			  for (count = 0;count < 8; count=count+1)
				 registradores[count] = 0;
			
      else if ((we3 == 1) && (wa3 != 0))
        registradores [wa3] = wd3;
    end
	 
  always @(ra1, ra2, registradores)
    begin
      rd1 = registradores[ra1];
      rd2 = registradores[ra2];
		x0 = registradores[0];
		x1 = registradores[1];
		x2 = registradores[2];
		x3 = registradores[3];
		x4 = registradores[4];
		x5 = registradores[5];
		x6 = registradores[6];
		x7 = registradores[7];
    end
endmodule


module MUX2x1(input [7:0] in0, in1,
              input sel,
              output reg [7:0]saida);
  always @(*)
    begin
      case (sel)
        1'b0 : saida = in0;
        default : saida = in1;
      endcase
    end
    
endmodule

module MUX4x1(input [7:0] in0, in1, in2, in3,
              input [1:0] sel,
              output reg [7:0]saida);
  always @(*)
    begin
      casex (sel)
        2'b00 : saida = in0;
		  2'b01 : saida = in1;
		  2'b10 : saida = in2;
		  2'b11 : saida = in3;
        default : saida = 'bxxxxxxxx;
      endcase
    end
    
endmodule


module ULA(input [7:0] SrcA, SrcB,
           input [2:0] ULAControl,
           output reg [7:0] ULAResult,
           output reg FlagZ);
  always @(*)
    begin
      case (ULAControl)
        3'b000 : ULAResult = SrcA + SrcB;
        3'b001 : ULAResult = SrcA + ~SrcB + 1;
        3'b010 : ULAResult = SrcA & SrcB;
        3'b011 : ULAResult = SrcA | SrcB;
        3'b101 : ULAResult = SrcA < SrcB;
        default : ULAResult = 0;
      endcase
		FlagZ = ULAResult == 0;
    end
  
endmodule


module hex_to_7seg (
  input [3:0] hex,
  output reg [6:0] seg
);

  // Tabela de verdade para decodificar hexadecimal para 7 seg
  always @(*) begin
    case (hex)
		
		4'h0 : seg = 7'b0000001;
		4'h1 : seg = 7'b1001111;
		4'h2 : seg = 7'b0010010;
		4'h3 : seg = 7'b0000110;
		4'h4 : seg = 7'b1001100;
		4'h5 : seg = 7'b0100100;
		4'h6 : seg = 7'b0100000;
		4'h7 : seg = 7'b0001111;
		4'h8 : seg = 7'b0000000;
		4'h9 : seg = 7'b0000100;
		4'hA : seg = 7'b0001000;
		4'hB : seg = 7'b1100000;
		4'hC : seg = 7'b0110001;
		4'hD : seg = 7'b1000010;
		4'hE : seg = 7'b0110000;
		default : seg = 7'b0111000;
    endcase
  end

endmodule

module Unidade_controle(input [6:0] Op, Funct7,
								input [2:0] Funct3,
								output reg [2:0] ULAControl,
								output reg ULASrc, RegWrite, MemWrite, Branch, Jal, JalR,
								output reg [1:0] ImmSrc, ResultSrc);
	always @(*)
		casex ({Op, Funct3, Funct7})
			'b01100110000000000 : begin RegWrite = 1; ImmSrc = 'bx; ULASrc = 0; ULAControl = 'b000; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//add
			'b01100110000100000 : begin RegWrite = 1; ImmSrc = 'bx; ULASrc = 0; ULAControl = 'b001; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//sub
			'b01100111110000000 : begin RegWrite = 1; ImmSrc = 'bx; ULASrc = 0; ULAControl = 'b010; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//and
			'b01100111100000000 : begin RegWrite = 1; ImmSrc = 'bx; ULASrc = 0; ULAControl = 'b011; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//or
			'b01100110100000000 : begin RegWrite = 1; ImmSrc = 'bx; ULASrc = 0; ULAControl = 'b101; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//slt
			'b0010011000xxxxxxx : begin RegWrite = 1; ImmSrc = 'b0; ULASrc = 1; ULAControl = 'b000; MemWrite = 0; ResultSrc = 0; Branch = 0; Jal = 'b0; JalR = 'b0; end//addi
			'b0000011000xxxxxxx : begin RegWrite = 1; ImmSrc = 'b0; ULASrc = 1; ULAControl = 'b000; MemWrite = 0; ResultSrc = 1; Branch = 0; Jal = 'b0; JalR = 'b0; end //LB
			'b0100011000xxxxxxx : begin RegWrite = 0; ImmSrc = 'b1; ULASrc = 1; ULAControl = 'b000; MemWrite = 1; ResultSrc = 'bx; Branch = 0; Jal = 'b0; JalR = 'b0; end //SB
			'b1100011000xxxxxxx : begin RegWrite = 0; ImmSrc = 'b10; ULASrc = 0; ULAControl = 'b001; MemWrite = 0; ResultSrc = 'bx; Branch = 'b1; Jal = 'b0; JalR = 'b0; end // BEQ
			'b1101111xxxxxxxxxx : begin RegWrite = 1; ImmSrc = 'b11; ULASrc = 'bx; ULAControl = 'bxxx; MemWrite = 0; ResultSrc = 'b10; Branch = 'b0; Jal = 'b1; JalR = 'b0; end //Jal
			'b1100111000xxxxxxx : begin RegWrite = 1; ImmSrc = 'b00; ULASrc = 'b1; ULAControl = 'b000; MemWrite = 0; ResultSrc = 'b10; Branch = 'b0; Jal = 'b0; JalR = 'b1; end //Jalr
			default : begin RegWrite = 'bx; ULASrc = 'bx; ULAControl = 'bx; ImmSrc = 'bx; MemWrite = 'bx; ResultSrc = 'bx; Branch = 'bx; Jal = 'bx; JalR = 'bx; end
		endcase	
			
endmodule

module ProgamCounter(input clk, rst, 
							input [7:0] PCin,
							output reg [7:0] PC);
	always @(posedge clk, negedge rst)
		if (!rst)
			PC <= 0;
		else
			PC <= PCin;
endmodule

module Memoria_instrucao(input [7:0] entrada,
								output reg [31:0] RD1);
	always @(*) 
		case (entrada)
		/*
			8'h00 : RD1 = 'h0ff00083;
			8'h04 : RD1 = 'h0e100fa3;
			8'h08 : RD1 = 'hfe000ce3;
		*/
			8'h00 : RD1 = 'h0ff00083;
			8'h04 : RD1 = 'h00008393;
			8'h08 : RD1 = 'h00120213;
			8'h0C : RD1 = 'h0040f0b3;
			8'h10 : RD1 = 'h00108093;
			8'h14 : RD1 = 'h0040f0b3;
			8'h18 : RD1 = 'h0e100fa3
;
		endcase
								
endmodule


module somador4bits (input wire [7:0] A,
							input wire [7:0] B,
							output wire [7:0] soma);
	assign soma = A+B;
endmodule

module RAM(input [7:0] A, WD,
           input WE, rst, clk,
           output reg [7:0] RD);
  
  reg [7:0] mem [0:255];
  integer i;
  
  always @(posedge clk, negedge rst)
    begin
    if(!rst)
      for (i = 0; i < 256; i = i+1)
        mem[i] = 0;
  	else if (WE == 1)
      mem[A] <= WD;
    end
  always @(*)
	begin
    RD <= mem[A];
	 end
	 
endmodule

module clock_divider (
  input clk,
  output reg clk_1hz
);

  reg [25:0] counter;

  always @(posedge clk)
    begin
      counter = counter + 1;
      if (counter == 2500000) 
			begin
			  clk_1hz <= ~clk_1hz;
			  counter <= 0;
		   end
    end
endmodule

module counter (
  input clk,
  input reset,
  output reg [3:0] count
);

  always @(posedge clk or negedge reset) begin
    if (!reset) begin
      count <= 0;
    end 
	 else 
	 begin
      if (count == 15) begin
        count <= 0;
      end 
		else 
		begin
        count <= count + 1;
      end
    end
  end

endmodule

module parallel_out(input EN,clk,reset,
						  input [7:0] RegData,Address,
						  output reg [7:0] DataOut);
						  
reg A = 1'b0;	


always @(*)

begin
if (Address == 8'hFF)
		begin
			A = 1'b1;
		end
		
		else
		begin
			A = 1'b0;
		end		
end	
	
always @ (posedge clk,negedge reset)	
begin

	if (!reset) begin
			DataOut <= 0;
		 end
		 
	else 	
		if ((EN && A) == 1'b1)
		begin
			DataOut <= RegData;
		end
		else
			DataOut <= DataOut;
			
end

endmodule
		
module parrallel_in(input [7:0] Address,MemData,DataIn,
							output reg [7:0] RegData);
reg A = 1'b0;							
always @ (*)

	begin
	if (Address == 8'hFF)
		begin
			A <= 1;
		end
		
		else
		begin
			A <= 0;
		end

	case(A)
		1'b0 : RegData <= MemData;
		1'b1 : RegData <= DataIn;
		default : RegData <= 1'b0;
	endcase
	end

endmodule
					
