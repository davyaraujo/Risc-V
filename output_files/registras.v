module BancodeRegistradores (input [7:0] escrita_data,
                            input write_enable, clock, reset,
                             input [2:0] write_address, register_address1, register_address2,
                            output reg [7:0] register_data1, register_data2);
                            

  reg [7:0] registrador [0:7];
  integer i;
  always @ (posedge clock or negedge reset)
    begin
		if(!reset)
		begin
			for(i=0; i<8; i=i+1)
             registrador[i] = 8'd0;
		end
		else
			
      if(write_enable)
        begin
          if(write_address == 0)
              registrador[0] = 8'd0;
          else
            registrador[write_address] <= escrita_data;
        end  
    end
  
  always @ (*)
    begin
      register_data1 <= registrador[register_address1];
      register_data2 <= registrador[register_address2];
    end
	 
	 
endmodule