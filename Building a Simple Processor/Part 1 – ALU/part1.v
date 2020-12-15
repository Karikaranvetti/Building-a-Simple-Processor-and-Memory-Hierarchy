/* ALU Arithmetic and Logic Operations
----------------------------------------------------------------------
|ALU_Sel|   ALU Operation
----------------------------------------------------------------------
| 000  |   ALU_Out = 2ed operand   //forward operation
----------------------------------------------------------------------
| 001  |   ALU_Out = A + B;
----------------------------------------------------------------------
| 010  |  ALU_Out = A & B;
----------------------------------------------------------------------
| 011  |   ALU_Out = A | B;
----------------------------------------------------------------------*/
module alu(DATA1, DATA2, RESULT, SELECT);                    // module assign  
input [7:0] DATA1,DATA2;           //ALU 8-bit Inputs 
input [2:0] SELECT;                // ALU Selection
output reg [7:0] RESULT;           // ALU 8-bit Output
always @ (DATA1,DATA2,SELECT,RESULT) begin        // if data1,data2,select and result anything change this always block executed once
    case (SELECT)
        3'b000:   //forward 
         #1   RESULT=DATA2;
        3'b001: // Addition
          #2  RESULT=DATA1 + DATA2;
        3'b010:  //  Logical and
         #1   RESULT=DATA1 & DATA2;
        3'b011:  //  Logical or
         #1   RESULT= DATA1 | DATA2;
 
    endcase
end


endmodule

  

module alu_part1_test;
//Inputs
 reg[7:0] data1,data2;
 reg[2:0] ALU_Select;

//Outputs
 wire[7:0] ALU_Result;
 

 // Verilog code for ALU
 integer i;
 alu test_unit(data1,data2,             // ALU 8-bit Inputs 
                ALU_Result,       // ALU 8-bit Output
                ALU_Select);      // ALU Selection
    initial begin
    
      $dumpfile("alu_part1_wavedata.vcd");
		  $dumpvars(0, alu_part1_test);

      data1 = 8'd5;        // here we can assign data1 and data2 inputs 
      data2 = 8'd25;
      ALU_Select = 3'd0;             //assigning select 
      
      #10
      data1 = 8'd50;        // here we can assign data1 and data2 inputs 
      data2 = 8'd60;
      ALU_Select = 3'd1;

       #10
      data1 = 8'd240;        // here we can assign data1 and data2 inputs 
      data2 = 8'd15;
      ALU_Select = 3'd2;

       #10
      data1 = 8'd63;        // here we can assign data1 and data2 inputs 
      data2 = 8'd31;
      ALU_Select = 3'd3;

      #10
      $finish;

       
    end
endmodule