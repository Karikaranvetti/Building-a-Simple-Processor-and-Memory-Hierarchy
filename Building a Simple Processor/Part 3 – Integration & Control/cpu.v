
/*************************
   E16172
   V.karikaran
   Lab5_Part3
   CPU Module 
**************************/


/*            ALU  UNIT
/******************************************************************************************8/

/* ALU Arithmetic and Logic Operations
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


/*                    REG FILE UNIT
/****************************************************************************************************************/
module reg_file (IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
  input CLK;
   input RESET;
   input WRITE;
   input [2:0] INADDRESS;
   input [7:0] IN;
   input [2:0] OUT1ADDRESS;
   output [7:0] OUT1;
   input [2:0] OUT2ADDRESS;
   output [7:0] OUT2;

   reg [7:0] 	 regfile [0:7];               //      defining 8 bit register as 8 element array 

   assign #2  OUT1 = regfile[OUT1ADDRESS];          // here regfile always update when posedge clk or reset then 
   assign #2  OUT2 = regfile[OUT2ADDRESS];

   always @(posedge CLK or RESET) begin          //  asynchronous reset condition defind here 
    if (RESET == 1'b1) begin            // level triggered condition
      #2                                   
	    regfile[0] <= 0;             // reset condition all regester geting zero value 
	    regfile[1] <= 0;
	    regfile[2] <= 0;
	    regfile[3] <= 0;
	    regfile[4] <= 0;
	    regfile[5] <= 0;
	    regfile[6] <= 0;
	    regfile[7] <= 0;
      end 
   end
   always @(posedge CLK) begin           // writing synchronously for this always block
      
	 if (WRITE) begin
      #2 regfile[INADDRESS] <= IN;
      end 
   end
endmodule


/*          MUX UNIT
/*********************************************************************************************/


 

module MUX(out,a,b,control);      //two input mux 
	input [7:0] a;
	input [7:0] b;
	output [7:0] out;
	input control;
	reg [7:0] out;

	always@(a,b,control)begin       //  if inputs change this block run 
		case(control)
			1'b0 : out=a[7:0];
			1'b1 : out=b[7:0];
		endcase
	end

endmodule


/*            2'S COMPLIMENT UNIT
/*********************************************************************************************/

module compliment(out,in);          // 2's compliment converter 
output [7:0] out;
input [7:0] in;
reg [7:0] comp=8'b11111111;
assign out=(comp-in)+8'b00000001; // math for converting 
endmodule




/*                     PROGRAM COUNTER UNIT
/*********************************************************************************************/
module counter (INSTaddr,clk,reset);          //program counter unit 
input clk;
input reset;
output reg [31:0] INSTaddr;
always @(posedge clk or posedge reset) begin           // this working as asynchronously 
case(reset)
1: #1 INSTaddr=31'd0;
0: #1  INSTaddr= INSTaddr+31'd4;
endcase
end
endmodule



/*              CPU UNIT
/*********************************************************************************************/


module cpu(PC, INSTRUCTION, CLK, RESET);
output  [31:0]  PC;
input CLK,RESET;
input  [31:0] INSTRUCTION;
 
wire [2:0] READREG1,READREG2,WRITEREG,OP_CODE;          // instruction decoding data addreses for sub module so those are wires 

reg WRITEENABLE,MUX1_SEL,MUX2_SEL;                // reg file write enable signal and two muxes control signals 

reg [2:0] ALUOP,OP1,OP2,DESTINATION;         // instruction decoding data addreses for combinational logic  so those are reg

reg [7:0] IM_VAL;       

assign  READREG2=INSTRUCTION[2:0];         // assing wire ports to  whenever change regaddres will get updates 

assign  READREG1=INSTRUCTION[10:8];         



assign  OP_CODE=INSTRUCTION[26:24];       
assign WRITEREG=DESTINATION;

 always @(*)
    begin
        if (OP_CODE==4'b0000) begin                              // assing wire ports to  whenever change regaddres will get updates 
            DESTINATION=INSTRUCTION[23:16];
        end else begin
            DESTINATION=INSTRUCTION[18:16];
        end
        #1                      // Instruction Decode delay is added here because while decodeing only op_code get updated
        case(OP_CODE)
        4'b0000:   begin    // loadi  function combinational logic     
                    WRITEENABLE=1;
                    MUX2_SEL=1'b0;
                    ALUOP= 3'b000;
                    IM_VAL=INSTRUCTION[7:0];
                end
         4'b0001:  begin     // move function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                     
                end        
        4'b0010:  begin      //add function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                     
                end
        4'b0011:   begin     // sub function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b1;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                     
                end
        4'b0100:  begin       // And function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b010;
                    
                end
         4'b0101:  begin       //OR function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b011;
                     
                end
        endcase
    end
     
     
     //...........submodule connections  ............///


 //reg_file (IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
  wire [7:0] IN;
  wire [7:0] OUT1;
  wire [7:0] OUT2;
  reg_file cpu_reg_file(IN,OUT1,OUT2,WRITEREG,READREG1,READREG2,WRITEENABLE,CLK,RESET);

 //module compliment(out,in);
 wire [7:0] OUT3;
 compliment cpu_compliment(OUT3,OUT2);

 //module MUX(out,a,b,control);
 wire [7:0] OUT4;
 MUX cpu_MUX1(OUT4,OUT2,OUT3,MUX1_SEL);

 //module MUX(out,a,b,control);
 wire [7:0] OUT5;
 MUX cpu_MUX2(OUT5,IM_VAL,OUT4,MUX2_SEL);

 //module alu(DATA1, DATA2, RESULT, SELECT); 
 alu cpu_alu(OUT1,OUT5,IN,ALUOP);

 //module counter (INSTaddr,clk,reset);
  counter cpu_counter(PC,CLK,RESET);

endmodule



/*********************************************************************************************/
   module test;
   
   //inputs
    reg clk,reset;
    wire [31:0] inst;
    reg [7:0] memory[1023:0];

    //outputs
    wire [31:0] PC;
     
  cpu mycpu(PC,inst,clk,reset);
  assign #2 inst={ memory[PC],memory[PC+1],memory[PC+2],memory[PC+3]};   // counter assing 
    

        
       
      initial
    begin
        { memory[0],memory[1],memory[2],memory[3]}=32'b00000000000000010000000011110111;
        { memory[4],memory[5],memory[6],memory[7]}=32'b00000010000000110000001000000001;
        { memory[8],memory[9],memory[10],memory[11]}=32'b00000011000000010000000100000001;
        { memory[12],memory[13],memory[14],memory[15]}=32'b00000001000000100000000000000001;
        { memory[16],memory[17],memory[18],memory[19]}=32'b00000010000001000000000100000010;
        { memory[20],memory[21],memory[22],memory[23]}=32'b00001000000000010000000000000010;
        { memory[24],memory[25],memory[26],memory[27]}=32'b00000001000000000000000000000001;
        { memory[28],memory[29],memory[30],memory[31]}=32'b00000000000000010000000000000010;
        { memory[32],memory[33],memory[34],memory[35]}=32'b00000001000000100000000000000001;
        { memory[36],memory[37],memory[38],memory[39]}=32'b00000000000000010000000000000011;
        { memory[40],memory[41],memory[42],memory[43]}=32'b00000001000000110000000000000001;
        { memory[44],memory[45],memory[46],memory[47]}=32'b00000000000000010000000000000100;
        { memory[48],memory[49],memory[50],memory[51]}=32'b00000001000001000000000000000001;
        { memory[52],memory[53],memory[54],memory[55]}=32'b00000000000000010000000000000101;             
        { memory[56],memory[57],memory[58],memory[59]}=32'b00000001000001010000000000000001;
        { memory[60],memory[61],memory[62],memory[63]}=32'b00000000000000010000000000000110;
        { memory[64],memory[65],memory[66],memory[67]}=32'b00000001000001100000000000000001;
        { memory[68],memory[69],memory[70],memory[71]}=32'b00000000000000010000000000000111;
        { memory[72],memory[73],memory[74],memory[75]}=32'b00000001000001110000000000000001;             
        { memory[76],memory[77],memory[78],memory[79]}=32'b00000000000000010000000000001000;
        { memory[80],memory[81],memory[82],memory[83]}=32'b00001000000000010000000000000001;
        { memory[84],memory[85],memory[86],memory[87]}=32'b00000100000001110000001100000100;
        { memory[88],memory[89],memory[90],memory[91]}=32'b00000101000000010000010100000110;
        
         
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		    $dumpvars(0, test);
        
        clk  = 1'b0;
        reset = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
         #7
         reset=1'b1;
         #5
        reset = 1'b0;
        // finish simulation after some time
        #250
        $finish;
        
    end
    
    // clock signal generation
    always
        #5 clk = ~clk;
    endmodule



//program
/*  Tthis the program we are run in our processer  
// loadi 1 0xF7
//  add 3 2 1  
//  sub 1 1 1
//  mov 2 1
 

//  add 4 1 2
 
// load 1 0x02
// mov 0 1
// loadi 1 0x02
// mov 2 1
 
// loadi 1 0x03
// mov 3 1
// loadi 1 0x04
// mov 4 1
// loadi 1 0x05
// mov 5 1
// loadi 1 0x06
// mov 6 1
// loadi 1 0x07
// mov 7 1
// loadi 1 0x08
 
// load 1 0x01