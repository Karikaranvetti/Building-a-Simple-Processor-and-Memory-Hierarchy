
/*************************
   E16172
   V.karikaran
   Lab6_Part1      
   CPU Module 
**************************/


/*            ALU  UNIT
/******************************************************************************************8/

/* ALU Arithmetic and Logic Operations
----------------------------------------------------------------------*/
 
module alu(DATA1, DATA2, RESULT, SELECT,ZERO,ONE);                    // module assign  
input [7:0] DATA1,DATA2;           //ALU 8-bit Inputs 
input [2:0] SELECT;                // ALU Selection
output reg [7:0] RESULT;         // ALU 8-bit Output

output  ZERO;           // ALU 8-bit Output
output  ONE;           // ALU 8-bit Output
assign ZERO=(~(RESULT[0] | RESULT[1] | RESULT[2] | RESULT[3] | RESULT[4] | RESULT[5] | RESULT[6] | RESULT[7]));       // this doing the XOR operation if All bits are zero then its give 1 as out put otherwise 0 is output

assign ONE=((RESULT[0] | RESULT[1] | RESULT[2] | RESULT[3] | RESULT[4] | RESULT[5] | RESULT[6] | RESULT[7]));   // this or operation for not equal 

always @ (DATA1,DATA2,SELECT,RESULT) begin        // if data1,data2,select and resuseclt anything change this always block executed once
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

module MUX8(out,a,b,control);      //two input mux 
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

/*        32 BIT  MUX UNIT  working with timing 
/*********************************************************************************************/

module MUX32(out,a,b,control,reset,clk);      //two input mux 
	input [31:0] a;
    input reset,clk;
	input [31:0] b;
	output  [31:0] out;
	input control;
	reg [31:0] out;

	always@(a or posedge clk or posedge reset)begin       //  Here if input1 ,fall of clk,rice of reset should be out update 
		 
        if(reset==1)begin
            out=32'd0;
        end else if (control==1'b0) begin
            out=a[31:0];
        end else if(control==1'b1) begin
            out=b[31:0];
        end
	end

endmodule

/*         PC and 32 bit  MUX UNIT working with timing 
/*********************************************************************************************/

module PC_MUX32(out,input1,input2,control,reset,clk);      //  
	input [31:0] input1;
    input reset,clk;
	input [31:0] input2;
	output  [31:0] out;
	input control;
	reg [31:0] out;

	always@(input1 or posedge clk or posedge reset)begin       //  Here if input1 ,fall of clk,rice of reset should be out update 
		 
           #1                     // this delay for PC updateing 
        if(reset==1)begin        // this start the program or giving the value for pc
            out=32'd0;
        end else if (control==1'b0) begin     // here to end mux function
            out=input1[31:0];
        end else if(control==1'b1) begin
            out=input2[31:0];            // if it is jump 
        end
	end

endmodule



/*            2'S COMPLIMENT UNIT
/*********************************************************************************************/

module compliment(out,in);          // 2's compliment converter 
    output [7:0] out;
    input [7:0] in;
    reg [7:0] comp=8'b11111111;
    assign #1 out=(comp-in)+8'b00000001; // math for converting 
endmodule




/*             arithmetic shift right
/*********************************************************************************************/

module Arith_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
    input EANABLE;
    input [7:0] SHIFT;
    input [7:0] IN_DATA;
    output reg [7:0] OUT_DATA;

     
    always @(EANABLE or SHIFT or IN_DATA or OUT_DATA) begin
     
      if (EANABLE==1) begin          // if right shift condition only shifting happeing 
         #1                              // for shifting delay is 1 time unit 
           case (SHIFT[3:0])                               // we can shift maximum 8 other then then always smae value 
          4'd0:OUT_DATA = IN_DATA;
          4'd1:OUT_DATA = { IN_DATA[7],IN_DATA[7:1] };
          4'd2:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7:2] };
          4'd3:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:3] };
          4'd4:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:4] };
          4'd5:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:5] };
          4'd6:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:6] };
          4'd7:OUT_DATA = { IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:7] };
          
            
            default: OUT_DATA={ IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7],IN_DATA[7:7] };
          endcase
          
          
      end else begin
          OUT_DATA=8'd0;            // if eanableis 0 then out put is zero 
          end
    end

endmodule


/*             logical shift right
/*********************************************************************************************/



module Logic_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
    input EANABLE;
    input [7:0] SHIFT;
    input [7:0] IN_DATA;
    output reg [7:0] OUT_DATA;

     
    
    always @( EANABLE or SHIFT or IN_DATA or OUT_DATA) begin
      if (EANABLE==1) begin                             // if right shift condition only shifting happeing 
       #1                                             // for shifting delay is 1 time unit 
           case (SHIFT[3:0])                          // we can shift maximum 8 other then then always smae value 
          4'd0:OUT_DATA = IN_DATA;                    
          4'd1:OUT_DATA = { 1'b0,IN_DATA[7:1] };
          4'd2:OUT_DATA = { 2'b0,IN_DATA[7:2] };
          4'd3:OUT_DATA = { 3'b0,IN_DATA[7:3] };
          4'd4:OUT_DATA = { 4'b0,IN_DATA[7:4] };
          4'd5:OUT_DATA = { 5'b0,IN_DATA[7:5] };
          4'd6:OUT_DATA = { 6'b0,IN_DATA[7:6] };
          4'd7:OUT_DATA = { 7'b0,IN_DATA[7:7] };
          
            
            default: OUT_DATA=8'b0;
          endcase
          
          
      end else begin
          OUT_DATA=8'd0;                                // if eanableis 0 then out put is zero 
          end
    end

endmodule

/*             logical shift left
/*********************************************************************************************/

module Logic_left(EANABLE,SHIFT,IN_DATA,OUT_DATA);
    input EANABLE;
    input [7:0] SHIFT;
    input [7:0] IN_DATA;
    output reg [7:0] OUT_DATA;

    integer i;
    integer j;
    
    always @(EANABLE or SHIFT or IN_DATA or OUT_DATA) begin
      if (EANABLE==1) begin           // if right shift condition only shifting happeing 
       #1                                                 // for shifting delay is 1 time unit 
          case (SHIFT[3:0])                                               // we can shift maximum 8 other then then always smae value 
          4'd0:OUT_DATA = IN_DATA;
          4'd1:OUT_DATA = { IN_DATA[6:0],1'b0 };
          4'd2:OUT_DATA = { IN_DATA[5:0],2'b0 };
          4'd3:OUT_DATA = { IN_DATA[4:0],3'b0 };
          4'd4:OUT_DATA = { IN_DATA[3:0],4'b0 };
          4'd5:OUT_DATA = { IN_DATA[2:0],5'b0 };
          4'd6:OUT_DATA = { IN_DATA[1:0],6'b0 };
          4'd7:OUT_DATA = { IN_DATA[0:0],7'b0 };
          
            
            default: OUT_DATA=8'b0;
          endcase
          
          
      end else begin
          OUT_DATA=8'd0;                   // if eanableis 0 then out put is zero 
          end
    end

endmodule


/*             rotate right unit
/*********************************************************************************************/
module  Rotate_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
    input EANABLE;
    input [7:0] SHIFT;
    input [7:0] IN_DATA;
    output reg [7:0] OUT_DATA;
    wire [7:0] point;
    
assign  point[2:0]=SHIFT[2:0];

    always @(EANABLE or SHIFT or IN_DATA or OUT_DATA) begin
     
      if (EANABLE==1) begin                    
       #1                                       // for shifting delay is 1 time unit                     
          case (point[2:0]) 
          3'd0:OUT_DATA = IN_DATA;
          3'd1:OUT_DATA = { IN_DATA[0],IN_DATA[7:1]};
          3'd2:OUT_DATA = { IN_DATA[1],IN_DATA[0],IN_DATA[7:2]};
          3'd3:OUT_DATA = { IN_DATA[2],IN_DATA[1],IN_DATA[0],IN_DATA[7:3]};
          3'd4:OUT_DATA = { IN_DATA[3],IN_DATA[2],IN_DATA[1],IN_DATA[0],IN_DATA[7:4]};
          3'd5:OUT_DATA = { IN_DATA[4],IN_DATA[3],IN_DATA[2],IN_DATA[1],IN_DATA[0],IN_DATA[7:5]};
          3'd6:OUT_DATA = { IN_DATA[5],IN_DATA[4],IN_DATA[3],IN_DATA[2],IN_DATA[1],IN_DATA[0],IN_DATA[7:6]};
          3'd7:OUT_DATA = { IN_DATA[6],IN_DATA[5],IN_DATA[4],IN_DATA[3],IN_DATA[2],IN_DATA[1],IN_DATA[0],IN_DATA[7:7]};
          
            
            default: OUT_DATA=8'b0;
          endcase
          
          
      end else begin
          OUT_DATA=8'd0;                               // if eanableis 0 then out put is zero 
          end
    end

endmodule

// /*                    MULTIPLICATION UNIT
// /*********************************************************************************************/
module  multipilication(EANABLE,IN1,IN2,OUT_DATA);
    input EANABLE;
    input [7:0] IN1,IN2;
    output reg [7:0] OUT_DATA;
    reg [7:0] point[7:0];
     
always @(EANABLE or IN1 or IN2 or OUT_DATA) begin
      if (EANABLE==1) begin                          // if mult  condition only multiplication  happeing 
       #2                                           // for multiplication  delay is 1 time unit 
          point[0]={(IN1[7] & IN2[0]),(IN1[6] & IN2[0]),(IN1[5] & IN2[0]),(IN1[4] & IN2[0]),(IN1[3] & IN2[0]),(IN1[2] & IN2[0]),(IN1[1] & IN2[0]),(IN1[0] & IN2[0])};
          point[1]={(IN1[6] & IN2[1]),(IN1[5] & IN2[1]),(IN1[4] & IN2[1]),(IN1[3] & IN2[1]),(IN1[2] & IN2[1]),(IN1[1] & IN2[1]),(IN1[0] & IN2[1]),1'd0};
          point[2]={(IN1[5] & IN2[2]),(IN1[4] & IN2[2]),(IN1[3] & IN2[2]),(IN1[2] & IN2[2]),(IN1[1] & IN2[2]),(IN1[0] & IN2[2]),2'd0};  
          point[3]={(IN1[4] & IN2[3]),(IN1[3] & IN2[3]),(IN1[2] & IN2[3]),(IN1[1] & IN2[3]),(IN1[0] & IN2[3]),3'd0};  
          point[4]={(IN1[3] & IN2[4]),(IN1[2] & IN2[4]),(IN1[1] & IN2[4]),(IN1[0] & IN2[4]),4'd0};  
          point[5]={(IN1[2] & IN2[5]),(IN1[1] & IN2[5]),(IN1[0] & IN2[5]),5'd0};
          point[6]={(IN1[1] & IN2[6]),(IN1[0] & IN2[6]),6'd0};
          point[7]={(IN1[0] & IN2[7]),7'd0};

          OUT_DATA=point[0] + point[1] + point[2] + point[3] + point[4] + point[5] + point[6] + point[7];
      end else begin
          OUT_DATA=8'd0;
          end
    end

endmodule





// /*                    DEFAULT PROGRAM COUNTER UNIT
// /*********************************************************************************************/
 module counter (in_pc,out_pc,clk,reset,busywait);          //program counter unit 
 
    input [31:0] in_pc;
    input clk,reset,busywait;
    output reg  [31:0] out_pc;
    always @( busywait or posedge clk or posedge reset) begin   // this block always adding +4 with current pc value  in fall clk time
        case(busywait)
           1'b0:  out_pc=in_pc+32'd4;
           1'b1: out_pc=in_pc;
        endcase
    end

 endmodule

//                         Sing Extend and Shift unit
/*********************************************************************************************/

module sing_extend_shift(IN,OUT);
    input [7:0] IN;           // 8 bit immediate value as input 
    output  [31:0] OUT;     // 32 bit output value for pc  
    reg [31:0] result;
    assign OUT=result;
   always @(IN) begin // If IN change this will run 
        case(IN[7])   // most signifigent bit of input diside whether is this positive or negative 
        1'b1:result<={22'b1111111111111111111111,IN,2'b00};   // if it is a negative value then sing extention 
        1'b0:result<={22'b0000000000000000000000,IN,2'b00};    // if it is a positive value then sing extention  
    endcase
   
   end

endmodule

/*______________________________________________________________________________________________________________________________*/
module data_memory(
	clock,
    reset,
    read,
    write,
    address,
    writedata,
    readdata,
    busywait
);
input           clock;
input           reset;
input           read;
input           write;
input[7:0]      address;
input[7:0]      writedata;
output reg [7:0]readdata;
output reg      busywait;

//Declare memory array 256x8-bits 
reg [7:0] memory_array [255:0];

//Detecting an incoming memory access
reg readaccess, writeaccess;
always @(read, write)
begin
	busywait = (read || write)? 1 : 0;
	readaccess = (read && !write)? 1 : 0;
	writeaccess = (!read && write)? 1 : 0;
end

//Reading & writing
always @(posedge clock)
begin
    if(readaccess)
    begin
        readdata = #40 memory_array[address];
        busywait = 0;
		readaccess = 0;
    end
    if(writeaccess)
	begin
        memory_array[address] = #40 writedata;
        busywait = 0;
		writeaccess = 0;
    end
end

integer i;

//Reset memory
always @(posedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] = 0;
        
        busywait = 0;
		readaccess = 0;
		writeaccess = 0;
    end
end

endmodule


/*              CPU UNIT
/*********************************************************************************************/


module cpu(PC, INSTRUCTION, CLK, RESET, read, write, address,writedata,readdata,busywait);
output  [31:0]  PC;
output read,write;
output [7:0] writedata,address;
input CLK,RESET,busywait;
input [7:0] readdata;
input  [31:0] INSTRUCTION;
 
wire [2:0] READREG1,READREG2,WRITEREG;          // instruction decoding data addreses for sub module so those are wires 
wire [4:0] OP_CODE;
// control singnals
reg WRITEENABLE;     // for read reg writing 
reg MUX1_SEL;      // for 2s compliment selecton 
reg MUX2_SEL;      // for alu immediate value selection 
reg JUMP;           // jump mux selection 
reg BEQ;          // beq mux selection 
reg BNE;         // bne mux selection 
reg ALUM_selc;     //the selecting for date fatch from memory or just alu output 
reg A_right,AR_EANABLE;       // arithmatic right shift control
reg L_right,LR_EANABLE;       // logic right shift control
reg L_left,LL_EANABLE;      // logic left shift 
reg R_right,RR_EANABLE;       //    rotate unit control
reg Mult_EANABLE,Mult_select;                // reg file write enable signal and two muxes control signals 
reg [7:0] IM_VAL;    // this wire for geting immdeate 
reg [7:0] S_IM_VAL;
reg Mread;  // control signal for memory read 
reg Mwrite;  // control signal for memory write 
assign read=Mread;    ///assing for output read 
assign write=Mwrite;  


reg [2:0] ALUOP,OP1,OP2,DESTINATION;         // instruction decoding data addreses for combinational logic  so those are reg


wire MUX4_SEL;          // this for bne and defulte pc selection wire
wire MUX3_SEL;       // this for beq and defulte pc selection wire
wire [31:0] Branch;    // this wire holding value of jump or value of beq  for pc updating 
assign #2 Branch=NEXT+SinEx;        // this the adder operation so that for 2 time unit delay 
assign MUX3_SEL=BEQ & zero;          // selecting signal genetion for beq 

assign MUX4_SEL=BNE & one; 

assign PC=J_out;            // here assinnig output of jump mux value as pc  

assign  READREG2=INSTRUCTION[2:0];         // assing wire ports to  whenever change regaddres will get updates 

assign  READREG1=INSTRUCTION[10:8];         
  
 
assign  OP_CODE=INSTRUCTION[28:24];       
assign WRITEREG=DESTINATION;

 always @(*)
    begin
        if (OP_CODE==5'b00000) begin                              // assing wire ports to  whenever change regaddres will get updates 
            DESTINATION=INSTRUCTION[18:16];
        end else begin
            DESTINATION=INSTRUCTION[18:16];
        end
         Mread=1'd0;   
         Mwrite=1'd0;


        #1                      // Instruction Decode delay is added here because while decodeing only op_code get updated
        case(OP_CODE)
        5'b00000:   begin    // loadi  function combinational logic     
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b0;
                    ALUOP= 3'b000;
                    IM_VAL=INSTRUCTION[7:0];
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                    ALUM_selc=1'd0;
                    Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                end
         5'b00001:  begin     // move function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                     
                end        
        5'b00010:  begin      //add function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                    IM_VAL=8'd0;
                    BEQ=1'd0;
                    JUMP=1'd0;
                     BNE=1'd0;
                     A_right=1'b0;
                     AR_EANABLE=1'b0;
                      L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                end
        5'b00011:   begin     // sub function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b1;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0; 
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                      L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                end
        5'b00100:  begin       // And function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b010;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                     R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                    
                end
         5'b00101:  begin       //OR function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b011;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                     A_right=1'b0;
                     AR_EANABLE=1'b0;
                      L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                      
                     
                     
                end
        5'b00110:  begin       //J function combinational logic 
                    WRITEENABLE=0;
                    BEQ=1'd0;
                    JUMP=1'd1;
                    BNE=1'd0;
                    IM_VAL=INSTRUCTION[23:16];
                     ALUM_selc=1'd0;
                    Mwrite=1'd0;
                     Mread=1'd0; 
                     
                      
                end           
        5'b00111:  begin       //beq function combinational logic 
                    WRITEENABLE=0;
                    MUX1_SEL=1'b1;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                    BEQ=1'd1;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=INSTRUCTION[23:16];
                    A_right=1'b0;
                     AR_EANABLE=1'b0;
                      L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                end 
         5'b01000:  begin      //mult function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    IM_VAL=8'd0;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd1;
                    Mult_select=1'd1;
                     ALUM_selc=1'd0;
                    Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                     
                end    
        5'b01001:  begin       //bne function combinational logic 
                    WRITEENABLE=0;
                    MUX1_SEL=1'b1;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b001;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd1;
                    IM_VAL=INSTRUCTION[23:16];
                    A_right=1'b0;
                     AR_EANABLE=1'b0;
                      L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                     S_IM_VAL=8'd0;
                     Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                     
                end
         5'b01010:  begin     // sra function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b1;
                    AR_EANABLE=1'b1;
                    L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=INSTRUCTION[15:8];
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                end    
         5'b01011:  begin     // srl function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                    L_right=1'b1;
                    LR_EANABLE=1'b1;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=INSTRUCTION[15:8];
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                end  
         5'b01100:  begin     // sll function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                    L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b1;
                    LL_EANABLE=1'b1;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                     S_IM_VAL=INSTRUCTION[15:8];
                     Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                    Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                end 
        5'b01101:  begin     // ror function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                    L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b1;
                    RR_EANABLE=1'b1;
                     S_IM_VAL=INSTRUCTION[15:8];
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd0;
                     Mwrite=1'd0;
                     Mread=1'd0; 
                     
                     
                end
         5'b01110:  begin     // swd  function combinational logic 
                    WRITEENABLE=0;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd1;
                     Mwrite=1'd1;
                     Mread=1'd0; 
                     
                     
                     
                end    
             5'b01111:  begin     // swi  function combinational logic 
                    WRITEENABLE=0;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b0;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=INSTRUCTION[7:0];
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd1;
                     Mwrite=1'd1;
                     Mread=1'd0; 
            end
             5'b10000:  begin     // lwd  function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b1;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=8'd0;  
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd1;
                     Mwrite=1'd0;
                     Mread=1'd1; 
            end      
            5'b10001:  begin     // lwi  function combinational logic 
                    WRITEENABLE=1;
                    MUX1_SEL=1'b0;
                    MUX2_SEL=1'b0;
                    ALUOP= 3'b000;
                    BEQ=1'd0;
                    JUMP=1'd0;
                    BNE=1'd0;
                    IM_VAL=INSTRUCTION[7:0]; 
                    A_right=1'b0;
                    AR_EANABLE=1'b0;
                     L_right=1'b0;
                    LR_EANABLE=1'b0;
                     L_left=1'b0;
                    LL_EANABLE=1'b0;
                    R_right=1'b0;
                    RR_EANABLE=1'b0;
                    S_IM_VAL=8'd0;
                    Mult_EANABLE=1'd0;
                    Mult_select=1'd0;
                     ALUM_selc=1'd1;
                     Mwrite=1'd0;
                     Mread=1'd1; 
            end 
        endcase
    end
     
     
     //...........submodule connections  ............///



//________________Register_file module ___________



//reg_file (IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
  wire [7:0] IN;
  wire [7:0] OUT1;
  wire [7:0] OUT2;
  wire [7:0] ALU_OUT;
  reg_file cpu_reg_file(ALU_OUT,OUT1,OUT2,WRITEREG,READREG1,READREG2,WRITEENABLE,CLK,RESET);
assign writedata=OUT1;   // assing for writedata out put
assign  address=IN;        // assing for addres output
 
 //________________ALU module ___________
 //module alu(DATA1, DATA2, RESULT, SELECT,ZERO,ONE); 
 wire zero;
 wire one;
 wire [7:0] OUT5;
 alu cpu_alu(OUT1,OUT5,IN,ALUOP,zero,one);


//________________ALU_MUX ___________
//module MUX8(out,a,b,control); 

MUX8 cpu_ALUMUX(ALU_OUT,IN,readdata,ALUM_selc);



 


//________________2s compliment module and mux connction__________________

 //module compliment(out,in);
 wire [7:0] OUT3;
 compliment cpu_compliment(OUT3,OUT2);       

 //module MUX8(out,a,b,control);
 wire [7:0] OUT4;
 MUX8 cpu_MUX1(OUT4,OUT2,OUT3,MUX1_SEL);

 //____________________Arithmetic right shift module  and mux connection _____________________

 //module Arith_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
 wire [7:0] AR_OUT;
Arith_right cpu_Arith_right(AR_EANABLE,S_IM_VAL,OUT4,AR_OUT);


//module MUX8(out,a,b,control);
 wire [7:0] OUT_s1;
 MUX8 cpu_MUX_s1(OUT_s1,OUT4,AR_OUT,A_right);      


//____________________Logic right module and mux _______________________

//module Logic_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
 wire [7:0] LR_OUT;
 Logic_right cpu_Logic_right(LR_EANABLE,S_IM_VAL,OUT_s1,LR_OUT);

 //module MUX8(out,a,b,control);
 wire [7:0] OUT_s2;
 MUX8 cpu_MUX_s2(OUT_s2,OUT_s1,LR_OUT,L_right); 


//__________________Logic left module and mux _____________________

//module Logic_left(EANABLE,SHIFT,IN_DATA,OUT_DATA);
wire [7:0] LL_OUT;
Logic_left cpu_Logic_left(LL_EANABLE,S_IM_VAL,OUT_s2,LL_OUT);

//module MUX8(out,a,b,control);
 wire [7:0] OUT_s3;
 MUX8 cpu_MUX_s3(OUT_s3,OUT_s2,LL_OUT,L_left); 

//_____________________Rotate right module and mux ______________________

//module  Rotate_right(EANABLE,SHIFT,IN_DATA,OUT_DATA);
wire [7:0] RR_OUT;
Rotate_right cpu_Rotate_right(RR_EANABLE,S_IM_VAL,OUT_s3,RR_OUT);


//module MUX8(out,a,b,control);
 wire [7:0] OUT_s4;
 MUX8 cpu_MUX_s4(OUT_s4,OUT_s3,RR_OUT,R_right); 


//___________Multiplication module and mux _____________

//module  multipilication(EANABLE,IN1,IN2,OUT_DATA);
wire [7:0]  Mult_out;
multipilication cpu_multipilication(Mult_EANABLE,OUT_s4,OUT1,Mult_out);


//module MUX8(out,a,b,control);
 wire [7:0] OUT_M1;
 MUX8 cpu_MUX_M1(OUT_M1,OUT_s4,Mult_out,Mult_select); 


//__________________Immdiate value getting mux and send to the ALU__________

 //module MUX8(out,a,b,control);
 
 MUX8 cpu_MUX2(OUT5,IM_VAL,OUT_M1,MUX2_SEL);




//________________PC COUNTING UNIT_____________

 // module counter (in_pc,out_pc,clk,reset);     
 wire [31:0]NEXT;
 wire [31:0] J_out;
  counter cpu_counter(J_out,NEXT,CLK,RESET,busywait);


//_____________SHIFT and Sing Extend module for Pc immdiate value 

//module sing_extend_shift(IN,OUT);
wire [31:0]SinEx;
sing_extend_shift cpu_sing_extend_shift(IM_VAL,SinEx);


//__________32 bit mux unit for BEQ___________

//module MUX32(out,a,b,control,reset,clk);
wire [31:0] Beq_out;                            
MUX32 cpu_MUX32_1(Beq_out,NEXT,Branch,MUX3_SEL,RESET,CLK);

//______________32 bit mux unit for BNE ________

//module MUX32(out,a,b,control,reset,clk);
wire [31:0] Bne_out;                            
MUX32 cpu_MUX32_2(Bne_out,Beq_out,Branch,MUX4_SEL,RESET,CLK);

//______________ 32 bit mux unit for JUMP and RESETING unit _______________
//module PC_MUX32(out,a,b,control,reset,clk);

PC_MUX32  cpu_PC_MUX32 (J_out,Bne_out,Branch,JUMP,RESET,CLK);




endmodule



//_________________________CPU TEST BENCH_______________________________________

/*********************************************************************************************/ 
   module test;
   
   //inputs
    reg clk,reset;
    wire busywait;
    wire [7:0] readdata;
    wire [31:0] inst;
    reg [7:0] memory[1023:0];

    //outputs
    wire [31:0] PC;
    wire read,write;
    wire [7:0] writedata,address;
     
  cpu mycpu(PC,inst,clk,reset, read, write, address,writedata,readdata,busywait);
  //module data_memory(clock,reset,read,write,address,writedata,readdata,busywait);
  data_memory my_data_memory(clk,reset,read,write,address,writedata,readdata,busywait);
  assign #2 inst={ memory[PC],memory[PC+1],memory[PC+2],memory[PC+3]};   // counter assing 
    

        
       
      initial
    begin
       { memory[0],memory[1],memory[2],memory[3]}=32'b00000000000000010000000000000011;
        { memory[4],memory[5],memory[6],memory[7]}=32'b00000000000001000000000000000001;
        { memory[8],memory[9],memory[10],memory[11]}=32'b00000000000000100000000000000101;
        { memory[12],memory[13],memory[14],memory[15]}=32'b00000000000000110000000000000011;
        { memory[16],memory[17],memory[18],memory[19]}=32'b00000010000001010000001000000011;
        { memory[20],memory[21],memory[22],memory[23]}=32'b00000011000000010000000100000100;
        { memory[24],memory[25],memory[26],memory[27]}=32'b00000001000001100000000000000001;
        { memory[28],memory[29],memory[30],memory[31]}=32'b00000111000000100000010000000001;
        { memory[32],memory[33],memory[34],memory[35]}=32'b00000110111111000000000000000000;
        { memory[36],memory[37],memory[38],memory[39]}=32'b00000000000000010000000001101001;
        { memory[40],memory[41],memory[42],memory[43]}=32'b00000000000001000000000000000001;
        { memory[44],memory[45],memory[46],memory[47]}=32'b00000000000000100000000000000101;
        { memory[48],memory[49],memory[50],memory[51]}=32'b00000000000000110000000000000011;
        { memory[52],memory[53],memory[54],memory[55]}=32'b00001110000000000000000100000100;
        { memory[56],memory[57],memory[58],memory[59]}=32'b00000000000000000000000000000011;
        { memory[60],memory[61],memory[62],memory[63]}=32'b00001111000000000000001000000010;
        { memory[64],memory[65],memory[66],memory[67]}=32'b00001000000001000000000100000010;
        { memory[68],memory[69],memory[70],memory[71]}=32'b00001100000001000000000100000010;
        { memory[72],memory[73],memory[74],memory[75]}=32'b00001011000001000000000100000010;             
        { memory[76],memory[77],memory[78],memory[79]}=32'b00001010000001000000000100000010;
        { memory[80],memory[81],memory[82],memory[83]}=32'b00001101000001000000000100000010;
        { memory[84],memory[85],memory[86],memory[87]}=32'b00001001000000100000000100000010;
        { memory[88],memory[89],memory[90],memory[91]}=32'b00000000000000110000000000000001;
        { memory[92],memory[93],memory[94],memory[95]}=32'b00010000000001010000000000000011;
        { memory[96],memory[97],memory[98],memory[99]}=32'b00000000000000000000000000000011;
        { memory[100],memory[101],memory[102],memory[103]}=32'b00010001000001100000000000000010;
        { memory[104],memory[105],memory[106],memory[107]}=32'b00001001111101100000001100000000;


         //00001111000000000000000100000010;//00001110000000000000000100000010;
         //b00000010000001010000001000000011; 
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
        #500 
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 clk = ~clk;
    endmodule



//program
/*  This the  code we are run in our processer  

loadi 1 0x03
loadi 4 0x01
loadi 2 0x05
loadi 3 0x03
add 5 2 3 //comment
sub 1 1 4
mov 6 1
beq 0x02 4 1
j 0xFC
loadi 1 0x69
loadi 4 0x01
loadi 2 0x05
loadi 3 0x03
swd 1 4
loadi 0 0x03
swi 2 0x02

mult 4 1 2           
sll 4 1 0x02         
srl 4 1 0x02         
sra 4 1 0x02         
ror 4 1 0x02         
bne 0x02 1 2         
loadi 3 0x01       
lwd 5 3
loadi 0 0x03 
lwi 6  0x02
bne 0xF6 3 0 

*/

