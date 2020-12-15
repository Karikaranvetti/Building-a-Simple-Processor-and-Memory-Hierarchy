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

   always @(posedge CLK,RESET) begin          //  asynchronous reset condition defind here 
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

module reg_file_tb;
    
    reg [7:0] WRITEDATA;
    reg [2:0] WRITEREG, READREG1, READREG2;
    reg CLK, RESET, WRITEENABLE; 
    wire [7:0] REGOUT1, REGOUT2;
    
    reg_file myregfile(WRITEDATA, REGOUT1, REGOUT2, WRITEREG, READREG1, READREG2, WRITEENABLE, CLK, RESET);
       
    initial
    begin
        CLK = 1'b1;
        
        // generate files needed to plot the waveform using GTKWave
          $dumpfile("reg_file_tb.vcd");
		      $dumpvars(0, reg_file_tb);
        
        // assign values with time to input signals to see output 
        RESET = 1'b1;
        WRITEENABLE = 1'b0;
        
        #1
        RESET = 1'b0;
        READREG1 = 3'd0;
        READREG2 = 3'd4;
        
        #8
        RESET = 1'b0;
        
        #5
        WRITEREG = 3'd2;
        WRITEDATA = 8'd95;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #1
        READREG1 = 3'd2;
        
        #9
        WRITEREG = 3'd1;
        WRITEDATA = 8'd28;
        WRITEENABLE = 1'b1;
        READREG1 = 3'd1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #10
        WRITEREG = 3'd4;
        WRITEDATA = 8'd6;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        
        #10
        WRITEENABLE = 1'b0;
        
        #6
        WRITEREG = 3'd1;
        WRITEDATA = 8'd50;
        WRITEENABLE = 1'b1;
        
        #5
        WRITEENABLE = 1'b0;

        #10
        WRITEREG = 3'd7;
        WRITEDATA = 8'd15;
        WRITEENABLE = 1'b1;
        READREG2 = 3'd7;
        
        #20
        RESET = 1'b1;
        #15
        RESET = 1'b0;
        $finish;
    end
    
    // clock signal generation
    always
        #5 CLK = ~CLK;
        

endmodule

 