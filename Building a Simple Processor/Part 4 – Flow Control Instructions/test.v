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
         { memory[0],memory[1],memory[2],memory[3]}=32'b00000000000000010000000000000011;
        { memory[4],memory[5],memory[6],memory[7]}=32'b00000000000001000000000000000001;
        { memory[8],memory[9],memory[10],memory[11]}=32'b00000000000000100000000000000101;
        { memory[12],memory[13],memory[14],memory[15]}=32'b00000000000000110000000000000011;
        { memory[16],memory[17],memory[18],memory[19]}=32'b00000010000001010000001000000011;
        { memory[20],memory[21],memory[22],memory[23]}=32'b00000011000000010000000100000100;
        { memory[24],memory[25],memory[26],memory[27]}=32'b00000001000001100000000000000001;
        { memory[28],memory[29],memory[30],memory[31]}=32'b00000111000000100000010000000001;
        { memory[32],memory[33],memory[34],memory[35]}=32'b00000110111111000000000000000000;
        { memory[36],memory[37],memory[38],memory[39]}=32'b00000000000000010000000000000011;
        { memory[40],memory[41],memory[42],memory[43]}=32'b00000000000001000000000000000001;
        { memory[44],memory[45],memory[46],memory[47]}=32'b00000000000000100000000000000101;
        { memory[48],memory[49],memory[50],memory[51]}=32'b00000000000000110000000000000011;

         
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
        #200
        $finish;
        
    end
    
    // clock signal generation
    always
        #5 clk = ~clk;
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
loadi 1 0x03
loadi 4 0x01
loadi 2 0x05
loadi 3 0x03

*/