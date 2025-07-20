//-----------------------------------------------------------------------------
//
// Title       : \Mux2-1\ 
// Design      : Mux
// Author      : rua
// Company     : berzeit
//
//-----------------------------------------------------------------------------
//
// File        : c:/My_Designs/ArchProject/Mux/src/Mux2-1.v
// Generated   : Sun Jan  5 22:40:51 2025
// From        : Interface description file
// By          : ItfToHdl ver. 1.0
//
//-----------------------------------------------------------------------------
//
// Description : 
//
//-----------------------------------------------------------------------------

`timescale 1ps / 1ps


////////////////////////////EXTENDER//////////////////////////////////////////////////////	
//this extender will extend the immiddiat value from 6 bits to 16 bit .
module extender (
	
	input [5:0] in, 
	input sign_ext, 
	output reg [15:0] out
);

	always @(*) begin 
		if (sign_ext && in[5])
			out = {10'hFFF, in};
		else 
			out = in ;
	end
	
endmodule	

///////////////////////////////REGISTER FILE/////////////////////////////////////////////  
//this is register file wich include 16 register with width 16 .
module reg_file (
	
	input clk,
	input regWrite,
	input [2:0] regDst,regSrc1,regSrc2,
	input [15:0] bus_w,
	output reg [15:0] out1, out2
);
	
  reg [15:0] regArray [0:7] = '{0, 8, 36, 3, 12,
                                66, 40, 30};	 
								  //must change

always @(*) begin //The output is taken asynchronously								
out1 = regArray[regSrc1];
out2 = regArray[regSrc2];	
	end
	
	always @(posedge clk) begin	
		
   if (regWrite)
	regArray[regDst] <= bus_w;
end		
	
endmodule
/////////////////////////////DATA MEMORY///////////////////////////////////////////////

module dataMemory #(
    parameter WIDTH = 100  // Parameterized memory size
)
(
    output reg [15:0] dataOut,
    input clk,
    input [15:0] address,
    input [15:0] dataIn,
    input memRead,
    input memWrite
);

    reg [15:0] mem [0:WIDTH-1]; // Memory array with parameterized size

    // Initialize memory
    initial begin
        if (WIDTH > 80) mem[80] = 8;
        if (WIDTH > 81) mem[81] = 44;
        if (WIDTH > 82) mem[82] = 32;  
        if (WIDTH > 83) mem[83] = -122;
        if (WIDTH > 84) mem[84] = -200;
        if (WIDTH > 85) mem[85] = -2;
        if (WIDTH > 86) mem[86] = 211;
        if (WIDTH > 87) mem[87] = 12;
        if (WIDTH > 95) mem[95] = 120;
    end

    // Write to memory on clock edge
    always @(posedge clk) begin
        if (memWrite && address < WIDTH) begin
            mem[address] <= dataIn;
        end
    end

    // Read from memory
    always @* begin
        if (memRead && address < WIDTH) begin
            dataOut = mem[address];
        end else begin
            dataOut = 16'b0;  // Default value
        end
    end

endmodule


//////////////////////////FLOP////////////////////////////////////////////	 
//this flop register will store the data betwen stages .
module flop (  
	
  output reg [15:0] out,
	input clk,
	input writeEn,
  input [15:0] in,
	input reset
);
	always @(posedge clk or negedge reset) begin
	
		if (!reset)
			out <= 0;
		
		else if (writeEn)
			out <= in; 
	end
	
endmodule
/////////////////////////////MUX2x1//////////////////////////////////////////
module mux2x1 #(
	parameter WIDTH = 16
) 	 

   (input [WIDTH-1:0] a, b,     
    input select,  
    output [WIDTH-1:0] out    
);


	assign out = select ? b : a;

endmodule 
/////////////////////////////////MUX4x1///////////////////////////////
module mux4x1  #(
	parameter WIDTH = 16
) 
	
	(input [WIDTH-1:0] a, b, c, d,  	 
	input [1:0] s,	 
	
	output reg [WIDTH-1:0] out
);
	
	
	always @* begin 
	
		case (s) 
			2'b00 : out = a;
			2'b01 : out = b;
			2'b10 : out = c;
			2'b11 : out = d;
		endcase
end


endmodule
//////////////////////////////////////////////////	

module ALU (
    input [2:0] ALU_OP, 
    input signed [15:0] a, 
    input signed [15:0] b,
    output zero, 
    output reg signed [15:0] result 
);

    // Status flags
    assign zero = (result == 0);

    // ALU Operations
    always @(*) begin
        case (ALU_OP)
            3'b000: result = a & b;             // AND operation
            3'b001: result = a + b;             // ADD operation
            3'b010: result = a - b;             // SUB operation
            3'b011: result = a << b[3:0];       // Logical shift left by b[3:0]
            3'b100: result = $signed(a) >>> b[3:0]; // Use $signed to enforce arithmetic shift

        endcase
    end

endmodule	
////////////////////////////////////////////////////////
module ALUcontrol (
    input [2:0] ALU_Ctr, //000 -> function ,001->ADD , 010 -> opCode 
    input [3:0] opcode,
    input [2:0] func,
    output reg [2:0] ALU_OP
);

parameter   
    AND = 3'b000,
    ADD = 3'b001,
    SUB = 3'b010,
    SLL = 3'b011,
    SRL = 3'b100;

always @* begin
	if(ALU_Ctr == 3'b000 )begin
		 ALU_OP = func;
		end else if (ALU_Ctr == ADD) begin
        // for fitch state , to calculate PC+1 
        ALU_OP = ADD;
    end else if(ALU_Ctr == 3'b010) begin
        // Determine ALU_OP based on opcode when ALU_Ctr is 010
        case (opcode)
            4'b0010: ALU_OP = AND; 
            4'b0011, 
            4'b0100, 
            4'b0101: ALU_OP = ADD;  // Example: ADD operation
            4'b0110, 
            4'b0111, 
            4'b1000: ALU_OP = SUB;  // Example: SUB operation
            default: ALU_OP = AND;  // Default: AND operation
        endcase
    end
end

endmodule
////////////////////////////////////////////////////////
module PCcontrol (
    input [3:0] opcode, 
    input z,
    input PCWriteUncond,
    output PCwrite
);
 
    // Parameters
    parameter BNE = 6'b000111;
    parameter BEQ = 6'b000110;
    parameter FOR = 6'b001000;

    // Wire declaration
    wire branch;

    // Continuous assignments
    assign branch = ((opcode == BNE) && !z) || 
                    ((opcode == BEQ) && z) || 
                    ((opcode == FOR) && !z); 

    assign PCwrite = (branch) || PCWriteUncond;

endmodule
	 

//////////////////////////////////////////////////////////////
module mainController (
    output reg PCWriteUncond, Reg_Des,
    SignedSel, IRwrite, RegWrite, RRWrite,
    MemWrite, MemRead, WB_Data,RsSrc,RtSrc,
    output reg [2:0] PC_Src, ALU_Ctr,
    output reg [1:0] ALUSrc1, ALUSrc2,
    input clk, reset,
    input [3:0] opcode,
    input [2:0] func  
	 
);
    reg jmp;
    reg [3:0] state; // State register

    wire logical;

    // State encoding
    parameter InstructionFetch = 0, 
              InstructionDecode = 1, 
              AddressComputation = 2, 
              LoadAccess = 3, 
              LoadCompletion = 4,
              StoreAccess = 5,
              R_TypeALU = 6,
              R_Type_ALU_Completion = 7,
              logicalALU = 8,
			 logical_ALU_Completion = 9, 
              Branch = 10, 
              ForState = 11,
              ForCompletion = 12;

    // Opcode and function encoding
    parameter R_Type = 4'b0000,
              ANDI = 4'b0010,
              ADDI = 4'b0011,
              LW = 4'b0100,
              SW = 4'b0101,
              BEQ = 4'b0110,
              BNE = 4'b0111,
              FOR = 4'b1000,
              J_Type = 4'b0001 ; 
			  		  
	parameter 
	JMP = 3'b000 ,
	CALL = 3'b001 ,
	RET	 = 3'b010 ;
	
	
   reg jump, call, rs ,rt,rd ;
   reg [2:0] PC ;  
   
   
   always @(*) begin
	   
	   jump = 1'b0 ;
	   call = 1'b0 ;
	   PC  = 3'b000; 
	   rs = 1'b0 ;
	   rt  = 1'b0 ;
	   rd = 1'b0 ;
	   
	   
	   call = (func == 3'b001);
	   		
	   case(func)	
		   
		 JMP : 	begin 
		 	jump = 1'b1 ;
	     	PC  = 3'b001;
		 
		 end 
		 
		 CALL : begin  
		 jump = 1'b1 ;
	     call = 1'b1 ;
	     PC  = 3'b000; 
		 end 
		 
		 RET : begin 
		 jump = 1'b1 ;
	     PC  = 3'b000; 
		 end 
		   
	   endcase	
	   
	   case(opcode)
		   R_Type,ANDI, ADDI,LW,SW :begin 
		   RsSrc = 1'b0 ;
		   RtSrc  = 1'b1 ;
		   Reg_Des = 1'b1 ;
		   end 
		   BEQ,BNE,FOR : begin
		   RsSrc = 1'b1 ;
		   RtSrc  = 1'b0 ;
		   Reg_Des = 1'b0 ;
		   end 
		   endcase

   end 	
	    
    // Define logical condition
    assign logical = ((opcode == ANDI) || (opcode == ADDI));

    always @(posedge clk or negedge reset) begin
        if (!reset)
            state <= InstructionFetch;
        else begin
            case (state)
                InstructionFetch: state <= InstructionDecode;

                InstructionDecode: begin
                    case (opcode)
                        R_Type: state <= R_TypeALU;
                        LW, SW: state <= AddressComputation;
                        BEQ, BNE: state <= Branch;
                        ANDI, ADDI: state <= logicalALU;
                        FOR: state <= ForState;
                        J_Type: state <= InstructionFetch ;
                        default: state <= InstructionFetch;
                    endcase
                end
									
		  	AddressComputation : begin
				  
				case (opcode)
					LW : state <= LoadAccess; 
					SW : state <= StoreAccess;
				endcase
			end
					
			LoadAccess : state <= LoadCompletion ;
			
			R_TypeALU : state <= R_Type_ALU_Completion ; 
			
			logicalALU : state <= logical_ALU_Completion ; 
			
			ForState : state <= ForCompletion ;
			
			StoreAccess,LoadCompletion,R_Type_ALU_Completion, 
			logical_ALU_Completion ,Branch, ForCompletion : 
				     state <=  InstructionFetch ;
		endcase	
		end	 
	
	end	  
		
	always @(*) begin
		
		SignedSel = 0; 
		IRwrite = 0;
		Reg_Des = 0;
		ALUSrc1= 0;
		ALUSrc2 = 0;
		RRWrite = 0;
		PC_Src = 0;
		MemWrite = 0;
		MemRead = 0;
		WB_Data = 0;
		RegWrite = 0; 
		PCWriteUncond = 0;
		ALU_Ctr = 3'b001; 
	    RsSrc = 1'b0 ;
		RtSrc  = 1'b0 ;

			
		case (state) 
			
			InstructionFetch : begin
				
				IRwrite = 1'b1;
  				PC_Src = 3'b010;
 				PCWriteUncond = 1'b1;
 				ALUSrc1=2'b00;
 				ALUSrc2=2'b10;
				ALU_Ctr = 3'b001;

			end
			
		  	InstructionDecode : begin
					
				SignedSel = 1;
 				ALUSrc1 = 2'b00;
				ALUSrc2 =   2'b01;
				ALU_Ctr = 3'b001;
				PC_Src = PC ; 
				PCWriteUncond = JMP; 
				RsSrc = rs ;
	            RtSrc = rt	;
	            Reg_Des = rd ;
				RRWrite = CALL;				
		
			end

		  	AddressComputation : begin
				  
				SignedSel = 1;
 				ALUSrc1 = 2'b01;
				ALUSrc2 = 2'b01; 
				ALU_Ctr = 3'b010;

			end	  
							
			 
			LoadAccess : MemRead = 1'b1;
	 		StoreAccess :	MemWrite = 1'b1; 
			 
			LoadCompletion : begin 
				   WB_Data = 1;
			       RegWrite = 1;
				   Reg_Des = 1;
			end 		
			
			R_TypeALU :	begin
				 
				ALUSrc1 = 2'b01;
				ALUSrc2 = 2'b00;
				ALU_Ctr = 3'b000 ;
			end	 
			
			R_Type_ALU_Completion :  begin
				RegWrite = 1'b1;	
				Reg_Des =  1'b1; //Rd
				end
			
			logicalALU : begin 
				
				SignedSel = logical ;
				ALUSrc1 = 2'b01;
				ALUSrc2 = 2'b01;
				ALU_Ctr = 3'b010 ;
				
			logical_ALU_Completion :  begin
				RegWrite = 1'b1;	
				Reg_Des =  1'b0; //Rt
				end				
			end	 
	
		Branch : begin	
			ALUSrc1= 2'b01;
			ALUSrc2 = 2'b00;  
			PC_Src = 3'b011;
			PCWriteUncond = 0;	
		end 
		
		ForState : begin 
			ALUSrc1= 2'b11;
			ALUSrc2 = 2'b10;  
			PC_Src = 3'b100;
			ALU_Ctr = 3'b010 ;
		end
		
	   ForCompletion: begin	
		   RegWrite = 1;	   //Rt	 
		   Reg_Des =  1'b0; //Rt
		   
	   end
	
		endcase
		
	end   

endmodule	 
/////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module mainController_tb;
    // Testbench variables
    reg clk;
    reg reset;
    reg [3:0] opcode;
    reg [2:0] func;

    wire PCWriteUncond, Reg_Des, SignedSel, IRwrite, RegWrite, RRWrite;
    wire MemWrite, MemRead, WB_Data, RsSrc, RtSrc;
    wire [2:0] PC_Src, ALU_Ctr;
    wire [1:0] ALUSrc1, ALUSrc2;

    // Instantiate the mainController module
    mainController uut (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .func(func),
        .PCWriteUncond(PCWriteUncond),
        .Reg_Des(Reg_Des),
        .SignedSel(SignedSel),
        .IRwrite(IRwrite),
        .RegWrite(RegWrite),
        .RRWrite(RRWrite),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .WB_Data(WB_Data),
        .RsSrc(RsSrc),
        .RtSrc(RtSrc),
        .PC_Src(PC_Src),
        .ALU_Ctr(ALU_Ctr),
        .ALUSrc1(ALUSrc1),
        .ALUSrc2(ALUSrc2)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period
    end

    // Test sequence
    initial begin
        // Initialize inputs
        reset = 0; opcode = 4'b0000; func = 3'b000;
        #10 reset = 1; // Release reset after 10ns

        // Print start of simulation
        $display("[Simulation Start]");

        // Test Instruction Fetch
        #10 opcode = 4'b0000; // R-Type
        $display("Time: %0t | State: Instruction Fetch | Opcode: %b | Func: %b", $time, opcode, func);

        #10 opcode = 4'b0010; // ANDI
        $display("Time: %0t | State: Instruction Decode | Opcode: %b | Func: %b", $time, opcode, func);

        #10 opcode = 4'b0011; // ADDI
        $display("Time: %0t | State: Instruction Decode | Opcode: %b | Func: %b", $time, opcode, func);

        // Test Jump Instructions
        #10 opcode = 4'b0001; func = 3'b000; // JMP
        $display("Time: %0t | Jump Instruction | Opcode: %b | Func: JMP (%b)", $time, opcode, func);

        #10 func = 3'b001; // CALL
        $display("Time: %0t | Jump Instruction | Opcode: %b | Func: CALL (%b)", $time, opcode, func);

        #10 func = 3'b010; // RET
        $display("Time: %0t | Jump Instruction | Opcode: %b | Func: RET (%b)", $time, opcode, func);

        // Test Load/Store Instructions
        #10 opcode = 4'b0100; // LW
        $display("Time: %0t | Load Word | Opcode: %b", $time, opcode);

        #10 opcode = 4'b0101; // SW
        $display("Time: %0t | Store Word | Opcode: %b", $time, opcode);

        // Test Branch Instructions
        #10 opcode = 4'b0110; // BEQ
        $display("Time: %0t | Branch Equal | Opcode: %b", $time, opcode);

        #10 opcode = 4'b0111; // BNE
        $display("Time: %0t | Branch Not Equal | Opcode: %b", $time, opcode);

        // Test FOR Loop
        #10 opcode = 4'b1000; // FOR
        $display("Time: %0t | FOR Loop | Opcode: %b", $time, opcode);

        // End simulation
        #50 $display("[Simulation End]");
        $finish;
    end
endmodule
///////////////////////////////////////////////////////	 
module mux8x1  #(
	parameter WIDTH = 16
) 
	
	(input [WIDTH-1:0] a, b, c, d ,e ,f , g , h,  	 
	input [2:0] s,	 
	
	output reg [WIDTH-1:0] out
);
	
	
	always @* begin 
	
		case (s) 
			3'b000 : out = a;
			3'b001 : out = b;
			3'b010 : out = c;
			3'b011 : out = d;
			3'b100 : out = e;
			3'b101 : out = f;
			3'b110 : out = g;
			3'b111 : out = h;
		endcase
end


endmodule 
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
module datapath ( 
	
	input PCwrite, Reg_Des, RsSrc, RtSrc,	clk, reset,
    SignedSel, IRwrite, RegWrite, RRWrite, WB_Data,
		  
	input [15:0] data_out_instructionMemory, data_out_dataMemory,
					  
	input [1:0] ALUSrc1, ALUSrc2,
	input [2:0] PC_Src, ALU_OP,
	
	output [3:0] opcode, 

	output [15:0] address_instruction, data_in, address_data,
	output z,
	output [15:0] ALU_result  // result of the ALU, input to the ALUout buffer
);

	wire [2:0] Rs, Rt, Rd; 
	wire [15:0] pc_temp;
	
	assign pc_temp = address_instruction;
	
	wire [15:0] instruction, Bus1, Bus2, B_operand, A_operand, PCtype, extended_immediate,
		ALU_operand1, ALU_operand2,  //output of the alu source 1 & 2 mux
		jumpTargetAddress, // output of the pc 8x1mux
		RR_address, //output of the RR buffer
		ALU_result_buffer; // output of the ALUout buffer 
	
	wire[15:0] MDR_out, registerResult;
	wire [5:0] immediate;


	assign opcode = instruction[15:12];
	assign Rd = instruction[11:9];
	assign Rs = instruction[8:6];
	assign Rt = instruction[5:3];
	
	assign immediate = instruction[5:0];
	assign jumpTargetAddress={address_instruction[15:9], instruction[8:0]};    // concatination with offset	
	assign data_in = B_operand;	
	assign address_data = ALU_result_buffer;

	//PC:
	flop pc(.out(address_instruction) , .clk(clk), .writeEn(PCwrite), .in(PCtype) , .reset(reset));	
	
	
	/*module reg_file (
	
	input clk,
	input regWrite,
	input [2:0] regDst,regSrc1,regSrc2,
	input [15:0] bus_w,
	output reg [15:0] out1, out2
);*/
	reg_file file (	
		.clk(clk),
		.regWrite(RegWrite), 
		.regDst(Rd), .regSrc1(Rs), .regSrc2(Rt),
		.bus_w(registerResult), 
		.out1(Bus1), .out2(Bus2)
	);

	//IR (Instruction Register) stores the instruction fetched from the instruction memory:
	flop IR (.out(instruction), .clk(clk), .writeEn(IRwrite), .in(data_out_instructionMemory), .reset(1'b1));	 
	
	//A buffer serves as a buffer for the first operand of the ALU (A_operand):
	flop A (.out(A_operand), .clk(clk), .writeEn(1'b1), .in(Bus1), .reset(1'b1)); 
	
	//B buffer serves as a buffer for the second operand of the ALU (B_operand):
	flop B (.out(B_operand), .clk(clk), .writeEn(1'b1), .in(Bus2), .reset(1'b1)); 	
	
	ALU alu(
	
		.ALU_OP(ALU_OP), 
		.a(ALU_operand1), 
		.b(ALU_operand2),
		.zero(z), 
		.result(ALU_result) 
	);
	
	//ALUout buffer stores the output of the ALU temporarily:
	flop ALUout(.out(ALU_result_buffer), .clk(clk), .writeEn(1'b1), .in(ALU_result), .reset(1'b1));
	
	//flop EXtoMEM(.out(data_in), .clk(clk), .writeEn(1'b1), .in(B_operand), .reset(1'b1));
	
	//MDR buufer stores data read from the data memory temporarily:
	flop MDR(.out(MDR_out), .clk(clk), .writeEn(1'b1), .in(data_out_dataMemory), .reset(1'b1));
	
	//Return Register: 
	flop RR(.out(RR_address), .clk(clk), .writeEn(RRWrite), .in(ALU_result), .reset(reset)); 

	//register source1 mux
	mux2x1 #(3) RegSrc1_mux (.a(Rd), .b(Rs), .select(RsSrc), .out(Rs)); 

	//register source2 mux
	mux2x1 #(3) RegSrc2_mux (.a(Rs), .b(Rt), .select(RtSrc), .out(Rt)); 

	//register destination mux
	mux2x1 #(3) RegDes_mux (.a(Rd), .b(Rs), .select(Reg_Des), .out(Rd)); 

	//alu source1 mux
	mux4x1 #(16) ALUsrc1_mux (.a(pc_temp), .b(A_operand), .c(B_operand), .d(16'b0), .s(ALUSrc1), .out(ALU_operand1));
	
	//alu source2 mux
	mux4x1 #(16) ALUsrc2_mux (.a(B_operand), .b(extended_immediate), .c(16'b1), .d(16'b0), .s(ALUSrc2), .out(ALU_operand2));
	
	// alu mux
	mux2x1 #(16) ALU_mux(.a(ALU_result_buffer), .b(MDR_out), .select(WB_Data), .out(registerResult));
	
	//pc mux 
	mux8x1 #(16) pc_mux(.a(RR_address), .b(jumpTargetAddress), .c(ALU_result), .d(ALU_result_buffer), .e(Bus1), .f(16'b0), .g(16'b0), .h(16'b0), .s(PC_Src), .out(PCtype));
	
	//Extender
	extender Extender(.in(immediate), .sign_ext(SignedSel), .out(extended_immediate));				  

endmodule		  
///////////////////////////////////////////////////////////////////////////////////////////////////////	  
		  
//////////////////////////////////////////////////////
module CPU ( 
	//this cpu module will combine the datapath with all the controls(ALU,Main,PC).
	
	input [15:0] data_out_dataMemory, data_out_instructionMemory, 
	input clk, reset,
	output [15:0] data_in, address_data, address_instruction,
	
	output MemWrite, MemRead,
	output [15:0] ALU_result,
	output [2:0] ALU_OP
);

	wire [3:0] opcode;
	wire [2:0] func , ALU_Ctr, PC_Src ;
	
	wire   RegWrite, Reg_Des,RsSrc,RtSrc,
		   IRwrite, PCWriteUncond,
		   SignedSel,  PCwrite ,RRWRite,WB_Data,RRWrite;
		   
	wire [1:0] ALUSrc1, ALUSrc2 ;
	
	wire z;


datapath DataPath ( 
	
.PCwrite(PCwrite), .Reg_Des(Reg_Des), .RsSrc(RsSrc), .RtSrc(RtSrc),	.clk(clk), .reset(reset),
.SignedSel(SignedSel), .IRwrite(IRwrite), .RegWrite(RegWrite), .RRWrite(RRWrite), .WB_Data(WB_Data),
		  
.data_out_instructionMemory(data_out_instructionMemory), .data_out_dataMemory(data_out_dataMemory),	

.data_in(data_in), .address_data(address_data),
					  
	.ALUSrc1(ALUSrc1), .ALUSrc2(ALUSrc2),
	.PC_Src(PC_Src), .ALU_OP(ALU_OP),
	
	.opcode(opcode), 

	.address_instruction(address_instruction),
	.z(z), 
	.ALU_result(ALU_result)  // result of the ALU, input to the ALUout buffer
); 

 mainController Main_Controller(
     .PCWriteUncond(PCWriteUncond), .Reg_Des(Reg_Des),
    .SignedSel(SignedSel), .IRwrite(IRwrite), .RegWrite(RegWrite), .RRWrite(RRWrite),
    .MemWrite(MemWrite), .MemRead(MemRead), .WB_Data(WB_Data), .RsSrc(RsSrc), .RtSrc(RtSrc),
     .PC_Src(PC_Src), .ALU_Ctr(ALU_Ctr),
    .ALUSrc1(ALUSrc1), .ALUSrc2(ALUSrc2),
    .clk(clk), .reset(reset),
    .opcode(opcode),
    .func(func)  
	 
); 


PCcontrol pc_control(
    .opcode(opcode), 
    .z(z),
    .PCWriteUncond(PCWriteUncond),
    .PCwrite(PCwrite)
);	
	
ALUcontrol ALU_control(
    .ALU_Ctr(ALU_Ctr),.opcode(opcode),
    .func(func),.ALU_OP(ALU_OP)
);	
	
endmodule	 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//combine between the cpu, datamemory, instruction memory .
module computer (
	
     input clk, reset,
     output [15:0] address_instruction, 
	 output [15:0] data_out_instructionMemory,
     				data_in, data_out_dataMemory, address_data,
  	 output MemRead,MemWrite,

	 output [15:0] ALU_result,
	 output [2:0] ALU_OP
  );

	
	instructionMemory inst_mem(
	
		.data(data_out_instructionMemory), 
		.clk(clk),
		.address(address_instruction)					  
	); 

	dataMemory data_mem(
	
		.dataOut(data_out_dataMemory),
		.clk(clk),
		.address(address_data),
		.dataIn(data_in),
		.memRead(MemRead), 
		.memWrite(MemWrite)
	);

	CPU cpu(
	
        .data_out_dataMemory(data_out_dataMemory),
        .data_out_instructionMemory(data_out_instructionMemory), 
        .clk(clk),
        .data_in(data_in),
        .address_data(address_data),
        .address_instruction(address_instruction),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
		.reset(reset), 
		.ALU_result(ALU_result),
		.ALU_OP(ALU_OP)
    );
	
endmodule  
////////////////////////////////////////////////////////////////////////////////
module instructionMemory #(
    parameter WIDTH = 16
) (
    output reg [15:0] data,    // Output will be 16 bits, consisting one word
    input [WIDTH-1:0] address,   // Address input
    input clk
);

    // Memory array to store words (16 bits each)
    reg [15:0] mem [0:32]; // Define memory with 16-bit width for word addressing

    // Initial block to initialize memory with some values
    initial begin
        // Storing instructions in little-endian format
        mem[0] = 16'h0899; // add r4, r2, r3
        mem[1] = 16'h0A9A; // sub r5, r2, r3
        mem[2] = 16'h0C9B; // SLL r6, r2, r3
        mem[3] = 16'h0E9C; // SRL r7, r2, r3

    end

    // Assign the selected memory address to the output data
    always @(posedge clk) begin
        data <= mem[address];
    end

endmodule  
/////////////////////////////////////////
`timescale 1ns / 1ps

module datapath_tb;

    // Inputs
    reg PCwrite, Reg_Des, RsSrc, RtSrc, clk, reset;
    reg SignedSel, IRwrite, RegWrite, RRWrite, WB_Data;
    reg [15:0] data_out_instructionMemory, data_out_dataMemory, data_in, address_data;
    reg [1:0] ALUSrc1, ALUSrc2;
    reg [2:0] PC_Src, ALU_OP;

    // Outputs
    wire [3:0] opcode;
    wire [15:0] address_instruction;
    wire z;
    wire [15:0] ALU_result;

    // Instantiate the datapath module
    datapath uut (
        .PCwrite(PCwrite), 
        .Reg_Des(Reg_Des), 
        .RsSrc(RsSrc), 
        .RtSrc(RtSrc), 
        .clk(clk), 
        .reset(reset), 
        .SignedSel(SignedSel), 
        .IRwrite(IRwrite), 
        .RegWrite(RegWrite), 
        .RRWrite(RRWrite), 
        .WB_Data(WB_Data), 
        .data_out_instructionMemory(data_out_instructionMemory), 
        .data_out_dataMemory(data_out_dataMemory), 
        .ALUSrc1(ALUSrc1), 
        .ALUSrc2(ALUSrc2), 
        .PC_Src(PC_Src), 
        .ALU_OP(ALU_OP), 
        .opcode(opcode), 
        .address_instruction(address_instruction), 
        .z(z), 
        .ALU_result(ALU_result),
		.data_in(data_in),
		.address_data(address_data)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize Inputs
        clk = 0;
        reset = 0; // Reset low initially
        PCwrite = 0;
        Reg_Des = 0;
        RsSrc = 0;
        RtSrc = 0;
        SignedSel = 0;
        IRwrite = 0;
        RegWrite = 0;
        RRWrite = 0;
        WB_Data = 0;
        data_out_instructionMemory = 16'b0;
        data_out_dataMemory = 16'b0;
        ALUSrc1 = 2'b0;
        ALUSrc2 = 2'b0;
        PC_Src = 3'b0;
        ALU_OP = 3'b0;

        // Assert reset
        #10;
        reset = 1;
        $display("Time: %0t | Reset Asserted", $time);

        #10;
        reset = 0; // Deassert reset
        $display("Time: %0t | Reset Deasserted", $time);

        // Debug instruction register and decoding
        $monitor("Time: %0t | Instruction Register: %b | Decoded Rs: %d | Rt: %d | Rd: %d", 
                 $time, uut.instruction, uut.Rs, uut.Rt, uut.Rd);

        // Load instruction into instruction register
        data_out_instructionMemory = 16'b0000000100100011; // ADD Rd=001, Rs=010, Rt=011
        IRwrite = 1; // Write instruction in the first cycle
        #10;
        IRwrite = 0;
        $display("Time: %0t | Instruction Loaded: %b", $time, uut.instruction);

        // Allow one cycle for decode
        #10;
        $display("Time: %0t | Decode Stage: Rs=%d | Rt=%d | Rd=%d", $time, uut.Rs, uut.Rt, uut.Rd);

        // Perform ALU operation
        ALUSrc1 = 2'b01; // Rs
        ALUSrc2 = 2'b00; // Rt
        ALU_OP = 3'b001; // ADD
        #10;

        // Debug ALU
        $display("Time: %0t | ALU Operand1: %d | ALU Operand2: %d | ALU Result: %d", $time, uut.ALU_operand1, uut.ALU_operand2, ALU_result);

        #50;
        $finish;
    end

endmodule

////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module reg_file_tb;

    // Inputs
    reg clk, regWrite;
    reg [2:0] regDst, regSrc1, regSrc2;
    reg [15:0] bus_w;

    // Outputs
    wire [15:0] out1, out2;

    // Instantiate the reg_file module
    reg_file uut (
        .clk(clk),
        .regWrite(regWrite),
        .regDst(regDst),
        .regSrc1(regSrc1),
        .regSrc2(regSrc2),
        .bus_w(bus_w),
        .out1(out1),
        .out2(out2)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize inputs
        clk = 0;
        regWrite = 0;
        regDst = 3'b0;
        regSrc1 = 3'b0;
        regSrc2 = 3'b0;
        bus_w = 16'b0;

        // Wait for global reset
        #10;

        // Write to register 1
        regDst = 3'b001; // Write to register 1
        bus_w = 16'd42;  // Value to write
        regWrite = 1;
        #10;
        regWrite = 0;

        // Read from register 1
        regSrc1 = 3'b001; // Read from register 1
        regSrc2 = 3'b010; // Read from register 2
        #10;
        $display("Register 1 Value: %d (Expected: 42)", out1);
        $display("Register 2 Value: %d (Expected: 36)", out2);

        // Write to register 2
        regDst = 3'b010; // Write to register 2
        bus_w = 16'd84;  // Value to write
        regWrite = 1;
        #10;
        regWrite = 0;

        // Read from registers
        regSrc1 = 3'b010; // Read from register 2
        regSrc2 = 3'b001; // Read from register 1
        #10;
        $display("Register 2 Value: %d (Expected: 84)", out1);
        $display("Register 1 Value: %d (Expected: 42)", out2);

        // Finish simulation
        #50;
        $finish;
    end

endmodule	 
///////////////////////////////////
module instruction_decoder_tb;
    reg [15:0] instruction; // 16-bit instruction input
    wire [3:0] opcode;      // 4-bit opcode
    wire [2:0] Rs, Rt, Rd;  // 3-bit fields

    // Decode logic
    assign opcode = instruction[15:12];
    assign Rd = instruction[11:9];
    assign Rs = instruction[8:6];
    assign Rt = instruction[5:3];

    initial begin
        instruction = 16'b0000000100100011; 
        #10;
        $display("Instruction: %b | Opcode: %b | Rs: %b | Rt: %b | Rd: %b", 
                 instruction, opcode, Rs, Rt, Rd);
        $finish;
    end
endmodule	
///////////////////////////////////////////////////////////////////



`timescale 1ns / 1ps

module Computer_tb;

    // Testbench signals
    reg clk;
    reg reset;
    wire [15:0] address_instruction;
    wire [15:0] data_out_instructionMemory;
    wire [15:0] data_in;
    wire [15:0] data_out_dataMemory;
    wire [15:0] address_data;
    wire MemRead, MemWrite;
    wire [15:0] ALU_result;
    wire [2:0] ALU_OP;

    // Instantiate the module under test
    computer uut (
        .clk(clk),
        .reset(reset),
        .address_instruction(address_instruction),
        .data_out_instructionMemory(data_out_instructionMemory),
        .data_in(data_in),
        .data_out_dataMemory(data_out_dataMemory),
        .address_data(address_data),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALU_result(ALU_result),
        .ALU_OP(ALU_OP)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #20 clk = ~clk; // 10 ns clock period
    end

    // Test sequence
    initial begin
        // Initialize inputs
        reset = 1;
        #10 reset = 0; // Deassert reset after 10ns
        #20 reset = 1;

        // Load initial instructions into instruction memory
        $display("[Time: %0t] Loading instructions into instruction memory", $time);

        // Example instruction sequence: ADD, SUB, AND, JMP, LW, SW, BEQ, BNE, FOR
        uut.inst_mem.mem[0] = 16'h0899; // ADD R4, R2, R3
        uut.inst_mem.mem[1] = 16'h0A9A; // SUB R5, R2, R3
        uut.inst_mem.mem[2] = 16'h0C9B; // SLL R6, R2, R3
        uut.inst_mem.mem[3] = 16'h0E9C; // SRL R7, R2, R3
        uut.inst_mem.mem[4] = 16'h2104; // LW R1, 4(R2)
        uut.inst_mem.mem[5] = 16'h2504; // SW R1, 4(R2)
        uut.inst_mem.mem[6] = 16'h3101; // BEQ R2, R1, 1
        uut.inst_mem.mem[7] = 16'h3701; // BNE R2, R1, 1
        uut.inst_mem.mem[8] = 16'h4000; // FOR R2, R3
        uut.inst_mem.mem[9] = 16'h1000; // JMP to address 0

        // Simulate basic operations
        $display("[Time: %0t] Simulating instruction fetch", $time);
        repeat (20) begin
            @(posedge clk);
            #1; // Small delay to allow signal propagation
        end

        // Wait and observe results
        $display("[Time: %0t] Testing completed", $time);
        $finish;
    end

    // Monitor signals for debugging
    initial begin
        $monitor("[Time: %0t] clk=%b, reset=%b, address_instruction=%h, data_out_instructionMemory=%h, data_in=%h, data_out_dataMemory=%h, address_data=%h, MemRead=%b, MemWrite=%b, ALU_result=%h, ALU_OP=%b", 
                 $time, clk, reset, address_instruction, data_out_instructionMemory, data_in, data_out_dataMemory, address_data, MemRead, MemWrite, ALU_result, ALU_OP);
    end

endmodule 

module counter (
    input clk,                  // Clock signal
    input reset,                // Reset signal
    input enable,               // Enable signal
    input [2:0] instruction_type, // Instruction type (3-bit to identify different types)
    output reg [31:0] total_instructions,  // Total executed instructions
    output reg [31:0] load_instructions,   // Load instructions
    output reg [31:0] store_instructions,  // Store instructions
    output reg [31:0] alu_instructions,    // ALU instructions
    output reg [31:0] control_instructions, // Control instructions
    output reg [31:0] clock_cycles,        // Total clock cycles

);

    // Instruction type codes
    parameter LOAD   = 3'b001;
    parameter STORE  = 3'b010;
    parameter ALU    = 3'b011;
    parameter CONTROL = 3'b100;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all counters
            total_instructions  <= 0;
            load_instructions   <= 0;
            store_instructions  <= 0;
            alu_instructions    <= 0;
            control_instructions <= 0;
            clock_cycles        <= 0;

        end else if (enable) begin
            // Increment clock cycle counter
            clock_cycles <= clock_cycles + 1;

            // Increment corresponding counters based on instruction type
            total_instructions <= total_instructions + 1;
            case (instruction_type)
                LOAD:   load_instructions <= load_instructions + 1;
                STORE:  store_instructions <= store_instructions + 1;
                ALU:    alu_instructions <= alu_instructions + 1;
                CONTROL: control_instructions <= control_instructions + 1;
            endcase

        end
    end
endmodule	  
/////////////////////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module counter_tb;

    // Inputs
    reg clk;
    reg reset;
    reg enable;
    reg [2:0] instruction_type;

    // Outputs
    wire [31:0] total_instructions;
    wire [31:0] load_instructions;
    wire [31:0] store_instructions;
    wire [31:0] alu_instructions;
    wire [31:0] control_instructions;
    wire [31:0] clock_cycles;

    // Instantiate the counter module
    counter uut (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .instruction_type(instruction_type),
        .total_instructions(total_instructions),
        .load_instructions(load_instructions),
        .store_instructions(store_instructions),
        .alu_instructions(alu_instructions),
        .control_instructions(control_instructions),
        .clock_cycles(clock_cycles)
    );

    // Clock generation (50 MHz)
    initial begin
        clk = 0;
        forever #10 clk = ~clk;  // Toggle clock every 10ns (50MHz clock)
    end

    // Testbench logic
    initial begin
        // Initialize inputs
        reset = 0;
        enable = 0;
        instruction_type = 3'b000;

        // Apply reset
        #20;
        reset = 1;
        #20;
        reset = 0;

        // Test with different instruction types
        enable = 1;

        // LOAD instruction
        instruction_type = 3'b001; // LOAD
        #20;  // Wait 20ns
        
        instruction_type = 3'b010; // STORE
        #20;  // Wait 20ns

        instruction_type = 3'b011; // ALU
        #20;  // Wait 20ns

        instruction_type = 3'b100; // CONTROL
        #20;  // Wait 20ns

        // Disable enable signal
        enable = 0;
        #40;

        // Check counter does not update when enable = 0
        instruction_type = 3'b001; // LOAD
        #40; 
        
        // End simulation
        $finish;
    end

endmodule