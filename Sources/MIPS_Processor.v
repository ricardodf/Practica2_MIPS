/******************************************************************
Description
	* This is the top-level of a MIPS processor
	* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
	* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
	* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
	* This processor was made for computer organization class at ITESO.
******************************************************************/
module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 1024,
	parameter INCREMENT = 4
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//******************************************************************/
//******************************************************************/
assign  PortOut = 0;

//******************************************************************/
//******************************************************************/
// Data types to connect modules
wire BranchNE_wire;
wire BranchEQ_wire;
wire RegDst_wire;
wire NotZeroANDBrachNE;
wire ZeroANDBrachEQ;
wire ORForBranch;
wire ALUSrc_wire;
wire RegWrite_wire;
wire Zero_wire;
wire MemRead_wire;
wire MemtoReg_wire;
wire MemWrite_wire;
wire jumpWire;
wire jalWire;
wire jrWire;
wire [2:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [4:0] WriteRegister_wire;
wire [4:0] RaMux_Out;
wire [4:0] MuxPcJROut;
wire [27:0] i_shift_Wire;
wire [31:0] ReadDataOut_Wire;
wire [31:0] MUX_PX_wire;
wire [31:0] PC_wire;
wire [31:0] Instruction_wire;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] InmmediateExtend_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] ALUResult_wire;
wire [31:0] PC_4_wire;
wire [31:0] InmmediateExtendAnded_wire;
wire [31:0] PCtoBranch_wire;
wire [31:0] ReadDataALUResultOut_Wire;
wire [31:0] shiftBranch_wire;
wire [31:0] addBranchOut_wire;
wire [31:0] MuxBranch_Result;
wire [31:0] JumpOut;
wire [31:0] PCMux_Out;
wire [31:0] FinalPCMux_Out;
integer ALUStatus;
	
Control
ControlUnit
(
	.OP(Instruction_wire[31:26]),
	.instructionFunct(Instruction_wire[5:0]),
	.RegDst(RegDst_wire),
	.BranchEQ(BranchEQ_wire),
	.BranchNE(BranchNE_wire),
	.MemRead(MemRead_wire),
	.MemtoReg(MemtoReg_wire),
	.MemWrite(MemWrite_wire),
	.ALUSrc(alu_src_wire),
	.RegWrite(reg_write_wire),
	.ALUOp(ALUOp_wire),
	.jump(jumpWire),
	.jal(jalWire),
	.jr(jrWire)
);

PC_Register ProgramCounter(
    .clk(clk),
    .reset(reset),
    .NewPC(PC_4_wire),
    .PCValue(PC_wire)
);

ProgramMemory #(.MEMORY_DEPTH(MEMORY_DEPTH))
ROMProgramMemory
(
	.Address(PC_wire),
	.Instruction(Instruction_wire)
);

Adder32bits PC_Puls_4(
	.Data0(PC_wire),
	.Data1(4),
	.Result(PC_4_wire)
);

Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire),
	.MUX_Data0(Instruction_wire[20:16]),
	.MUX_Data1(Instruction_wire[15:11]),
	.MUX_Output(WriteRegister_wire)
);

RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(RegWrite_wire),
	.WriteRegister(WriteRegister_wire),
	.ReadRegister1(Instruction_wire[25:21]),
	.ReadRegister2(Instruction_wire[20:16]),
	.WriteData(ALUResult_wire),
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)
);

SignExtend
SignExtendForConstants
(   
   .DataInput(Instruction_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSrc_wire),
	.MUX_Data0(ReadData2_wire),
	.MUX_Data1(InmmediateExtend_wire),
	.MUX_Output(ReadData2OrInmmediate_wire)
);

ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire),
	.ALUFunction(Instruction_wire[5:0]),
	.ALUOperation(ALUOperation_wire)
);

ALU
Arithmetic_Logic_Unit 
(
	.ALUOperation(ALUOperation_wire),
	.A(ReadData1_wire),
	.B(ReadData2OrInmmediate_wire),
	.shamt(Instruction_wire[10:6]),
	.Zero(Zero_wire),
	.ALUResult(ALUResult_wire)
);

DataMemory 
#(	
	.DATA_WIDTH(32),
	.MEMORY_DEPTH(256)
)
RAM
(
	.WriteData(ReadData2_wire),
	.Address(ALUResult_wire),
	.MemWrite(MemWrite_wire),
	.MemRead(MemRead_wire),
	.clk(clk),
	.ReadData(ReadDataOut_Wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndALUResult
(
	.Selector(MemtoReg_wire),
	.MUX_Data0(ALUResult_wire),
	.MUX_Data1(ReadDataOut_Wire),
	.MUX_Output(ReadDataALUResultOut_Wire)
);
	
assign ALUResultOut = ALUResult_wire;
	
ShiftLeft2 
shiftBranch
(   
	.DataInput(InmmediateExtend_wire),
	.DataOutput(shiftBranch_wire)
);

Adder32bits
Add_ShiftBranch
(
	.Data0(PC_4_wire),
	.Data1(shiftBranch_wire),
	.Result(addBranchOut_wire)
);
	
Multiplexer2to1
#(
	.NBits(32)
)
Mux_AddBranch
(
	.Selector((BranchEQ_wire & Zero_wire)|(BranchNE_wire & ~Zero_wire)),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(addBranchOut_wire),
	.MUX_Output(MuxBranch_Result)
);

assign i_shift_Wire = Instruction_wire[25:0]<<2;

Multiplexer2to1
#(
	.NBits(32)
)
JumpMux
(
	.Selector(jumpWire),
	.MUX_Data0(MuxBranch_Result),
	.MUX_Data1({PC_4_wire[31:28] , i_shift_Wire}),
	.MUX_Output(JumpOut)
);

Multiplexer2to1
#(
	.NBits(5)
)
RaMux
(
	.Selector(jalWire),
	.MUX_Data0(WriteRegister_wire),
	.MUX_Data1(31),
	.MUX_Output(RaMux_Out)
);

Multiplexer2to1
#(
	.NBits(32)
)
PCMux
(
	.Selector(jalWire),
	.MUX_Data0(ReadDataALUResultOut_Wire),
	.MUX_Data1(PC_4_wire),
	.MUX_Output(PCMux_Out)
);

Multiplexer2to1
#(
	.NBits(32)
)
MuxPCJR
(
	.Selector(jrWire),
	.MUX_Data0(JumpOut),
	.MUX_Data1(ReadData1_wire),
	.MUX_Output(FinalPCMux_Out)
);
	
endmodule
