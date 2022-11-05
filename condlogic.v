module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	NextPC,
	RegW,
	MemW,
	PCWrite,
	RegWrite,
	MemWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire NextPC;
	input wire RegW;
	input wire MemW;
	output wire PCWrite;
	output wire RegWrite;
	output wire MemWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;

	wire CurrentCondEx;
	wire PCSrc;
	flopenr #(2) flagreg1(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])
	);
	flopenr #(2) flagreg0(
		.clk(clk),
		.reset(reset),
		.en(FlagWrite[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
	);
	flopr #(1) condexreg(
		.clk(clk),
		.reset(reset),
		.d(CondEx),
		.q(CurrentCondEx)
	);
	condcheck cc(
		.Cond(Cond),
		.Flags(Flags),
		.CondEx(CondEx)
	);
	//assign FlagWrite = FlagW & {2 {CurrentCondEx}};
	assign FlagWrite = FlagW & {2 {CondEx}};
	assign RegWrite = RegW & CurrentCondEx;
	assign MemWrite = MemW & CurrentCondEx;
	assign PCSrc = PCS & CurrentCondEx;
	assign PCWrite = PCSrc | NextPC;
endmodule