module mainfsm (
	clk,
	reset,
	Op,
	Funct,
	IRWrite,
	AdrSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	NextPC,
	RegW,
	MemW,
	Branch,
	ALUOp
);
	input wire clk;
	input wire reset;
	input wire [1:0] Op;
	input wire [5:0] Funct;
	output wire IRWrite;
	output wire AdrSrc;
	output wire ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ResultSrc;
	output wire NextPC;
	output wire RegW;
	output wire MemW;
	output wire Branch;
	output wire ALUOp;
	reg [3:0] state;
	reg [3:0] nextstate;
	reg [12:0] controls;

	localparam [3:0] FETCH = 0;
	localparam [3:0] DECODE = 1;
	localparam [3:0] MEMADR = 2;
	localparam [3:0] MEMRD = 3;
	localparam [3:0] MEMWB = 4;
	localparam [3:0] MEMWR = 5;
	localparam [3:0] EXECUTER = 6;
	localparam [3:0] EXECUTEI = 7;
	localparam [3:0] ALUWB = 8;
	localparam [3:0] BRANCH = 9;
	localparam [3:0] UNKNOWN = 10;
	
	// state register
	always @(posedge clk or posedge reset)
		if (reset)
			state <= FETCH;
		else
			state <= nextstate;

  	// next state logic
	always @(*)
		casex (state)
			FETCH: nextstate = DECODE;
			DECODE:
				case (Op)
					2'b00:
						if (Funct[5])
							nextstate = EXECUTEI;
						else
							nextstate = EXECUTER;
					2'b01: nextstate = MEMADR;
					2'b10: nextstate = BRANCH;
					default: nextstate = UNKNOWN;
				endcase
			MEMADR:
				if (Funct[0]) 
					nextstate = MEMRD;
				else
					nextstate = MEMWR;
			MEMRD: nextstate = MEMWB;
			MEMWB: nextstate = FETCH;
			MEMWR: nextstate = FETCH;
			EXECUTER: nextstate = ALUWB;
			EXECUTEI: nextstate = ALUWB;
			ALUWB: nextstate = FETCH;
			BRANCH: nextstate = FETCH;
			default: nextstate = FETCH;
		endcase

	// state-dependent output logic
	always @(*) begin
		case (state)
			FETCH: controls = 12'b100010101100;
			DECODE: controls = 12'b000000101100;
			MEMADR: controls = 12'b000000100010;
			MEMRD: controls = 12'b000001000010;
			MEMWB: controls = 12'b000101010010;
			MEMWR: controls = 12'b001001000010;
			EXECUTER: controls = 12'b000000100001;
			EXECUTEI: controls = 12'b000000100011;
			ALUWB: controls = 12'b000100000010;
			BRANCH: controls = 12'b010000100010;
			default: controls = 12'bxxxxxxxxxxxx;
		endcase
	end
	assign {NextPC, Branch, MemW, RegW, IRWrite, AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, ALUOp} = controls;
endmodule