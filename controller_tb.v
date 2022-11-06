`timescale 1ns/1ps
module controller_tb;
    reg clk, reset;
    reg [35:0] testvector[35:0];
    reg [31:0] vectornum;
    
    reg [31:0] errors;

    reg [31:0] Instr;
    reg [3:0] ALUFlags;
    wire [31:0] Result;
    reg [31:0] Expected;
    
    reg flag;
    reg Op;
    reg Funct;

    integer i = 0;

    wire PCWrite;
	wire MemWrite;
	wire RegWrite;
	wire IRWrite;
	wire AdrSrc;
	wire [1:0] RegSrc;
	wire [1:0] ALUSrcA;
	wire [1:0] ALUSrcB;
	wire [1:0] ResultSrc;
	wire [1:0] ImmSrc;
	wire [1:0] ALUControl;

    controller dut(.clk(clk), .reset(reset), .Instr(Instr[31:12]), .ALUFlags(ALUFlags), .PCWrite(PCWrite), .MemWrite(MemWrite), .RegWrite(RegWrite), .IRWrite(IRWrite), .AdrSrc(AdrSrc), .RegSrc(RegSrc), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ResultSrc(ResultSrc), .ImmSrc(ImmSrc), .ALUControl(ALUControl));

    always begin
      clk=1; #5; clk=0; #5;    
    end

    initial begin
      $readmemh("controller_tv.tv", testvector);
      errors=0;
      vectornum=0;
      reset <= 1; #2; reset<=0;
    end

    always @(posedge clk) begin

        if (i === 0) begin
            i = i + 1;
            Instr = testvector[vectornum][31+4:4];
            ALUFlags = testvector[vectornum][3:0];
            Op = Instr[27:26];
            Funct = Instr[20];
            vectornum=vectornum+1;
        end
        else if (i === 3 & Op === 2'b10) begin
            i = 0;
        end

        else if (i === 4 & Op === 2'b01 & Funct === 0)begin
            i = 0;
        end
        else if (i === 4 & Op === 2'b00)begin
            i = 0;
        end
        else if (i === 5 & Op === 2'b01 & Funct === 1)begin
            i = 0;
        end

        else begin
        i = i + 1;
        end

        if (testvector[vectornum][0] === 1'bx) begin
            $finish;
        end
    end



    initial begin
          $dumpfile("controller.vcd");
          $dumpvars;
    end
endmodule