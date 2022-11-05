`timescale 1ns/1ps
module controller_tb;
    reg clk, reset;
    reg [101:0] testvector[63:0];
    reg [31:0] vectornum;
    
    reg [31:0] errors;

    reg [31:0] Instr;
    reg [3:0] ALUFlags;
    wire [31:0] Result;
    reg [31:0] Expected;
    

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

    controller dut(.clk(clk), .reset(reset), .Instr(Instr), .ALUFlags(ALUFlags), .PCWrite(PCWrite), .MemWrite(MemWrite), .RegWrite(RegWrite), .IRWrite(IRWrite), .AdrSrc(AdrSrc), .RegSrc(RegSrc), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ResultSrc(ResultSrc), .ImmSrc(ImmSrc), .ALUControl(ALUControl))

    always begin
      clk=1; #5; clk=0; #5;    
    end

    initial begin
      $readmemh("controller_tv.tv", testvector);      // readmemb("filename",dest):  read file b is binary formay, h is hexadecimal format
      errors=0;
      vectornum=0;
      reset = 1; #2; reset=0;
    end

    always @(posedge clk) begin
        Instr = testvector[vectornum][31:0];
        ALUFlags = testvector[vectornum][32+4:32];
        Expected = testvector[vectornum][32+4+1+12:32+4+1];
    end

    always @(negedge clk) begin
        if  (~reset) begin
            if (Result !== Expected) begin
                $display("testvector: %h",testvector[vectornum]);
                $display("Vectornum: %d",vectornum);
                $display("instr: %d", Instr);
                $display("output Result:%b, expected Result:%b",Result,Expected);
                errors=errors+1;
            end
            vectornum=vectornum+1;
            if (testvector[vectornum][0] === 1'bx) begin
                $display("total errors: %d",errors);
                $finish;  
            end
        end
    end

    initial begin
          $dumpfile("controller.vcd");
          $dumpvars;
    end
endmodule