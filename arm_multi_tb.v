`timescale 1ns/1ps
module testbench;
    integer i = 0;
	reg clk;
	reg reset;
	wire [31:0] WriteData;
	wire [31:0] Adr;
	wire MemWrite;
	top dut(
		.clk(clk),
		.reset(reset),
		.WriteData(WriteData),
		.Adr(Adr),
		.MemWrite(MemWrite)
	);
	initial begin
		reset <= 1;
		#(2)
			;
		reset <= 0;
	end
	always begin
		clk <= 1;
		#(5)
			;
		clk <= 0;
		#(5)
			;
        i <= i + 1;
	end
	always @(posedge clk) begin
		if (MemWrite) begin
			if ((Adr === 100) & (WriteData === 7)) begin
				$display("Simulation succeeded");
				$finish;
			end
			else if (Adr !== 96) begin
				$display("Simulation failed");
				$finish;
			end
        end
        if (i > 200) $finish;
    end
	initial begin
		$dumpfile("arm_multi.vcd");
		$dumpvars;
	end
endmodule