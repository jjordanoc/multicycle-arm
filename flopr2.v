module flopr2 (
	clk,
	reset,
	d0,
    d1,
	q0,
    q1
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d0;
    input wire [WIDTH - 1:0] d1;
	output reg [WIDTH - 1:0] q0;
    output reg [WIDTH - 1:0] q1;
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			q0 <= 0;
            q1 <= 0;
        end
		else begin
			q0 <= d0;
            q1 <= d1;
        end
    end
endmodule