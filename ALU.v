`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:32:01 01/12/2026 
// Design Name: 
// Module Name:    ALU 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module ALU(
	input [31:0] A,
	input [31:0] B,
	input test,
	input add,
	input [2:0] ALUF,
	output reg [31:0] ALU_OUT,
	// FIM SafeCore hook: XOR output with mask when inject_bit_flip=1
	input wire inject_bit_flip,
	input wire [31:0] bit_flip_mask
    );
	 
	 wire [31:0] S;
	 wire [2:0] F;
	 wire neg, sub, COMP_OUT;

	 // Internal combinational result before fault injection
	 wire [31:0] alu_raw;
	 reg  [31:0] alu_result;

	 assign F = (add) ? 3'b011: ALUF;
	 assign sub = test | (!F[0]);
	 
	 CSA32 add_sub32(
		.A(A[31:0]),
		.B(B[31:0]),
		.SUB(sub),
		.ALU_OUT(S[31:0]),
		.NEG(neg)
	);
	
	comparator comp(
		.S(S[31:0]),
		.neg(neg),
		.F(F[2:0]),
		.COMP_OUT(COMP_OUT)
	);
	
	// Compute clean ALU result
	always @(*) begin
		if (test) begin
			alu_result = {31'b0, COMP_OUT};
		end
		else if (!F[2]) begin
			alu_result = S;
		end
		else if (F[1]) begin
			alu_result = A & B;
		end
		else if (F[0]) begin
			alu_result = A | B;
		end
		else begin
			alu_result = A ^ B;
		end
	end

	// Apply bit-flip fault injection: XOR selected bits of the result
	always @(*) begin
		if (inject_bit_flip)
			ALU_OUT = alu_result ^ bit_flip_mask;
		else
			ALU_OUT = alu_result;
	end
		


endmodule
