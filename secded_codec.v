`timescale 1ns / 1ps

// ======================================================================
// Standard Hamming (38,32) + 1 Global Parity Bit for Double Error Detection
// ======================================================================
module secded_encoder (
    input  wire [31:0] data_in,
    output wire [38:0] data_encoded
);
    wire [5:0] p;
    wire       p_global;

    // pre calculated using a hamming matrix + XOR trees
    assign p[0] = data_in[0] ^ data_in[1] ^ data_in[3] ^ data_in[4] ^ data_in[6] ^ 
                  data_in[8] ^ data_in[10] ^ data_in[11] ^ data_in[13] ^ data_in[15] ^ 
                  data_in[17] ^ data_in[19] ^ data_in[21] ^ data_in[23] ^ data_in[25] ^ 
                  data_in[26] ^ data_in[28] ^ data_in[30];

    assign p[1] = data_in[0] ^ data_in[2] ^ data_in[3] ^ data_in[5] ^ data_in[6] ^ 
                  data_in[9] ^ data_in[10] ^ data_in[12] ^ data_in[13] ^ data_in[16] ^ 
                  data_in[17] ^ data_in[20] ^ data_in[21] ^ data_in[24] ^ data_in[25] ^ 
                  data_in[27] ^ data_in[28] ^ data_in[31];

    assign p[2] = data_in[1] ^ data_in[2] ^ data_in[3] ^ data_in[7] ^ data_in[8] ^ 
                  data_in[9] ^ data_in[10] ^ data_in[14] ^ data_in[15] ^ data_in[16] ^ 
                  data_in[17] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[25] ^ 
                  data_in[29] ^ data_in[30] ^ data_in[31];

    assign p[3] = data_in[4] ^ data_in[5] ^ data_in[6] ^ data_in[7] ^ data_in[8] ^ 
                  data_in[9] ^ data_in[10] ^ data_in[18] ^ data_in[19] ^ data_in[20] ^ 
                  data_in[21] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[25];

    assign p[4] = data_in[11] ^ data_in[12] ^ data_in[13] ^ data_in[14] ^ data_in[15] ^ 
                  data_in[16] ^ data_in[17] ^ data_in[18] ^ data_in[19] ^ data_in[20] ^ 
                  data_in[21] ^ data_in[22] ^ data_in[23] ^ data_in[24] ^ data_in[25];

    assign p[5] = data_in[26] ^ data_in[27] ^ data_in[28] ^ data_in[29] ^ data_in[30] ^ data_in[31];

    // global parity =, which is a XOR on all data and the hamming bits
    assign p_global = ^data_in ^ ^p;

    // final data writing
    assign data_encoded = {p_global, p, data_in};
endmodule

// ======================================================================
// the SECDED Decoder (39-bit Encoded -> 32-bit Data)
// ======================================================================
module secded_decoder (
    input  wire [38:0] data_encoded_in,
    output reg  [31:0] data_corrected_out,
    output reg         err_single_corrected,
    output reg         err_fatal_double
);
    wire [31:0] d;
    wire [5:0]  p_in;
    
    wire [5:0]  p_calc;
    wire [5:0]  syndrome;
    wire        global_syndrome;

    assign d    = data_encoded_in[31:0];
    assign p_in = data_encoded_in[37:32];

    // re calculating all of the data we got
    assign p_calc[0] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[17] ^ d[19] ^ d[21] ^ d[23] ^ d[25] ^ d[26] ^ d[28] ^ d[30];
    assign p_calc[1] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[16] ^ d[17] ^ d[20] ^ d[21] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[31];
    assign p_calc[2] = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[29] ^ d[30] ^ d[31];
    assign p_calc[3] = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25];
    assign p_calc[4] = d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[25];
    assign p_calc[5] = d[26] ^ d[27] ^ d[28] ^ d[29] ^ d[30] ^ d[31];

    // using the syndrome to point on the error
    assign syndrome = p_in ^ p_calc;
    
    // global syndrome which is a XOR on all of the data we got
    // getting 1 means we have one bit error, getting 0 meas we have two bit error (or a valid data)
    assign global_syndrome = ^data_encoded_in; 

    always @(*) begin
        data_corrected_out   = d;
        err_single_corrected = 1'b0;
        err_fatal_double     = 1'b0;

        if (syndrome == 6'b0) begin
            if (global_syndrome != 1'b0) begin
                // the syndrome is 0 yet the global is 1 which means the error is in the parity - data is good
                err_single_corrected = 1'b1; 
            end
        end else begin
            if (global_syndrome != 1'b0) begin
                // the syndrome is not 0 and the global is wrong as well which meanswe have a single bit error - data is good corrupt so we fix it
                err_single_corrected = 1'b1;
                
                // mapping the syndrome to the bit and flipping it to the correct value
                case (syndrome)
                    6'd3:  data_corrected_out[0]  = ~d[0];
                    6'd5:  data_corrected_out[1]  = ~d[1];
                    6'd6:  data_corrected_out[2]  = ~d[2];
                    6'd7:  data_corrected_out[3]  = ~d[3];
                    6'd9:  data_corrected_out[4]  = ~d[4];
                    6'd10: data_corrected_out[5]  = ~d[5];
                    6'd11: data_corrected_out[6]  = ~d[6];
                    6'd12: data_corrected_out[7]  = ~d[7];
                    6'd13: data_corrected_out[8]  = ~d[8];
                    6'd14: data_corrected_out[9]  = ~d[9];
                    6'd15: data_corrected_out[10] = ~d[10];
                    6'd17: data_corrected_out[11] = ~d[11];
                    6'd18: data_corrected_out[12] = ~d[12];
                    6'd19: data_corrected_out[13] = ~d[13];
                    6'd20: data_corrected_out[14] = ~d[14];
                    6'd21: data_corrected_out[15] = ~d[15];
                    6'd22: data_corrected_out[16] = ~d[16];
                    6'd23: data_corrected_out[17] = ~d[17];
                    6'd24: data_corrected_out[18] = ~d[18];
                    6'd25: data_corrected_out[19] = ~d[19];
                    6'd26: data_corrected_out[20] = ~d[20];
                    6'd27: data_corrected_out[21] = ~d[21];
                    6'd28: data_corrected_out[22] = ~d[22];
                    6'd29: data_corrected_out[23] = ~d[23];
                    6'd30: data_corrected_out[24] = ~d[24];
                    6'd31: data_corrected_out[25] = ~d[25];
                    6'd33: data_corrected_out[26] = ~d[26];
                    6'd34: data_corrected_out[27] = ~d[27];
                    6'd35: data_corrected_out[28] = ~d[28];
                    6'd36: data_corrected_out[29] = ~d[29];
                    6'd37: data_corrected_out[30] = ~d[30];
                    6'd38: data_corrected_out[31] = ~d[31];
                    default: ; 
                endcase
            end else begin
                // syndrome is not 0 yet parity is good - double bit error!
                err_fatal_double = 1'b1;
            end
        end
    end
endmodule
