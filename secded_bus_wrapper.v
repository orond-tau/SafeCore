`timescale 1ns / 1ps
// ==========================================================================
// secded_bus_wrapper
//
// Sits on the DI (memory read data) path between the I/O Control Logic and
// the DLX datapath.  On every memory read the encoder re-generates parity
// for the incoming 32-bit word, the decoder checks/corrects it, and the
// corrected word is forwarded to the DLX.
//
// Because the RESA SDRAM stores plain 32-bit words (no stored parity), this
// wrapper uses the encode->decode round-trip as an on-the-fly check:
//   1. Encode the 32-bit DI word to produce 39-bit codeword.
//   2. Let FIM optionally corrupt the codeword (fault injection test path).
//   3. Decode to recover corrected data and detect errors.
//
// Ports
//   di_raw          : 32-bit word from I/O Control Logic (memory bus)
//   inject_bit_flip : from FIM SafeCore (activates codeword corruption)
//   bit_flip_mask   : which of the 32 data bits to flip (from FIM)
//   di_corrected    : corrected 32-bit word forwarded to DLX datapath
//   err_single      : single-bit error was corrected this cycle
//   err_double      : double-bit (fatal) error detected — tie to monitor slave
// ==========================================================================
module secded_bus_wrapper (
    input  wire [31:0] di_raw,
    // FIM fault injection (used during self-test only; tie to 0 in production)
    input  wire        inject_bit_flip,
    input  wire [31:0] bit_flip_mask,
    // Outputs to DLX datapath and monitor slave
    output wire [31:0] di_corrected,
    output wire        err_single,
    output wire        err_double
);

    // Step 1: encode the raw word
    wire [38:0] encoded;
    secded_encoder enc (
        .data_in     (di_raw),
        .data_encoded(encoded)
    );

    // Step 2: optionally corrupt the data bits (for FIM self-test)
    // Only the lower 32 bits (data portion) are flipped; parity bits are untouched
    // so the decoder sees a realistic single/double-bit error scenario.
    wire [38:0] encoded_possibly_corrupt;
    assign encoded_possibly_corrupt = inject_bit_flip
        ? {encoded[38:32], encoded[31:0] ^ bit_flip_mask}
        : encoded;

    // Step 3: decode and correct
    secded_decoder dec (
        .data_encoded_in    (encoded_possibly_corrupt),
        .data_corrected_out (di_corrected),
        .err_single_corrected(err_single),
        .err_fatal_double   (err_double)
    );

endmodule
