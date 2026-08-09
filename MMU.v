`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// MMU.v  (SafeCore build -- PASS-THROUGH)
//
// SafeCore selection is performed in Top_Level_safecore by SNOOPING the DLX
// SDRAM-write bus at reserved addresses 0x7Ex (FIM) / 0x7Fx (WWDT). That snoop
// requires the access to stay on the SDRAM path (MMU_AO[24] = 0). Therefore the
// MMU must NOT remap any DLX address into the slave (bit24=1) space.
//
// The earlier 0xF0/0xF1 -> bit24=1 remapping is removed: it routed SafeCore
// writes to the host-only slave path (which never lands during a DLX run) and
// simultaneously defeated the snoop. This pass-through keeps every DLX access on
// the SDRAM/snoop path, which is the mechanism that provably works.
//
// To configure a SafeCore, the program does a normal store to 0x7E0..0x7FC.
//////////////////////////////////////////////////////////////////////////////////

module MMU(
    input  wire [31:0] AO,
    output wire [31:0] MMU_AO
);

    // Top 8 bits forced to 0 -> bit24 = 0 always -> SDRAM/snoop path for all
    // accesses, matching the IO_LOGIC RAM address range (0x0..0xFFFFF).
    assign MMU_AO = {8'b0, AO[23:0]};

endmodule
