`timescale 1ns / 1ps

module fim_safecore (
    // RESA Bus Interface
    input wire clk,
    input wire rst_n,           // System Reset
    input wire [9:0] addr,      // Address offset
    input wire [31:0] data_in,  // Data from CPU
    output reg [31:0] data_out, // Data to CPU
    input wire wr_in_n,         // Write Enable (Active Low)
    input wire card_sel,        // Chip Select
    output reg sack_n,          // Slave Acknowledge

    // Fault Injection Hooks (Probes)
    output reg inject_pc_fault,     // Freeze PC Logic
    output reg inject_bit_flip,     // XOR manipulation on ALU datapath output
    output reg inject_mem_flip,     // Codeword corruption on SECDED read path
    output reg [31:0] bit_flip_mask // Which bits to flip
);

    // --- Register Map ---
    // Offset 0x0: Control Register
    //   Bit 0: Enable Fault Injection (Starts Timer)
    //   Bit 1: Fault Type (0 = PC Freeze, 1 = Bit Flip)
    //   Bit 2: Bit-Flip Target (0 = ALU datapath, 1 = memory/SECDED read path)
    //          (only meaningful when Bit 1 = 1)
    reg [31:0] fim_ctrl;    
    
    // Offset 0x4: Mask Register (For Bit-Flip)
    reg [31:0] fim_mask;    
    
    // Offset 0x8: Trigger Delay (How many clock cycles to wait before injecting)
    reg [31:0] fim_delay;   
    
    // Offset 0xC: Fault Duration (How many clock cycles the fault lasts. 32'hFFFFFFFF for stuck-at)
    reg [31:0] fim_duration; 

    // --- Internal State & Counters ---
    reg [31:0] timer;
    reg [31:0] duration_counter;
    reg [1:0] state;

    localparam S_IDLE      = 2'd0;
    localparam S_WAITING   = 2'd1;
    localparam S_INJECTING = 2'd2;
    localparam S_DONE      = 2'd3;

    // --- Bus Interface Logic ---
    wire write_req = card_sel && (~wr_in_n);
    wire read_req  = card_sel && (wr_in_n);

    // Handshake: Immediate Acknowledge
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sack_n <= 1'b1;
        else if (card_sel) sack_n <= 1'b0;
        else sack_n <= 1'b1;
    end

    // Memory Mapped I/O - Write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fim_ctrl <= 32'b0;
            fim_mask <= 32'b0;
            fim_delay <= 32'b0;
            fim_duration <= 32'd1; // Default duration: 1 cycle pulse
        end else if (write_req) begin
            case (addr[3:0])
                4'h0: fim_ctrl <= data_in;
                4'h4: fim_mask <= data_in;
                4'h8: fim_delay <= data_in;
                4'hC: fim_duration <= data_in;
            endcase
        end
    end

    // Memory Mapped I/O - Read
    always @(*) begin
        data_out = 32'h0;
        if (read_req) begin
            case (addr[3:0])
                4'h0: data_out = fim_ctrl;
                4'h4: data_out = fim_mask;
                4'h8: data_out = fim_delay;
                4'hC: data_out = fim_duration;
                default: data_out = 32'hDEADBEEF;
            endcase
        end
    end

    // --- Fault Injection State Machine ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            timer <= 32'b0;
            duration_counter <= 32'b0;
            inject_pc_fault <= 1'b0;
            inject_bit_flip <= 1'b0;
            inject_mem_flip <= 1'b0;
            bit_flip_mask <= 32'b0;
        end else begin
            // Default continuous assignments
            bit_flip_mask <= fim_mask;
            
            case (state)
                S_IDLE: begin
                    inject_pc_fault <= 1'b0;
                    inject_bit_flip <= 1'b0;
                    inject_mem_flip <= 1'b0;
                    if (fim_ctrl[0]) begin // Enable bit triggered
                        timer <= fim_delay;
                        duration_counter <= fim_duration;
                        state <= S_WAITING;
                    end
                end

                S_WAITING: begin
                    if (timer > 0) begin
                        timer <= timer - 1;
                    end else begin
                        state <= S_INJECTING;
                    end
                    
                    // Abort condition
                    if (!fim_ctrl[0]) state <= S_IDLE;
                end

                S_INJECTING: begin
                    // Assert the requested fault based on fim_ctrl[1] (type) and,
                    // for a bit-flip, fim_ctrl[2] (target: ALU vs memory path).
                    if (fim_ctrl[1] == 1'b0) begin
                        inject_pc_fault <= 1'b1;
                    end else begin
                        if (fim_ctrl[2] == 1'b0)
                            inject_bit_flip <= 1'b1;   // ALU output XOR
                        else
                            inject_mem_flip <= 1'b1;   // SECDED read-path corruption
                    end

                    // Handle fault duration
                    if (duration_counter > 0) begin
                        duration_counter <= duration_counter - 1;
                    end else begin
                        inject_pc_fault <= 1'b0;
                        inject_bit_flip <= 1'b0;
                        inject_mem_flip <= 1'b0;
                        state <= S_DONE;
                    end
                    
                    // Abort condition
                    if (!fim_ctrl[0]) state <= S_IDLE;
                end

                S_DONE: begin
                    inject_pc_fault <= 1'b0;
                    inject_bit_flip <= 1'b0;
                    inject_mem_flip <= 1'b0;
                    // Wait for CPU to clear enable bit before restarting
                    if (!fim_ctrl[0]) state <= S_IDLE;
                end
            endcase
        end
    end
endmodule
