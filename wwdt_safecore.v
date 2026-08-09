`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// wwdt_safecore.v  (WINDOWED, PRESCALED -- committee build)
//
// Windowed Watchdog Timer SafeCore.
//
// Register map (byte offsets, decoded from addr[3:0]):
//   0x0  reg_ctrl     write nonzero -> arm (enter S_LOAD)
//   0x4  reg_kick     write 0xAA55AA55 to service the watchdog
//   0x8  reg_win_min  kick is only valid once counter has decremented to <= win_min
//   0xC  reg_win_max  countdown start value (timeout window, in TICKS)
//
// TIME BASE (PRESCALER):
//   The countdown decrements once every PRESCALE clock cycles (one "tick"),
//   NOT every clock. A windowed watchdog is serviced by software, whose kick
//   cadence is measured in instructions / memory transactions -- tens of clocks
//   apart -- not single cycles. Scaling the window into ticks puts win_min/
//   win_max in the same time domain as the kicking loop, so a software kick can
//   deterministically land inside the window. (Set PRESCALE=1 for the original
//   cycle-accurate behaviour.)
//
// KICK LATCH:
//   reg_kick self-clears one cycle after a write, so the magic value is visible
//   for a single clock. To guarantee the FSM observes every kick regardless of
//   bus timing, a magic write SETS a sticky kick_pending flag; the FSM consumes
//   and clears it. This removes the write-vs-FSM race on reg_kick.
//
// Behaviour:
//   S_IDLE:  reg_ctrl=0; wait for arming.
//   S_LOAD:  counter <= reg_win_max; move to S_COUNT.
//   S_COUNT: decrement counter once per tick. Kick handling:
//              valid kick  (counter <= reg_win_min) -> reload (S_LOAD)
//              early kick  (counter >  reg_win_min) -> S_FAULT_FAST
//            counter == 0 (timeout)                 -> S_FAULT_SLOW
//            IRQ asserted when counter < IRQ_THRESHOLD ticks.
//   S_FAULT_*: wd_reset_n stays low PERSISTENTLY until rst_n (system reset)
//              clears the WWDT, holding CONTROL_SDLXSM and the DLX in reset.
//////////////////////////////////////////////////////////////////////////////////

module wwdt_safecore (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [9:0]  addr,
    input  wire [31:0] data_in,
    output reg  [31:0] data_out,
    input  wire        wr_in_n,
    input  wire        card_sel,
    output reg         sack_n,
    output reg         irq_out,
    output reg         wd_reset_n,
    // DEBUG: expose captured registers directly (combinational, always valid)
    output wire [31:0] dbg_ctrl,
    output wire [31:0] dbg_win_min,
    output wire [31:0] dbg_win_max,
    output wire [31:0] dbg_state
);

    // ---- Tunable parameters ------------------------------------------------
    // PRESCALE: clock cycles per countdown tick. Chosen so one tick comfortably
    // exceeds one software kick-loop iteration (a few stores = tens of clocks).
    // 1024 ticks at 60 MHz ~= 17 us per tick -- a software loop kicks easily
    // within a window of a few ticks, while the no-kick timeout still fires fast.
    localparam PRESCALE      = 32'd1024;
    localparam KICK_MAGIC    = 32'hAA55AA55;
    localparam IRQ_THRESHOLD = 32'd4;     // assert IRQ when within 4 ticks of timeout

    // Register file
    reg [31:0] reg_ctrl;
    reg [31:0] reg_kick;
    reg [31:0] reg_win_min;
    reg [31:0] reg_win_max;

    // FSM
    reg [31:0] counter;       // in ticks
    reg [31:0] prescale_cnt;  // clocks within the current tick
    reg [2:0]  state;

    // Sticky kick flag: set when a magic kick is written, cleared by the FSM.
    reg        kick_pending;
    // Edge-detect for kick_write: a kick store spans several clocks (multi-cycle
    // SDRAM handshake), but it is ONE logical kick. Register kick_write and act
    // only on its rising edge so a single store = exactly one kick event.
    reg        kick_write_d;

    localparam S_IDLE       = 3'd0;
    localparam S_LOAD       = 3'd1;
    localparam S_COUNT      = 3'd2;
    localparam S_FAULT_FAST = 3'd3;
    localparam S_FAULT_SLOW = 3'd4;

    // DEBUG readback taps (diagnostic only)
    assign dbg_ctrl    = reg_ctrl;
    assign dbg_win_min = reg_win_min;
    assign dbg_win_max = reg_win_max;
    assign dbg_state   = {16'hBEEF, 13'b0, state};  // known-good BEEF build

    wire write_req = card_sel && (~wr_in_n);
    wire read_req  = card_sel && ( wr_in_n);

    // A magic kick is being written this cycle (combinational detect).
    wire kick_write = write_req && (addr[3:0] == 4'h4) && (data_in == KICK_MAGIC);

    // Slave acknowledge (high default, low while selected)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        sack_n <= 1'b1;
        else if (card_sel) sack_n <= 1'b0;
        else               sack_n <= 1'b1;
    end

    // Register write logic (data_in is MDO, the DLX write bus)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ctrl    <= 32'd0;
            reg_win_min <= 32'd0;
            reg_win_max <= 32'd0;
            reg_kick    <= 32'd0;
        end else if (write_req) begin
            case (addr[3:0])
                4'h0: reg_ctrl    <= data_in;
                4'h4: reg_kick    <= data_in;
                4'h8: reg_win_min <= data_in;
                4'hC: reg_win_max <= data_in;
            endcase
        end else begin
            reg_kick <= 32'b0;  // self-clearing (kept for readback visibility)
        end
    end

    // Read logic
    always @(*) begin
        data_out = 32'h0;
        if (read_req) begin
            case (addr[4:0])
                5'h00:   data_out = reg_ctrl;
                5'h08:   data_out = reg_win_min;
                5'h0C:   data_out = reg_win_max;
                5'h10:   data_out = {29'b0, state};
                default: data_out = 32'hDEADBEEF;
            endcase
        end
    end

    // ---- WWDT core FSM --------------------------------------------------
    // wd_reset_n is explicitly assigned IN EVERY STATE (no top-of-block
    // default). In S_FAULT_FAST/SLOW it stays 0 every cycle until rst_n clears
    // the FSM, holding CONTROL_SDLXSM's internal_reset asserted.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            counter      <= 32'b0;
            prescale_cnt <= 32'b0;
            wd_reset_n   <= 1'b1;
            irq_out      <= 1'b0;
            kick_pending <= 1'b0;
            kick_write_d <= 1'b0;
        end else begin
            // Sticky kick capture: latch ONLY on the rising edge of kick_write,
            // so one store (which spans several clocks) is exactly one kick.
            kick_write_d <= kick_write;
            if (kick_write && !kick_write_d)
                kick_pending <= 1'b1;

            case (state)
                S_IDLE: begin
                    wd_reset_n   <= 1'b1;
                    irq_out      <= 1'b0;
                    prescale_cnt <= 32'b0;
                    kick_pending <= 1'b0;          // ignore kicks while disarmed
                    if (reg_ctrl != 32'b0) state <= S_LOAD;
                end

                S_LOAD: begin
                    wd_reset_n   <= 1'b1;
                    irq_out      <= 1'b0;
                    counter      <= reg_win_max;
                    prescale_cnt <= 32'b0;
                    kick_pending <= 1'b0;          // fresh window, drop stale kicks
                    state        <= S_COUNT;
                end

                S_COUNT: begin
                    wd_reset_n <= 1'b1;

                    // ---- prescaled countdown: one tick per PRESCALE clocks ----
                    if (prescale_cnt >= (PRESCALE - 1)) begin
                        prescale_cnt <= 32'b0;
                        if (counter > 32'b0)
                            counter <= counter - 32'b1;
                    end else begin
                        prescale_cnt <= prescale_cnt + 32'b1;
                    end

                    // ---- kick handling (windowed) ----
                    // Consume a pending kick. Valid only once the counter has
                    // decremented into the window (counter <= win_min); a kick
                    // while still above the window is an early/illegal kick.
                    if (kick_pending) begin
                        kick_pending <= 1'b0;
                        if (counter > reg_win_min)
                            state <= S_FAULT_FAST;     // kicked too early
                        else
                            state <= S_LOAD;           // valid kick -> reload
                    end else if (counter == 32'b0) begin
                        state <= S_FAULT_SLOW;         // timeout
                    end

                    // IRQ as the window nears expiry
                    if (counter < IRQ_THRESHOLD && counter > 32'b0)
                        irq_out <= 1'b1;
                    else
                        irq_out <= 1'b0;
                end

                S_FAULT_FAST, S_FAULT_SLOW: begin
                    wd_reset_n   <= 1'b0;   // PERSISTENT until rst_n
                    irq_out      <= 1'b0;
                    kick_pending <= 1'b0;
                end

                default: begin
                    wd_reset_n   <= 1'b1;
                    irq_out      <= 1'b0;
                    kick_pending <= 1'b0;
                    state        <= S_IDLE;
                end
            endcase
        end
    end

endmodule
