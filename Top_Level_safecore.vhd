--------------------------------------------------------------------------------
-- Top_Level_safecore.vhd  (CLEAN BUILD)
--
-- DLX + SafeCore wrapper. Replaces the schematic-generated Top_Level entity
-- as the ISE top module. Adds FIM (Fault Injection), WWDT (Windowed Watchdog
-- Timer), and SECDED to the existing DLX bus.
--
-- SAFECORE SELECT (SDRAM-write SNOOP -- see detailed note at the decode below):
--   The MMU is a pass-through; SafeCores are selected by snooping DLX SDRAM
--   writes at reserved addresses. A program configures a SafeCore with an
--   ordinary store:
--     WWDT (0x7Fx):  0x7F0 ctrl(arm)  0x7F4 kick  0x7F8 win_min  0x7FC win_max
--     FIM  (0x7Ex):  0x7E0 ctrl       0x7E4 mask  0x7E8 delay    0x7EC duration
--
-- RESET PATH (TRUE RESET):
--   The DLX is reset by RESET_combined = RESET OR (not wd_reset_n). A WWDT
--   timeout drives wd_reset_n low and latches it, so RESET_combined holds the
--   ENTIRE core -- CONTROL_SDLXSM and the full datapath (PC->0, A,B,C,MDR,MAR) --
--   in reset until a system RESET (RESA &Reset toggle) clears the WWDT.
--
-- SECDED:
--   Gated by the SECDED_ENABLE constant below. Default FALSE = baseline data
--   path (DLX reads raw DO, direct ACK_N) so first hardware bring-up is the
--   known-good baseline. Set TRUE to route corrected data + the matched
--   one-cycle ACK delay (final validation stage).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Top_Level_safecore is
   port (
      fpgaClk_i : in    std_logic;
      sdClkFb_i : in    std_logic;
      sdAddr_o  : out   std_logic_vector (12 downto 0);
      sdBs_o    : out   std_logic_vector (1 downto 0);
      sdCas_bo  : out   std_logic;
      sdCe_bo   : out   std_logic;
      sdCke_o   : out   std_logic;
      sdClk_o   : out   std_logic;
      sdDqmh_o  : out   std_logic;
      sdDqml_o  : out   std_logic;
      sdRas_bo  : out   std_logic;
      sdWe_bo   : out   std_logic;
      sdData_io : inout std_logic_vector (15 downto 0)
   );
end Top_Level_safecore;

architecture BEHAVIORAL of Top_Level_safecore is

   ---------------------------------------------------------------------------
   -- Internal signals (mirror original Top_Level + SafeCore additions)
   ---------------------------------------------------------------------------
   signal ACK_N             : std_logic;
   signal AEQZ              : std_logic;
   signal AI                : std_logic_vector (9 downto 0);
   signal ALUD              : std_logic_vector (2 downto 0);
   signal AS_N              : std_logic;
   signal busy              : std_logic;
   signal Card_Sel          : std_logic;
   signal CLK               : std_logic;
   signal D                 : std_logic_vector (31 downto 0);
   signal DINsel            : std_logic;
   signal DLX_STATE         : std_logic_vector (4 downto 0);
   signal DO                : std_logic_vector (31 downto 0);  -- SDRAM read data
   signal IN_INIT           : std_logic;
   signal ir                : std_logic_vector (31 downto 0);
   signal MAC_STATE         : std_logic_vector (1 downto 0);
   signal MAO               : std_logic_vector (31 downto 0);
   signal MDO               : std_logic_vector (31 downto 0);  -- DLX write data
   signal MDRsel            : std_logic;
   signal monitored_signals : std_logic_vector (31 downto 0);
   signal RACK_N            : std_logic;
   signal REG_ADR           : std_logic_vector (4 downto 0);
   signal RESET             : std_logic;
   signal RESET_combined    : std_logic;
   signal Rsel              : std_logic;
   signal R_DO              : std_logic_vector (31 downto 0);
   signal SACK_N            : std_logic;
   signal SDO               : std_logic_vector (31 downto 0);
   signal STEP_EN           : std_logic;
   signal STOP_N            : std_logic;
   signal S1SEL             : std_logic_vector (1 downto 0);
   signal S2SEL             : std_logic_vector (1 downto 0);
   signal WR_IN_N           : std_logic;
   signal WR_OUT_N          : std_logic;

   -- Schematic internal control signals
   signal XLXN_24 : std_logic;  -- MW
   signal XLXN_25 : std_logic;  -- MR
   signal XLXN_27 : std_logic;  -- GPRwe
   signal XLXN_28 : std_logic;  -- Asel
   signal XLXN_29 : std_logic;  -- IRce
   signal XLXN_31 : std_logic;  -- MARce
   signal XLXN_33 : std_logic;  -- MDRce
   signal XLXN_34 : std_logic;  -- PCce
   signal XLXN_35 : std_logic;  -- Cce
   signal XLXN_36 : std_logic;  -- Bce
   signal XLXN_37 : std_logic;  -- Ace
   signal XLXN_38 : std_logic;  -- right
   signal XLXN_39 : std_logic;  -- shift
   signal XLXN_40 : std_logic;  -- test
   signal XLXN_41 : std_logic;  -- add
   signal XLXN_45 : std_logic;  -- Itype
   signal XLXN_46 : std_logic;  -- jlink
   signal XLXN_48 : std_logic;  -- halt
   signal XLXN_63 : std_logic;  -- init

   ---------------------------------------------------------------------------
   -- SafeCore signals
   ---------------------------------------------------------------------------
   signal rst_n_sig         : std_logic;
   signal fim_sack_n        : std_logic;
   signal fim_data_out      : std_logic_vector (31 downto 0);
   signal inject_pc_fault   : std_logic;
   signal inject_bit_flip   : std_logic;
   signal inject_mem_flip   : std_logic;
   signal bit_flip_mask     : std_logic_vector (31 downto 0);
   signal wwdt_card_sel     : std_logic;
   signal wwdt_sack_n       : std_logic;
   signal wwdt_data_out     : std_logic_vector (31 downto 0);
   -- Two-phase snoop capture: decode the register offset during the ADDRESS
   -- phase (AS_N, MAO valid), then capture data one cycle later (as_n_mem, when
   -- the IO logic commits the write and MDO is valid -- by which time MAO has
   -- already been released). No single-instant capture works on this bus.
   signal wwdt_wr_hit       : std_logic;   -- write addr-phase decode (0x7Fx)
   signal fim_wr_hit        : std_logic;   -- write addr-phase decode (0x7Ex)
   signal wwdt_wr_hit_d     : std_logic;
   signal fim_wr_hit_d      : std_logic;
   signal snoop_off         : std_logic_vector (3 downto 0);  -- latched reg offset
   signal snoop_addr        : std_logic_vector (9 downto 0);  -- {6'b0, snoop_off}
   signal wwdt_wr           : std_logic;   -- 1-cycle write pulse at T1
   signal fim_wr            : std_logic;
   -- bus-probe detectors (logic-analyzer diagnostic)
   signal mao_is_7fx        : std_logic;
   signal mdo_is_AB         : std_logic;
   -- DEBUG readback taps from WWDT + pending-read intercept
   signal dbg_ctrl          : std_logic_vector (31 downto 0);
   signal dbg_win_min       : std_logic_vector (31 downto 0);
   signal dbg_win_max       : std_logic_vector (31 downto 0);
   signal dbg_state         : std_logic_vector (31 downto 0);
   signal dbg_value         : std_logic_vector (31 downto 0);
   signal rd_hit            : std_logic;   -- read addr-phase decode (0x7Fx)
   signal rd_pending        : std_logic;   -- held across the whole read access
   signal rd_off            : std_logic_vector (3 downto 0);
   signal wwdt_irq          : std_logic;
   signal wd_reset_n        : std_logic;
   signal DI_corrected      : std_logic_vector (31 downto 0);
   -- SECDED pipeline registers (break the DO->SECDED->DLX critical path so the
   -- design meets the 60 MHz SDRAM clock). DO_r registers the raw read word
   -- before SECDED; ACK_N_d delays the memory acknowledge by one cycle to match
   -- the one-cycle SECDED pipeline latency, so the DLX latches the *corrected*
   -- word exactly when it sees the (delayed) ack.
   signal DO_r              : std_logic_vector (31 downto 0);
   signal ACK_N_d           : std_logic;
   signal secded_err_single : std_logic;
   signal secded_err_double : std_logic;
   -- Sticky SECDED status: once an error is corrected/detected, the flag latches
   -- so a program can read it after the access. Readable via dbg_value (0x7FC,
   -- upper bits) so the correction/detection demo can PROVE an error was caught.
   signal secded_seen_single : std_logic := '0';
   signal secded_seen_double : std_logic := '0';
   signal secded_status      : std_logic_vector (31 downto 0);

   ---------------------------------------------------------------------------
   -- SECDED enable switch (final-stage; default FALSE = baseline data path).
   -- When FALSE: DLX reads raw DO with direct ACK_N (known-good baseline).
   -- When TRUE : DLX reads DI_corrected with the matched one-cycle ACK_N_d.
   ---------------------------------------------------------------------------
   constant SECDED_ENABLE   : boolean := true;   -- OPTION A: SECDED on, full wait-state pipeline
   signal dlx_di_src        : std_logic_vector (31 downto 0);
   signal dlx_ack_src       : std_logic;

   ---------------------------------------------------------------------------
   -- Component declarations
   ---------------------------------------------------------------------------
   component IO_LOGIC_U
      port (
         AS_N_i    : in    std_logic;
         fpgaClk_i : in    std_logic;
         in_init_i : in    std_logic;
         RACK_N_i  : in    std_logic;
         SACK_N_i  : in    std_logic;
         sdClkFb_i : in    std_logic;
         WR_N_i    : in    std_logic;
         MA_i      : in    std_logic_vector (31 downto 0);
         MD_i      : in    std_logic_vector (31 downto 0);
         RD_i      : in    std_logic_vector (31 downto 0);
         SD_i      : in    std_logic_vector (31 downto 0);
         CLK       : out   std_logic;
         MACK_N_o  : out   std_logic;
         RESET     : out   std_logic;
         Rsel_o    : out   std_logic;
         sdCas_bo  : out   std_logic;
         sdCe_bo   : out   std_logic;
         sdCke_o   : out   std_logic;
         sdClk_o   : out   std_logic;
         sdDqmh_o  : out   std_logic;
         sdDqml_o  : out   std_logic;
         sdRas_bo  : out   std_logic;
         sdWe_bo   : out   std_logic;
         Ssel_o    : out   std_logic;
         step_en_o : out   std_logic;
         WR_N_o    : out   std_logic;
         A_o       : out   std_logic_vector (9 downto 0);
         DO        : out   std_logic_vector (31 downto 0);
         sdAddr_o  : out   std_logic_vector (12 downto 0);
         sdBs_o    : out   std_logic_vector (1 downto 0);
         sdData_io : inout std_logic_vector (15 downto 0)
      );
   end component;

   component monitor_slave_MUSER_Top_Level
      port (
         IN_INIT : in    std_logic;
         STEP_EN : in    std_logic;
         STOP_N  : in    std_logic;
         D_IN    : in    std_logic_vector (31 downto 0);
         clk     : in    std_logic;
         A_IN    : in    std_logic_vector (31 downto 0);
         B_IN    : in    std_logic_vector (31 downto 0);
         Cardsel : in    std_logic;
         WR_N    : in    std_logic;
         AI      : in    std_logic_vector (9 downto 0);
         SACK_N  : out   std_logic;
         SDO     : out   std_logic_vector (31 downto 0);
         reg_adr : out   std_logic_vector (4 downto 0)
      );
   end component;

   component DLX_MUSER_Top_Level
      port (
         CLK             : in    std_logic;
         RESET           : in    std_logic;
         DI              : in    std_logic_vector (31 downto 0);
         Dadr            : in    std_logic_vector (4 downto 0);
         step_en         : in    std_logic;
         ACK_N           : in    std_logic;
         inject_pc_fault : in    std_logic;
         inject_bit_flip : in    std_logic;
         bit_flip_mask   : in    std_logic_vector (31 downto 0);
         wd_reset_n      : in    std_logic;
         AEQZ            : out   std_logic;
         add             : out   std_logic;
         test            : out   std_logic;
         shift           : out   std_logic;
         right           : out   std_logic;
         Ace             : out   std_logic;
         Bce             : out   std_logic;
         Cce             : out   std_logic;
         PCce            : out   std_logic;
         MDRce           : out   std_logic;
         MDRsel          : out   std_logic;
         MARce           : out   std_logic;
         DINce           : out   std_logic;
         Asel            : out   std_logic;
         IRce            : out   std_logic;
         func            : out   std_logic_vector (5 downto 0);
         MMU_AO          : out   std_logic_vector (31 downto 0);
         D               : out   std_logic_vector (31 downto 0);
         opcode          : out   std_logic_vector (5 downto 0);
         S1SEL           : out   std_logic_vector (1 downto 0);
         S2SEL           : out   std_logic_vector (1 downto 0);
         Itype           : out   std_logic;
         jlink           : out   std_logic;
         init            : out   std_logic;
         halt            : out   std_logic;
         ALUF            : out   std_logic_vector (2 downto 0);
         DLX_STATE       : out   std_logic_vector (4 downto 0);
         wr_n            : out   std_logic;
         MAC_STATE       : out   std_logic_vector (1 downto 0);
         stop_n          : out   std_logic;
         as_n            : out   std_logic;
         DO              : out   std_logic_vector (31 downto 0);
         IR              : out   std_logic_vector (31 downto 0);
         GPRwe           : out   std_logic;
         busy            : out   std_logic;
         MR              : out   std_logic;
         MW              : out   std_logic
      );
   end component;

   component fim_safecore
      port (
         clk             : in    std_logic;
         rst_n           : in    std_logic;
         addr            : in    std_logic_vector (9 downto 0);
         data_in         : in    std_logic_vector (31 downto 0);
         data_out        : out   std_logic_vector (31 downto 0);
         wr_in_n         : in    std_logic;
         card_sel        : in    std_logic;
         sack_n          : out   std_logic;
         inject_pc_fault : out   std_logic;
         inject_bit_flip : out   std_logic;
         inject_mem_flip : out   std_logic;
         bit_flip_mask   : out   std_logic_vector (31 downto 0)
      );
   end component;

   component wwdt_safecore
      port (
         clk        : in    std_logic;
         rst_n      : in    std_logic;
         addr       : in    std_logic_vector (9 downto 0);
         data_in    : in    std_logic_vector (31 downto 0);
         data_out   : out   std_logic_vector (31 downto 0);
         wr_in_n    : in    std_logic;
         card_sel   : in    std_logic;
         sack_n     : out   std_logic;
         irq_out    : out   std_logic;
         wd_reset_n : out   std_logic;
         dbg_ctrl   : out   std_logic_vector (31 downto 0);
         dbg_win_min : out  std_logic_vector (31 downto 0);
         dbg_win_max : out  std_logic_vector (31 downto 0);
         dbg_state  : out   std_logic_vector (31 downto 0)
      );
   end component;

   component secded_bus_wrapper
      port (
         di_raw          : in    std_logic_vector (31 downto 0);
         inject_bit_flip : in    std_logic;
         bit_flip_mask   : in    std_logic_vector (31 downto 0);
         di_corrected    : out   std_logic_vector (31 downto 0);
         err_single      : out   std_logic;
         err_double      : out   std_logic
      );
   end component;

begin

   ---------------------------------------------------------------------------
   -- Combinational glue
   ---------------------------------------------------------------------------

   -- SafeCores are active-low reset; RESA RESET is active-high.
   rst_n_sig <= not RESET;

   -- SECDED pipeline stage: register the raw read word and delay the memory
   -- acknowledge by one matching cycle. ACK_N is active-low, so it resets to '1'
   -- (deasserted). This keeps the DLX memory handshake aligned with the
   -- corrected data while keeping each combinational hop within the 60 MHz clock.
   -- OPTION A: full wait-state SECDED pipeline.
   -- Capture DO into DO_r on the cycle the memory asserts ACK_N (data valid).
   -- encode->decode runs on the REGISTERED DO_r during the next cycle, giving the
   -- SECDED logic a full clock to settle (combinational was too slow -> no-answer).
   -- ACK_N_d = ACK_N delayed one cycle releases the DLX exactly that next cycle,
   -- so it stays in FETCH/LOAD one extra cycle and latches the now-valid
   -- DI_corrected. Capture-cycle and release-cycle are paired (the half-pipeline
   -- that registered DO_r every cycle but released independently mis-aligned).
   secded_pipeline : process (CLK)
   begin
      if rising_edge(CLK) then
         if RESET = '1' then
            DO_r    <= (others => '0');
            ACK_N_d <= '1';
         else
            if ACK_N = '0' then
               DO_r <= DO;        -- capture read word on its valid cycle, then hold
            end if;
            ACK_N_d <= ACK_N;     -- release DLX one cycle later (after decode settles)
         end if;
      end if;
   end process;

   -- SECDED status sticky latch. err_single/err_double are single-cycle pulses
   -- from the decoder; latch them so a later read observes them. Cleared when the
   -- program writes the FIM disarm (fim_ctrl=0) -- reuse inject_mem_flip going low
   -- as the natural clear is unreliable, so clear on RESET only and let the demo
   -- read before re-arming. (Simple, robust: sticky until reset.)
   secded_status_latch : process (CLK)
   begin
      if rising_edge(CLK) then
         if RESET = '1' then
            secded_seen_single <= '0';
            secded_seen_double <= '0';
         else
            if secded_err_single = '1' then
               secded_seen_single <= '1';
            end if;
            if secded_err_double = '1' then
               secded_seen_double <= '1';
            end if;
         end if;
      end if;
   end process secded_status_latch;

   -- Status word: tag 0x5EC0 | live flags | sticky flags. Distinct tag so the
   -- demo can confirm this build is live and read both seen_* bits.
   secded_status <= x"5EC0" & "000000000000" & secded_seen_double & secded_seen_single
                    & secded_err_double & secded_err_single;

   -- DLX is idle in INIT or HALT.
   IN_INIT <= XLXN_48 or XLXN_63;

   -- DEBUG read-back (pending latch). Detect a 0x7Fx read in its ADDRESS phase,
   -- latch the offset, and hold the override until the DLX samples it (ACK_N_d,
   -- which is when the SECDED-delayed ack reaches the DLX). The override must
   -- persist through that sample cycle for the readback to be captured correctly.
   rd_hit <= '1' when (MAO(24) = '0' and IN_INIT = '0' and AS_N = '0'
                       and WR_OUT_N = '1' and MAO(11 downto 4) = x"7F") else '0';

   dbg_pending : process (CLK) is
   begin
      if rising_edge(CLK) then
         if rd_hit = '1' then
            rd_pending <= '1';
            rd_off     <= MAO(3 downto 0);
         elsif ACK_N_d = '0' then        -- read sampled by DLX -> drop override
            rd_pending <= '0';
         end if;
      end if;
   end process dbg_pending;

   dbg_value  <= dbg_win_max  when rd_off = x"C" else
                 secded_status when rd_off = x"0" else   -- 0x7F0: SECDED status (5EC0|flags)
                 dbg_win_min  when rd_off = x"8" else
                 dbg_state    when rd_off = x"4" else
                 x"DEADBEEF";

   -- SECDED source selection (constant-folded by SECDED_ENABLE), with the debug
   -- read-back overriding while a 0x7Fx read is pending.
   dlx_di_src  <= dbg_value when rd_pending = '1' else
                  DI_corrected when SECDED_ENABLE else DO;
   dlx_ack_src <= ACK_N_d      when SECDED_ENABLE else ACK_N;

   -- Combined reset feeding the DLX core: system RESET OR watchdog timeout.
   RESET_combined <= RESET or (not wd_reset_n);

   -- SafeCore selection via SDRAM-WRITE SNOOP.
   --
   -- The lab IO_LOGIC only asserts its slave-select (Ssel_o) for HOST-initiated
   -- accesses; a DLX access during a run can only reach SDRAM (verified: basic_test
   -- SDRAM reads/writes work, slave writes never land). So instead of fighting that,
   -- the SafeCores SNOOP the DLX's SDRAM write bus -- the path that provably works.
   --
   -- The DLX configures a SafeCore by writing to reserved SDRAM addresses; the
   -- wrapper watches MAO/MDO/WR_OUT_N and drives the SafeCore's card_sel + addr +
   -- data when one of those addresses is written. Reserved pages (clear of small
   -- programs):  WWDT = 0x7Fx, FIM = 0x7Ex. The low nibble of MAO is the module's
   -- own register offset, so MAO(9:0) feeds the module addr decode directly:
   --   WWDT: 0x7F0=ctrl(arm) 0x7F4=kick 0x7F8=win_min 0x7FC=win_max
   --   FIM : 0x7E0=ctrl      0x7E4=mask 0x7E8=delay   0x7EC=duration
   --
   -- TWO-PHASE SNOOP CAPTURE.
   --
   -- This bus never presents address and write-data on the same cycle:
   --   * MAO = 0x7Fx is valid only during the AS_N request phase (T0).
   --   * The write data on MDO is committed one cycle later, at as_n_mem (T1) --
   --     the point where the IO logic latches it. By T1 the DLX has released
   --     MAO. So every single-instant capture failed (raw/+1/trailing/ACK).
   -- Fix: decode the register OFFSET at T0 (latch MAO(3:0) while MAO is valid),
   -- then drive a one-cycle write pulse at T1 and let the SafeCore capture the
   -- now-valid MDO using the LATCHED offset (not the live, already-moved MAO).
   wwdt_wr_hit <= '1' when (MAO(24) = '0' and IN_INIT = '0' and AS_N = '0'
                            and WR_OUT_N = '0' and MAO(11 downto 4) = x"7F") else '0';
   fim_wr_hit  <= '1' when (MAO(24) = '0' and IN_INIT = '0' and AS_N = '0'
                            and WR_OUT_N = '0' and MAO(11 downto 4) = x"7E") else '0';

   snoop_capture : process (CLK) is
   begin
      if rising_edge(CLK) then
         wwdt_wr_hit_d <= wwdt_wr_hit;
         fim_wr_hit_d  <= fim_wr_hit;
         -- latch the register offset on the FIRST address-phase cycle (T0)
         if (wwdt_wr_hit = '1' and wwdt_wr_hit_d = '0') then
            snoop_off <= MAO(3 downto 0);
         elsif (fim_wr_hit = '1' and fim_wr_hit_d = '0') then
            snoop_off <= MAO(3 downto 0);
         end if;
         -- one-cycle write pulse, fired at T1 (cycle after the addr phase),
         -- where MDO is valid. card_sel pulse + latched offset + live MDO.
         wwdt_wr <= wwdt_wr_hit and (not wwdt_wr_hit_d);
         fim_wr  <= fim_wr_hit  and (not fim_wr_hit_d);
      end if;
   end process snoop_capture;

   -- monitor tap keeps showing a WWDT write event (now the T1 pulse)
   wwdt_card_sel <= wwdt_wr;
   snoop_addr    <= "000000" & snoop_off;


   -- Monitor keeps the PRIMARY slave handshake untouched: its SACK_N goes straight
   -- to IO_LOGIC. The snooping SafeCores need no slave ack, so the secondary path
   -- (RACK_N / R_DO) stays inert and SACK_N_i is the monitor's line, unchanged.
   RACK_N <= '1';
   R_DO   <= (others => '0');

   ---------------------------------------------------------------------------
   -- IO_LOGIC_U (bus master + SDRAM controller)
   ---------------------------------------------------------------------------
   XLXI_23 : IO_LOGIC_U
      port map (
         AS_N_i                 => AS_N,
         fpgaClk_i              => fpgaClk_i,
         in_init_i              => IN_INIT,
         MA_i  (31 downto 0)    => MAO (31 downto 0),
         MD_i  (31 downto 0)    => MDO (31 downto 0),         -- DLX write data straight to SDRAM
         RACK_N_i               => RACK_N,
         RD_i  (31 downto 0)    => R_DO (31 downto 0),
         SACK_N_i               => SACK_N,
         sdClkFb_i              => sdClkFb_i,
         SD_i  (31 downto 0)    => SDO (31 downto 0),
         WR_N_i                 => WR_OUT_N,
         A_o   (9 downto 0)     => AI  (9 downto 0),
         CLK                    => CLK,
         DO    (31 downto 0)    => DO (31 downto 0),
         MACK_N_o               => ACK_N,
         RESET                  => RESET,
         Rsel_o                 => Rsel,
         sdAddr_o(12 downto 0)  => sdAddr_o(12 downto 0),
         sdBs_o  (1 downto 0)   => sdBs_o  (1 downto 0),
         sdCas_bo               => sdCas_bo,
         sdCe_bo                => sdCe_bo,
         sdCke_o                => sdCke_o,
         sdClk_o                => sdClk_o,
         sdDqmh_o               => sdDqmh_o,
         sdDqml_o               => sdDqml_o,
         sdRas_bo               => sdRas_bo,
         sdWe_bo                => sdWe_bo,
         Ssel_o                 => Card_Sel,
         step_en_o              => STEP_EN,
         WR_N_o                 => WR_IN_N,
         sdData_io(15 downto 0) => sdData_io(15 downto 0)
      );

   ---------------------------------------------------------------------------
   -- SECDED wrapper (between SDRAM read data and DLX DI)
   ---------------------------------------------------------------------------
   XLXI_SECDED : secded_bus_wrapper
      port map (
         di_raw        (31 downto 0) => DO_r         (31 downto 0),
         inject_bit_flip             => inject_mem_flip,
         bit_flip_mask (31 downto 0) => bit_flip_mask(31 downto 0),
         di_corrected  (31 downto 0) => DI_corrected (31 downto 0),
         err_single                  => secded_err_single,
         err_double                  => secded_err_double
      );

   ---------------------------------------------------------------------------
   -- FIM SafeCore
   ---------------------------------------------------------------------------
   XLXI_FIM : fim_safecore
      port map (
         clk                         => CLK,
         rst_n                       => rst_n_sig,
         addr          (9 downto 0)  => snoop_addr   (9 downto 0),
         data_in       (31 downto 0) => MDO          (31 downto 0),
         data_out      (31 downto 0) => fim_data_out (31 downto 0),
         wr_in_n                     => '0',
         card_sel                    => fim_wr,
         sack_n                      => fim_sack_n,
         inject_pc_fault             => inject_pc_fault,
         inject_bit_flip             => inject_bit_flip,
         inject_mem_flip             => inject_mem_flip,
         bit_flip_mask (31 downto 0) => bit_flip_mask(31 downto 0)
      );

   ---------------------------------------------------------------------------
   -- WWDT SafeCore
   ---------------------------------------------------------------------------
   XLXI_WWDT : wwdt_safecore
      port map (
         clk                         => CLK,
         rst_n                       => rst_n_sig,
         addr          (9 downto 0)  => snoop_addr    (9 downto 0),
         data_in       (31 downto 0) => MDO           (31 downto 0),
         data_out      (31 downto 0) => wwdt_data_out (31 downto 0),
         wr_in_n                     => '0',
         card_sel                    => wwdt_wr,
         sack_n                      => wwdt_sack_n,
         irq_out                     => wwdt_irq,
         wd_reset_n                  => wd_reset_n,
         dbg_ctrl                    => dbg_ctrl,
         dbg_win_min                 => dbg_win_min,
         dbg_win_max                 => dbg_win_max,
         dbg_state                   => dbg_state
      );

   ---------------------------------------------------------------------------
   -- Monitor slave (untouched)
   ---------------------------------------------------------------------------
   XLXI_34 : monitor_slave_MUSER_Top_Level
      port map (
         AI     (9 downto 0)  => AI     (9 downto 0),
         A_IN   (31 downto 0) => D      (31 downto 0),
         B_IN   (31 downto 0) => MAO    (31 downto 0),
         Cardsel              => Card_Sel,
         clk                  => CLK,
         D_IN   (31 downto 0) => monitored_signals(31 downto 0),
         IN_INIT              => IN_INIT,
         STEP_EN              => STEP_EN,
         STOP_N               => STOP_N,
         WR_N                 => WR_IN_N,
         reg_adr(4 downto 0)  => REG_ADR(4 downto 0),
         SACK_N               => SACK_N,
         SDO    (31 downto 0) => SDO    (31 downto 0)
      );

   ---------------------------------------------------------------------------
   -- DLX processor
   --   - DI receives SECDED-corrected data
   --   - RESET <- RESET_combined (so WWDT timeout resets PC + datapath)
   ---------------------------------------------------------------------------
   XLXI_70 : DLX_MUSER_Top_Level
      port map (
         CLK                         => CLK,
         RESET                       => RESET_combined,  -- TRUE RESET: system RESET OR latched WWDT timeout
         ACK_N                       => dlx_ack_src,     -- ACK_N (baseline) or ACK_N_d (SECDED enabled)
         Dadr          (4 downto 0)  => REG_ADR     (4 downto 0),
         DI            (31 downto 0) => dlx_di_src   (31 downto 0),  -- DO (baseline) or DI_corrected (SECDED enabled)
         step_en                     => STEP_EN,
         inject_pc_fault             => inject_pc_fault,
         inject_bit_flip             => inject_bit_flip,
         bit_flip_mask (31 downto 0) => bit_flip_mask(31 downto 0),
         wd_reset_n                  => wd_reset_n,
         Ace                         => XLXN_37,
         add                         => XLXN_41,
         AEQZ                        => AEQZ,
         ALUF          (2 downto 0)  => ALUD         (2 downto 0),
         Asel                        => XLXN_28,
         as_n                        => AS_N,
         Bce                         => XLXN_36,
         busy                        => busy,
         Cce                         => XLXN_35,
         D             (31 downto 0) => D            (31 downto 0),
         DINce                       => DINsel,
         DLX_STATE     (4 downto 0)  => DLX_STATE    (4 downto 0),
         DO            (31 downto 0) => MDO          (31 downto 0),
         func                        => open,
         GPRwe                       => XLXN_27,
         halt                        => XLXN_48,
         init                        => XLXN_63,
         IR            (31 downto 0) => ir           (31 downto 0),
         IRce                        => XLXN_29,
         Itype                       => XLXN_45,
         jlink                       => XLXN_46,
         MAC_STATE     (1 downto 0)  => MAC_STATE    (1 downto 0),
         MARce                       => XLXN_31,
         MDRce                       => XLXN_33,
         MDRsel                      => MDRsel,
         MMU_AO        (31 downto 0) => MAO          (31 downto 0),
         MR                          => XLXN_25,
         MW                          => XLXN_24,
         opcode                      => open,
         PCce                        => XLXN_34,
         right                       => XLXN_38,
         shift                       => XLXN_39,
         stop_n                      => STOP_N,
         S1SEL         (1 downto 0)  => S1SEL        (1 downto 0),
         S2SEL         (1 downto 0)  => S2SEL        (1 downto 0),
         test                        => XLXN_40,
         wr_n                        => WR_OUT_N
      );

   ---------------------------------------------------------------------------
   -- Monitor signals (Logic Analyzer inputs).
   -- Bits 21, 23, 27 hold SafeCore signals; otherwise same as original.
   ---------------------------------------------------------------------------
   -- ============================================================== BUS PROBE
   -- LA bits remapped to watch the raw write bus and the snoop decode directly.
   -- Run the bus_probe loop (repeated  sw 0xAB -> 0x7FC) and capture a waveform.
   mao_is_7fx <= '1' when (MAO(11 downto 4) = x"7F") else '0';
   mdo_is_AB  <= '1' when (MDO = x"000000AB")        else '0';

   monitored_signals(0)  <= AS_N;          -- request strobe (active low)
   monitored_signals(1)  <= WR_OUT_N;      -- write (low) / read (high)
   monitored_signals(2)  <= ACK_N;         -- memory acknowledge (active low)
   monitored_signals(3)  <= IN_INIT;
   monitored_signals(4)  <= mao_is_7fx;    -- TRUE on cycles MAO is in the 0x7Fx page
   monitored_signals(5)  <= mdo_is_AB;     -- TRUE on cycles MDO holds 0x000000AB
   monitored_signals(6)  <= wwdt_wr_hit;   -- my T0 write decode (MAO=0x7Fx & AS_N & !WR)
   monitored_signals(7)  <= wwdt_wr;       -- my T1 capture pulse
   monitored_signals(8)  <= MAO(0);
   monitored_signals(9)  <= MAO(1);
   monitored_signals(10) <= MAO(2);
   monitored_signals(11) <= MAO(3);
   monitored_signals(12) <= MAO(4);
   monitored_signals(13) <= MAO(5);
   monitored_signals(14) <= MAO(6);
   monitored_signals(15) <= MAO(7);
   monitored_signals(16) <= MAO(8);
   monitored_signals(17) <= MAO(9);
   monitored_signals(18) <= MAO(10);
   monitored_signals(19) <= MAO(11);
   monitored_signals(20) <= MDO(0);
   monitored_signals(21) <= MDO(1);
   monitored_signals(22) <= MDO(2);
   monitored_signals(23) <= MDO(3);
   monitored_signals(24) <= MDO(4);
   monitored_signals(25) <= MDO(5);
   monitored_signals(26) <= MDO(6);
   monitored_signals(27) <= MDO(7);
   -- DOWNLOAD SIGNATURE: constant 1,0,1,0 on bits 28..31.
   -- Under the OLD .lbl these are dlx_st1..dlx_st4 -> they read 1,0,1,0 FLAT
   -- across every sample only when THIS bitstream is actually on the chip.
   monitored_signals(28) <= '1';
   monitored_signals(29) <= '0';
   monitored_signals(30) <= '1';
   monitored_signals(31) <= '0';

end BEHAVIORAL;
