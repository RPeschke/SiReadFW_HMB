---------------------------------------------------------------------------------
-- Title         : Command Interpreter
-- Project       : General Purpose Core
---------------------------------------------------------------------------------
-- File          : CommandInterpreter.vhd
-- Author        : Kurtis Nishimura, updated by Nathan Park (park.nathan@gmail.com)
---------------------------------------------------------------------------------
-- Description:
-- Packet parser for old Belle II format.
-- See: http://www.phys.hawaii.edu/~kurtisn/doku.php?id=itop:documentation:data_format
---------------------------------------------------------------------------------
 
library ieee;
    use ieee.std_logic_1164.all;
    use ieee.std_logic_arith.all;
    use ieee.std_logic_unsigned.all;
    use work.all;
    use work.UtilityPkg.all;
    use work.Eth1000BaseXPkg.all;
    use work.GigabitEthPkg.all;
    use work.BMD_definitions.all; --need to include BMD_definitions in addition to work.all

    use work.axi_stream_s32.all;

entity CommandInterpreter is
    generic (
        REG_ADDR_BITS_G : integer := 16;
        REG_DATA_BITS_G : integer := 16;
        TIMEOUT_G       : integer := 125000;
        TIMEOUT_G1       : integer := 536870912;
        GATE_DELAY_G    : time := 1 ns;
        num_DC          : integer := 0; -- 3
        packets         : integer := 8;
        data_in_packet        : integer := 8;
        chls       : integer := 1
    );
    port (
        -- User clock and reset
        usrClk      : in  sl;
        dataClk	   : in  sl;
        usrRst      : in  sl;
        -- Incoming data from PC
        rxData      : in  slv(31 downto 0);
        rxDataValid : in  sl;
        rxDataLast  : in  sl;
        rxDataReady : out sl;
        -- Outgoing response to PC
        txData      : out slv(31 downto 0);
        txDataValid : out sl;
        txDataLast  : out sl;
        txDataReady : in  sl;
        --DC Comm signals
        serialClkLck : in slv(num_DC downto 0);
        trigLinkSync : in slv(num_DC downto 0);
        DC_CMD 		 : out slv(31 downto 0) := (others => '0');
        QB_WrEn      : out slv(num_DC downto 0);
        QB_RdEn      : out slv(num_DC downto 0);
        DC_RESP		 : in slv(31 downto 0);
        DC_RESP_VALID: in slv(num_DC downto 0);
        EVNT_FLAG    : in sl;
        -- Register interfaces
        regAddr     : out slv(REG_ADDR_BITS_G-1 downto 0);
        regWrData   : out slv(REG_DATA_BITS_G-1 downto 0);
        regRdData   : in  slv(REG_DATA_BITS_G-1 downto 0);
        regReq      : out sl;
        regOp       : out sl;
        regAck      : in  sl;
        --debug ports
        ldQBLink 	: out sl;
        cmd_int_state : out slv(4 downto 0)
    );
end CommandInterpreter;

-- Define architecture
architecture rtl of CommandInterpreter is

    type StateType     is (IDLE_S,PACKET_SIZE_S,PACKET_TYPE_S, PING_S,PING_RESPONSE_S,
                           COMMAND_TARGET_S,COMMAND_ID_S,COMMAND_TYPE_S,
                           COMMAND_DATA_S,COMMAND_CHECKSUM_S,READ_S, READ_DATA, WRITE_S,
                           READ_RESPONSE_S,WRITE_RESPONSE_S,PACKET_CHECKSUM_S, SENDTRIG_S); 


    type StateType_2     is (IDLE_S, header_s, PING_S,  READ_S , READ_RESPONSE_S, WRITE_S ,WRITE_RESPONSE_S  );                            
    -- , PING_S,PING_RESPONSE_S, ERR_RESPONSE_S, CHECK_MORE_S,DUMP_S


    type RegType is record
        state       : StateType;
        regAddr     : slv(REG_ADDR_BITS_G-1 downto 0);
        regWrData   : slv(REG_DATA_BITS_G-1 downto 0);
        regRdData   : slv(REG_DATA_BITS_G-1 downto 0);
        regReq      : sl;
        regOp       : sl;
        sendResp    : sl;
        rxDataReady : sl;
        txData      : slv(31 downto 0);
        txDataValid : sl;
        txDataLast  : sl;
        wordsLeft   : slv(31 downto 0);
        wordOutCnt  : slv(7 downto 0);
        checksum    : slv(31 downto 0);
        deviceID 	: slv(31 downto 0);
        commandType : slv(31 downto 0);
        command     : slv(31 downto 0);
        commandId   : slv(23 downto 0);
        noResponse  : sl;
        -- errFlags    : slv(31 downto 0); --Mudit
        timeoutCnt  : slv(31 downto 0);
        timeoutCnt1  : slv(31 downto 0); --to count wait between two data samples
        dataCount  : integer  range 0 to 21; --Mudit
        packetCount : integer  range 0 to 9; --Mudit
    end record RegType;


    constant REG_INIT_C : RegType := (
        state       => IDLE_S,
        regAddr     => (others => '0'),
        regWrData   => (others => '0'),
        regRdData   => (others => '0'),
        regReq      => '0',
        regOp       => '0',
        sendResp    => '0',
        rxDataReady => '0',
        txData      => (others => '0'),
        txDataValid => '0',
        txDataLast  => '0',
        wordsLeft   => (others => '0'),
        wordOutCnt  => (others => '0'),
        checksum    => (others => '0'),
        deviceID    => (others => '0'),
        commandType => (others => '0'),
        command     => (others => '0'),
        commandId   => (others => '0'),
        noResponse  => '0',
        -- errFlags    => (others => '0'), --Mudit
        timeoutCnt  => (others => '0'),
        timeoutCnt1  => (others => '0'),
        dataCount  => 0,
        packetCount  => 0
    );

    signal r   : RegType := REG_INIT_C;
    -- signal t   : RegType := REG_INIT_C;
    signal rin : RegType;
    -- signal tin : RegType;
    signal loadQB : sl := '0';
    signal data_flag : sl := '0';
    signal QB_loadReg : Word32Array(2 downto 0);
    signal DC_cmdRespReq : slv(num_DC downto 0);
    signal start_load : sl := '0';
    signal start_load1 : sl := '0';
    signal DC_RESP_VALID_data : slv(num_DC downto 0);
    signal DC_RESP_data		: slv(31 downto 0);
    -- ISE attributes to keep signals for debugging
    -- attribute keep : string;
    -- attribute keep of r : signal is "true";
    -- attribute keep of crcOut : signal is "true";

    -- Vivado attributes to keep signals for debugging
    -- attribute dont_touch : string;
    -- attribute dont_touch of r : signal is "true";
    -- attribute dont_touch of crcOut : signal is "true";

    constant WORD_HEADER_C    : slv(31 downto 0) := x"00BE11E2";
    constant WORD_COMMAND_C   : slv(31 downto 0) := x"646F6974";
    constant WORD_PING_C      : slv(31 downto 0) := x"70696E67";
    constant WORD_READ_C      : slv(31 downto 0) := x"72656164";
    constant WORD_WRITE_C     : slv(31 downto 0) := x"72697465";
    constant WORD_WRITE_DAC     : slv(31 downto 0) := x"72697445";
    constant WORD_ACK_C       : slv(31 downto 0) := x"6F6B6179";
    constant WORD_READ_DATA      : slv(31 downto 0) := x"72652124";
    -- constant WORD_ERR_C       : slv(31 downto 0) := x"7768613f";

    -- constant ERR_BIT_SIZE_C    : slv(31 downto 0) := x"00000001";
    -- constant ERR_BIT_TYPE_C    : slv(31 downto 0) := x"00000002";
    -- constant ERR_BIT_DEST_C    : slv(31 downto 0) := x"00000004";
    -- constant ERR_BIT_COMM_TY_C : slv(31 downto 0) := x"00000008";
    -- constant ERR_BIT_COMM_CS_C : slv(31 downto 0) := x"00000010";
    -- constant ERR_BIT_CS_C      : slv(31 downto 0) := x"00000020";
    -- constant ERR_BIT_TIMEOUT_C : slv(31 downto 0) := x"00000040";
    -- constant QBLINK_FAILURE_C  : slv(31 downto 0) := x"00000500"; --link not up yet error

    constant wordDC				: slv(23 downto 0) := x"0000DC"; --command target is one or more DC
    constant broadcastDC       : slv(7 downto 0)  := x"0A"; --command target is all DCs
    signal wordScrodRevC      : slv(31 downto 0)  := (others=> '0');

    signal stateNum : slv(4 downto 0);
    signal dc_id : integer := 0;
    -- added signal to monitor wordsleft 15 oct 2020: Shivang
--	signal wordsleft_i  : std_logic_vector(31 downto 0) := (others=> '0');
    signal dc_ack : sl := '0';
    signal dc_ack1 : sl := '0';
    -- attribute keep : string;
    -- attribute keep of stateNum : signal is "true";
    signal s_axis_tready : sl := '1';
    signal tready_fifo : sl :=  '0';
    signal tdata_fifo : slv(31 downto 0) := (others => '0');
    signal tvalid_fifo : sl :=  '0';
    -- signal txData_i : slv(31 downto 0) := (others => '0');
    -- signal txDataValid_i : sl :=  '0';
    -- signal txDataReady_i : sl :=  '0';

    attribute mark_debug : string;
    attribute mark_debug of loadQB : signal is "true";
    attribute mark_debug of stateNum : signal is "true";
--	attribute mark_debug of wordsleft_i : signal is "true";
------------------------------------------------------------------------------------------------------------------------
    signal rx : axi_stream_32_slave := axi_stream_32_slave_null;
    signal dc_rx : axi_stream_32_slave := axi_stream_32_slave_null;
    signal tx : axi_stream_32_master := axi_stream_32_master_null;

    type header_t is record 
        packet_size:  slv(31 downto 0);
        packet_type: slv(31 downto 0);
        command_target: slv(31 downto 0);
        command_id: slv(23 downto 0);
        no_response : sl;
        command_type: slv(31 downto 0);
        command_data: slv(31 downto 0);
        dc_id : integer;
        loadQB : std_logic;
    end record;

    constant header_t_null :  header_t := (
        packet_size => (others => '0'),
        packet_type => (others => '0'),
        command_target => (others => '0'),
        command_id => (others => '0'),
        no_response  => '0',
        command_type => (others => '0'),
        command_data => (others => '0'),
        dc_id => 0,
        loadQB => '0'
    );



    pure function get_dc(command_target : std_logic_vector) return integer is
        variable ret : integer := 0;
    begin
        if (command_target(31 downto 8) = wordDC) and (command_target(7 downto 0) /= broadcastDC) then 
            ret := conv_integer(unsigned(command_target(7 downto 0)));
        elsif (command_target(7 downto 0) = broadcastDC) then
            ret := conv_integer(unsigned(broadcastDC));
        end if;
        return ret;

    end function;

    
    pure function get_loadQB(command_target : std_logic_vector) return sl is
    begin
        if command_target = wordScrodRevC    then 
            return '0';
        elsif (command_target(31 downto 8) = wordDC) and (command_target(7 downto 0) /= broadcastDC) then 
            return '1';
        elsif (command_target(7 downto 0) = broadcastDC) then
            return '1';
        end if;
        return  '0';
    end function;

    procedure fill_header(signal self : inout header_t ; index : in integer ; data: in std_logic_vector )  is 
        constant packet_size_index: integer := 0;
        constant packet_type_index: integer := 1;
        constant command_target_index: integer := 2;
        constant command_id_index: integer := 3;
        constant command_type_index: integer := 4;
        constant command_data_index: integer := 5;
    begin 
        if index = packet_size_index then 
            self.packet_size <= data(self.packet_size'range);
        elsif index = packet_type_index then 
            self.packet_type <= data(self.packet_type'range);
        elsif index = command_target_index then 
            self.command_target <= data(self.command_target'range);
            self.dc_id  <= get_dc(data);
            self.loadQB <= get_loadQB(data);
        elsif index = command_id_index then 
            self.command_id  <= data(self.command_id'range);
            self.no_response <= data (31);
        elsif index = command_type_index then 
            self.command_type <= data(self.command_type'range);
        elsif index = command_data_index then 
            self.command_data <= data(self.command_data'range);
        end if;



    end procedure;

    pure function get_header_slice(self : header_t ; index : in integer ) return std_logic_vector is
       variable  ret : slv(31 downto 0) := (others => '0');
    begin
        
         if index = 0 then 
            ret := WORD_HEADER_C;
        elsif index = 1 then 
            ret :=  x"00000006";
            if self.command_type = WORD_PING_C then 
                ret :=  x"00000005";
            end if;

        elsif index = 2 then 
            ret := WORD_ACK_C;
        elsif index = 3 then 
            ret := self.command_target;
        elsif index = 4 then 
            ret :=  x"00" & self.command_id;
        elsif index = 5 then 
            ret := self.command_type ; 
        end if;

        return  ret;
    end function;



    signal i_header : header_t := header_t_null;
    signal i_header_counter : integer := 0;

begin
    cmd_int_state <= stateNum;
    ldQBLink <= loadQB;
    DC_RESP_data <= DC_RESP when data_flag = '1' else (others =>'0');
    DC_RESP_VALID_data <= DC_RESP_VALID when data_flag = '1' else  "0";
    -- txData <= txData_i;
    -- txDataValid <= txDataValid_i;
    -- txDataReady_i <= txDataReady;
--	wordsleft_i <= r.wordsLeft;
    
    stateNum <= "00000" when r.state = IDLE_S else             -- 0 x00
                "00001" when r.state = PACKET_SIZE_S else      -- 1 x01
                "00010" when r.state = PACKET_TYPE_S else      -- 2 x02
                "00011" when r.state = COMMAND_TARGET_S else   -- 3 x03
                "00100" when r.state = COMMAND_ID_S else       -- 4 x04
                "00101" when r.state = COMMAND_TYPE_S else     -- 5 x05
                "00110" when r.state = COMMAND_DATA_S else     -- 6 x06
                "00111" when r.state = COMMAND_CHECKSUM_S else -- 7 x07
                "01000" when r.state = PING_S else             -- 8 x08
                "01001" when r.state = READ_S else             -- 9 x09
                "01010" when r.state = WRITE_S else            -- 10 x0A
                "01011" when r.state = READ_RESPONSE_S else    -- 11 x0B
                "01100" when r.state = WRITE_RESPONSE_S else   -- 12 x0C
                "01101" when r.state = PING_RESPONSE_S else
                "01110" when r.state = READ_DATA else    -- 13 x0D
                -- "01110" when r.state = ERR_RESPONSE_S else     -- 14 x0E
                -- "01111" when r.state = CHECK_MORE_S else       -- 15 x0F
                -- "10000" when r.state = PACKET_CHECKSUM_S else  -- 16 x10
                -- "10001" when r.state = DUMP_S else             -- 17 x11
                -- "10010" when r.state = IDLE_S else             -- 18 x12
                "11111";                                       -- 19 x1F


    wordScrodRevC(31 downto 0) <= x"0000A500";

--    data_fifos_generation : for i in 0 to chls-1 generate
--       data_fifos: entity work.serial_data_window_fifo_w32d1024
--   PORT MAP (
--     m_aclk => DATA_CLK,
--     s_aclk => sys_clk,
--     s_aresetn => not sync1(0),
--     s_axis_tvalid => samples_valid,
--     s_axis_tready => fifo_s_tready(i),
--     s_axis_tdata => ZERO1 & sample_data(i),
--     m_axis_tvalid => qblink_tvalid_i(i),
--     m_axis_tready => qblink_tready_i(i),
--     m_axis_tdata => qblink_tdata_i(i)
--   );
--    end generate;
    --detect start rising edge, checked if a signal was on for one cycle previously
    -- process(usrClk, DC_RESP_VALID, DC_RESP_VALID_i)
    -- begin
    --     if rising_edge(usrClk) then
    --         DC_RESP_VALID_i <= DC_RESP_VALID_i(0) & DC_RESP_VALID(0);
            
    --     end if;
    -- end process;



    rx.m2s.data <= rxData;
    rx.m2s.valid <= rxDataValid;
    rx.m2s.last <= rxDataLast;
    rxDataReady <= rx.s2m.ready;

    tx.s2m.ready <= txDataReady;
    txData <= tx.m2s.data;
    txDataValid <= tx.m2s.valid;
    txDataLast <= tx.m2s.last;

    dc_rx.m2s.valid <= tvalid_fifo;
    dc_rx.m2s.data <= tdata_fifo;
    dc_rx.m2s.last <= '0';
    tready_fifo <= dc_rx.s2m.ready;

    process(usrClk) is 
        variable rx_buff : std_logic_vector(31 downto 0) := (others => '0');
        variable tx_buff : std_logic_vector(31 downto 0) := (others => '0');
        variable v_ready_to_send: std_logic := '0';
        variable v_state : StateType_2 := IDLE_S;
        variable v_wordsLeft : std_logic_vector(31 downto 0) := (others => '0');
        variable v_device_id : std_logic_vector(31 downto 0) := (others => '0');
        variable v_commandId   : slv(23 downto 0);
        variable v_noResponse  : sl;

        variable   dataCount  : integer  range 0 to 21;
        variable  packetCount : integer  range 0 to 9; 
        variable v_timeoutCnt1  : slv(31 downto 0); 
        variable v_command  : slv(31 downto 0);
        variable v_regAddr  : slv(15 downto 0);
        variable v_regWrData  : slv(15 downto 0);
        variable v_wordOutCnt : std_logic_vector(7 downto 0);
        variable v_regRdData_buffer :slv(REG_DATA_BITS_G-1 downto 0);
    begin 
    if rising_edge(usrClk) then 
        pull(rx);
        pull(dc_rx);
        pull(tx);
        regReq  <= '0';
        DC_cmdRespReq <= (others =>'0');
        tx_buff := (others =>'0');
        v_ready_to_send := '0';
        if i_header.command_type = WORD_PING_C then 
            v_state := PING_S;
        end if;
        v_timeoutCnt1   := v_timeoutCnt1   +1;
        if v_timeoutCnt1 = TIMEOUT_G then 
            v_state    := IDLE_S; 
        end if;
                    

        case (v_state) is 
            when IDLE_S => 
                i_header_counter <= 0;
                i_header <= header_t_null;
                v_wordOutCnt := (others =>'0');
                v_regRdData_buffer:= (others =>'0');
                if isReceivingData(rx)  then 
                    v_timeoutCnt1 := 0;
                    read_data(rx, rx_buff);
                    if rx_buff = WORD_HEADER_C and not endOfStream(rx) then 
                        v_state := header_s;
                    end if;
                end if;
            when header_s => 

                if isReceivingData(rx)  then 
                    v_timeoutCnt1 := 0;
                    read_data(rx, rx_buff);
                    fill_header(i_header , i_header_counter ,rx_buff);
                    i_header_counter <=  i_header_counter+1;
                    if endOfStream(rx) then 
                        v_state := IDLE_S;
                    end if;
                   
                    if i_header_counter = 5 then 
                        regAddr  <= rx_buff(15 downto 0);
                        v_regAddr := rx_buff(15 downto 0);
                        regWrData <= rx_buff(31 downto 16);

                        if i_header.command_type  = WORD_READ_C then 
                            v_state := READ_S;
                        elsif i_header.command_type  = WORD_WRITE_C then                             
                            v_state := WRITE_S;
                        end if;

                    end if;

                end if;

            when PING_S => 
                
                 if i_header.no_response  = '1' then
                    v_state := IDLE_S;
                else
                    if i_header.loadQB = '1' then
                        if i_header.dc_id /= broadcastDC then
                            if serialClkLck(i_header.dc_id-1) = '1' and trigLinkSync(i_header.dc_id-1) = '1' then --check if QBLink is up (hardcoded)
                                v_state    := READ_RESPONSE_S;
                            else
                                v_state := IDLE_S;
                            end if;
                        end if;
                    else
                        v_state    := READ_RESPONSE_S;
                    end if;
                end if;




            when READ_S =>

                
                if i_header.loadQB = '1' then -- if reading DC, listen to QBLink
                    if i_header.dc_id /= broadcastDC then --IF not broadcasting to all DCs
                        DC_cmdRespReq(i_header.dc_id-1) <= '1';
                        if DC_RESP_VALID(i_header.dc_id-1) = '1' then --wait for DC to send register data
                            --do not use v.regRdData to collect readback data
                            if i_header.no_response = '1' then --if noResponse setting on, skip response to PC
                                v_state := IDLE_S; --CHECK_MORE_S, Mudit
                            else --if noResponse setting off, send Register data in response to PC
                                v_state    := READ_RESPONSE_S;
                            end if;
                        end if;

                    end if; 
                else  --if reading SCROD register:
                    regOp      <= '0'; -- set Registers to read mode
                    regReq     <= '1'; --request operation
                    if (regAck = '1') then
                        v_regRdData_buffer := regRdData;
                        v_state    := READ_RESPONSE_S;
                    end if;
                end if;

            when READ_RESPONSE_S =>
                
                if i_header.loadQB ='0' and regAck = '0'   then
                    tx_buff :=  v_regRdData_buffer& v_regAddr;
                    v_ready_to_send := '1';
                elsif i_header.loadQB  = '1' and i_header.dc_id /= broadcastDC  then
                    tx_buff :=  DC_RESP;
                    v_ready_to_send := '1';
                elsif i_header.command_type = WORD_PING_C then 
                    v_ready_to_send := '1';
                    tx_buff :=  (others =>'0');
                end if;

                if  ready_to_send(tx) and v_ready_to_send ='1' then 
                    v_timeoutCnt1 := 0;
                    send_data(tx, get_header_slice(i_header, v_wordOutCnt));
                    if v_wordOutCnt = 5 then 
                        send_data(tx, tx_buff );
                    elsif v_wordOutCnt >= 6 then 
                        Send_end_Of_Stream(tx);
                        v_state      := IDLE_S; --CHECK_MORE_S, Mudit
                    end if;
                    v_wordOutCnt := v_wordOutCnt + 1;
                end if;




            when WRITE_S => --TEMP allow cmd interpreter to write to both SCROD and DC registers simulataneously to test dual functionality
                
                if (loadQB = '1') and (dc_id /= broadcastDC) then --if writing to DC: wait for them to repeat correct address
                    DC_cmdRespReq(dc_id-1) <= '1';
                    if DC_RESP(15 downto 0) = r.regAddr then --write operation is successful if DC repeats address, even if simultaneous SCROD register operation fails.
                        if r.noResponse = '1' then --skip response to PC if noResponse setting is on
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then --if the DC does not repeat register before timeout, error is raised (even if SCROD register is written successfully).
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;

                elsif (loadQB = '1') and (dc_id = broadcastDC) then
                    DC_cmdRespReq(num_DC downto 0) <= (others => '1');
                    if (DC_RESP(15 downto 0) = r.regAddr) then
                        if r.noResponse = '1' then --skip response to PC if noResponse setting is on
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then --if the DC does not repeat register before timeout, error is raised (even if SCROD register is written successfully).
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                    -----------------------------
                else
                    v.regOp      := '1'; --enable write to SCROD Register
                    v.regReq     := '1'; --start writing
                    if (regAck = '1') then --if not reading DC, make sure SCROD register write was successful.
                        v.regReq    := '0';
                        if r.noResponse = '1' then
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                end if;
        end case;














        case(v_state) is
            when IDLE_S =>
                v_wordsLeft := ( others => '0');
                v_device_id := ( others => '0');
                v_commandId := ( others => '0');
                v_noResponse := '0';
                dataCount :=0;
                packetCount :=0;
                v_timeoutCnt1   := (others =>'0');
                v_command  := (others =>'0');
                v_regAddr  := (others =>'0');
                v_regWrData:= (others =>'0');
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff);
                    if rx_buff = WORD_HEADER_C and not endOfStream(rx) then 
                        v_state := PACKET_SIZE_S;
                    end if;
                end if;
            when PACKET_SIZE_S =>
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff);
                    if unsigned( rx_buff ) < 300  and not endOfStream(rx) then 
                        v_wordsLeft := rx_buff;
                        v_state := PACKET_TYPE_S;
                    else 
                        v_state := IDLE_S;
                    end if;
                end if;
            when PACKET_TYPE_S =>
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff);
                    if rx_buff = WORD_COMMAND_C  and not endOfStream(rx) then 
                        v_wordsLeft := rx_buff;
                        v_state := COMMAND_TARGET_S;
                    else 
                        v_state := IDLE_S;
                    end if;
                end if;
            when COMMAND_TARGET_S =>
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff );
                    v_device_id := rx_buff;
                    v_wordsLeft := v_wordsLeft - 1;
                    v_state := COMMAND_ID_S;
                    if  endOfStream(rx) then 
                        v_state := IDLE_S;
                    elsif rx_buff = wordScrodRevC    then 
                        loadQB <= '0';
                    elsif (rx_buff(31 downto 8) = wordDC) and (rx_buff(7 downto 0) /= broadcastDC) then 
                        dc_id <= conv_integer(unsigned(rx_buff(7 downto 0)));
                        loadQB <= '1';
                    elsif (rx_buff(7 downto 0) = broadcastDC) then
                        dc_id <= conv_integer(unsigned(broadcastDC)); --if broadcasting, set dc_id to "broadcast"
                        loadQB <= '1';
                    end if;
                end if;
            when COMMAND_ID_S =>
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff );
                    v_wordsLeft := v_wordsLeft - 1;
                    v_commandId  := rx_buff(23 downto 0);
                    v_noResponse := rx_buff(31);
                    v_state := COMMAND_TYPE_S;
                    if  endOfStream(rx) then 
                        v_state := IDLE_S;
                    end if;
                end if;

            when COMMAND_TYPE_S =>
                if isReceivingData(rx)  then 
                    read_data(rx, rx_buff );
                    if rx_buff = WORD_PING_C then
                        v_state := PING_S; --Mudit
                    elsif rx_buff = WORD_READ_C or rx_buff = WORD_WRITE_C or rx_buff = WORD_WRITE_DAC or rx_buff = WORD_READ_DATA then --added by me,  or rxData = WORD_WRITE_DAC
                        v_state := COMMAND_DATA_S;
                    end if;
                    if  endOfStream(rx) then 
                        v_state := IDLE_S;
                    end if;

                end if;

            when COMMAND_DATA_S =>
                if isReceivingData(rx)  then 
                    read_data(rx , rx_buff);
                    v_command := rx_buff;
                    v_regAddr   := rx_buff(15 downto 0);
                    v_regWrData := rx_buff(31 downto 16);
                    regAddr   <= v_regAddr;
                    regWrData <= v_regWrData;
                    v_state := COMMAND_CHECKSUM_S;
                    if  endOfStream(rx) then 
                        v_state := IDLE_S;
                    end if;

                end if;

            when READ_DATA =>
 
                v_timeoutCnt1   := v_timeoutCnt1   +1;
                    
                if ready_to_send(tx) and isReceivingData(dc_rx) then 
                    v_timeoutCnt1   := (others =>'0');
                    read_data(dc_rx, rx_buff);
                    send_data(tx ,  rx_buff);
                    dataCount := dataCount +1;

                    if dataCount = data_in_packet then 
                        Send_end_Of_Stream(tx);
                        dataCount := 0;
                        packetCount := packetCount+1;
                        if packetCount = packets then 
                            v_state := IDLE_S;
                        end if;
                    end if;
                end if;
                if v_timeoutCnt1   = TIMEOUT_G1 then
                    v_state := IDLE_S;
                end if;
 


        end case;

    end if;
    end process;

    SCRODRegComb : process(r,usrRst,rxData,rxDataValid,rxDataLast,
                           txDataReady,regRdData,regAck,wordScrodRevC,EVNT_FLAG) is
        variable v : RegType;
    begin
        v := r;

        -- Resets for pulsed outputs
        v.regReq      := '0';
        v.txDataValid := '0';
        v.txDataLast  := '0';
        rxDataReady   <= '0';


        -- State machine
        case(r.state) is
            when IDLE_S =>
                -- v.errFlags := (others => '0'); --Mudit
                v.checksum := (others => '0');
                DC_cmdRespReq <= (others => '1'); --default enable listening to DCs
                data_flag <= '0';
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    -- Possible errors:
                    -- This is last, stay here
                    if rxDataLast = '1' then
                        v.state := IDLE_S;
                        -- Header doesn't match format
                    elsif rxData /= WORD_HEADER_C then		-- Shivang: Commented on Oct 12, 2020 to probe the issue of no response from CI to the PC commands
                        v.state := IDLE_S; -- v.state := DUMP_S;  , Mudit
                        -- Otherwise, move on
                    else
                        v.state := PACKET_SIZE_S;
                    end if;
                end if;
            when PACKET_SIZE_S =>
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    v.wordsLeft := rxData;
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' or rxData > 300 then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Otherwise, move on
                    else
                        v.state := PACKET_TYPE_S;
                    end if;
                end if;
            when PACKET_TYPE_S =>
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    v.wordsLeft := r.wordsLeft - 1;
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Packet type isn't understood
                    elsif rxData /= WORD_COMMAND_C then
                        -- v.errFlags := r.errFlags + ERR_BIT_TYPE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Otherwise, move on
                    else
                        v.state := COMMAND_TARGET_S;
                    end if;
                end if;
            when COMMAND_TARGET_S =>
                v.deviceID := rxData;
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    v.wordsLeft := r.wordsLeft - 1;
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit

                        -- Target doesn't match this SCROD or broadcast or DC
                    elsif rxData /= wordScrodRevC and rxData(31 downto 8) /= wordDC then --target must be SCROD, DC. leaving out all dev broadcast(rxData /= x"00000000" )
                        -- v.errFlags := r.errFlags + ERR_BIT_DEST_C; --else error. --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Otherwise, move on
                    else
                        if rxData = wordScrodRevC  then
                            loadQB <= '0';
                        elsif (rxData(31 downto 8) = wordDC) and (rxData(7 downto 0) /= broadcastDC) then
                            dc_id <= conv_integer(unsigned(rxData(7 downto 0)));
                            loadQB <= '1';
                        else --unused case, disallow broadcast
                            dc_id <= conv_integer(unsigned(broadcastDC)); --if broadcasting, set dc_id to "broadcast"
                            loadQB <= '1';
                        end if;
                        v.state := COMMAND_ID_S;
                    end if;
                end if;
            when COMMAND_ID_S =>
                    --  v.errFlags := (others => '0'); --Mudit
                v.wordOutCnt  := (others => '0');
                v.timeoutCnt  := (others => '0');
                if rxDataValid = '1' then
                    rxDataReady   <= '1';
                    -- Checksum calculation starts here
                    v.checksum   := rxData;
                    v.wordsLeft  := r.wordsLeft - 1;
                    v.commandId  := rxData(23 downto 0);
                    v.noResponse := rxData(31);
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Otherwise, move on
                    else
                        v.state := COMMAND_TYPE_S;
                    end if;
                end if;
            when COMMAND_TYPE_S =>
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    v.checksum  := r.checksum + rxData;
                    v.commandType   := rxData;
                    v.wordsLeft := r.wordsLeft - 1;
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Move on for recognized commands
                    elsif rxData = WORD_PING_C then
                        v.state := COMMAND_CHECKSUM_S; --Mudit
                    elsif rxData = WORD_READ_C or rxData = WORD_WRITE_C or rxData = WORD_WRITE_DAC or rxData = WORD_READ_DATA then --added by me,  or rxData = WORD_WRITE_DAC
                        v.state := COMMAND_DATA_S;
                        -- Unrecognized command, dump
                    else
                        -- v.errFlags := r.errFlags + ERR_BIT_COMM_TY_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                end if;
            when COMMAND_DATA_S =>
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                          v.command   := rxData;
                    v.checksum  := r.checksum + rxData;
                    v.regAddr   := rxData(15 downto 0);
                    v.regWrData := rxData(31 downto 16);
                   v.wordsLeft := r.wordsLeft - 1;      -- commented on 10/18/20 Shivang (to stop wordsleft decrement in the event of Rd/Wr.
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Move on for recognized commands
                    else
                        v.state := COMMAND_CHECKSUM_S;
                    end if;
                end if;
            when COMMAND_CHECKSUM_S =>
                    --  v.errFlags := (others => '0');
                if rxDataValid = '1' then
                    rxDataReady <= '1';
                    v.wordsLeft := r.wordsLeft - 1;
                    -- Possible errors:
                    -- This is last, go back to IDLE
                    if rxDataLast = '1' then
                        -- v.errFlags := r.errFlags + ERR_BIT_SIZE_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Bad checksum
                    elsif r.checksum /= rxData then
                        -- v.errFlags := r.errFlags + ERR_BIT_COMM_CS_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        -- Command accepted, move to execute state
                    elsif r.commandType = WORD_PING_C then
                        v.state := PING_S; --Mudit
                    elsif r.commandType = WORD_WRITE_C or r.commandType = WORD_WRITE_DAC then --by me,  or r.commandType = WORD_WRITE_DAC
                        v.state := WRITE_S;
                    elsif r.commandType = WORD_READ_C then
                        v.state := READ_S;
                        data_flag <= '0';
                    elsif r.commandType = WORD_READ_DATA then
                        v.state := READ_DATA;
                        data_flag <= '1';
                        -- Unrecognized command
                    else
                        -- v.errFlags := r.errFlags + ERR_BIT_COMM_TY_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                end if;
            
            when READ_DATA =>
                v.timeoutCnt := r.timeoutCnt + 1;
                -- v.txDataLast := '0';
                tready_fifo <= '0';
                if loadQB = '1' then -- if reading DC, listen to QBLink
                    DC_cmdRespReq(dc_id-1) <= '1';
                    v.checksum := (others => '0');
                    if txDataReady = '1' and tvalid_fifo = '1' then
                        v.timeoutCnt1 := (others => '0');
                        v.txDataValid := '1';
                        v.txData := tdata_fifo;
                        tready_fifo <= '1';
                        v.dataCount := v.dataCount + 1;
                        -- v.txDataLast := '1';
                        if v.dataCount = data_in_packet then
                            v.txDataLast := '1';
                            v.dataCount := 0;
                            if v.packetCount = packets then
                                v.state := IDLE_S;
                                v.packetCount := 0;
                            else v.packetCount := v.packetCount + 1;
                            end if;
                        -- else v.state := READ_DATA;
                        end if;
                    else v.timeoutCnt1 := r.timeoutCnt1 + 1;
                    end if;

                    if r.timeoutCnt1 = TIMEOUT_G1 then -- if QBLink does output a word before timeout, send error to PC
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        v.packetCount := 0;
                        v.dataCount := 0;
                    end if;
                    
                end if;

            --commented by Mudit
            when PING_S =>
                if r.noResponse = '1' then
                    v.state := IDLE_S;
                else
                    if loadQB = '1' then
                        if dc_id /= broadcastDC then
                        -- 	if serialClkLck = "1" and trigLinkSync = "1" then --check if QBLink is up (hardcoded), earlier "1111"
                        -- 		v.checksum := (others => '0');
                        -- 		v.state    := PING_RESPONSE_S;
                        -- 	else
                        -- 		v.errFlags := r.errFlags + QBLINK_FAILURE_C;
                        -- 		v.state := ERR_RESPONSE_S;
                        -- 	end if;
                        -- else
                        --     if dc_id > num_DC+1 then 
                        --         v.errFlags := r.errFlags + QBLINK_FAILURE_C;
                        --         v.state := ERR_RESPONSE_S;
                        --     else

                            if serialClkLck(dc_id-1) = '1' and trigLinkSync(dc_id-1) = '1' then --check if QBLink is up (hardcoded)
                                v.checksum := (others => '0');
                                v.state    := PING_RESPONSE_S;
                            else
                                -- v.errFlags := r.errFlags + QBLINK_FAILURE_C;
                                v.state := IDLE_S;
                            end if;
                        --     end if;
                        end if;
                    else
                        v.checksum := (others => '0');
                        v.state    := PING_RESPONSE_S;
                    end if;
                end if;

            when READ_S =>
                  v.timeoutCnt := r.timeoutCnt + 1;
                if loadQB = '1' then -- if reading DC, listen to QBLink
                    if dc_id /= broadcastDC then --IF not broadcasting to all DCs
                        DC_cmdRespReq(dc_id-1) <= '1';
                        if DC_RESP_VALID(dc_id-1) = '1' then --wait for DC to send register data
                            --do not use v.regRdData to collect readback data
                            if r.noResponse = '1' then --if noResponse setting on, skip response to PC
                                v.state := IDLE_S; --CHECK_MORE_S, Mudit
                            else --if noResponse setting off, send Register data in response to PC
                                v.checksum := (others => '0');
                                v.state    := READ_RESPONSE_S;
                            end if;
                        elsif r.timeoutCnt = TIMEOUT_G then -- if QBLink does output a word before timeout, send error to PC
                            -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                            v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        end if;
                    elsif dc_id = broadcastdc then
                        if DC_RESP_VALID = (DC_RESP_VALID'range => '1') then --wait for DC to send register data
                            --do not use v.regRdData to collect readback data
                            if r.noResponse = '1' then --if noResponse setting on, skip response to PC
                                v.state := IDLE_S; --CHECK_MORE_S, Mudit
                            else --if noResponse setting off, send Register data in response to PC
                                v.checksum := (others => '0');
                                v.state    := READ_RESPONSE_S;
                            end if;
                        elsif r.timeoutCnt = TIMEOUT_G then -- if QBLink does output a word before timeout, send error to PC
                            -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                            v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                        end if;
                    end if; --end all cases for DC reading
                    ----------------------------------------------------
                else  --if reading SCROD register:
                    v.regOp      := '0'; -- set Registers to read mode
                    v.regReq     := '1'; --request operation
                    if (regAck = '1') then
                        v.regRdData := regRdData;
                        v.regReq    := '0';
                        if r.noResponse = '1' then
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := READ_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then -- if SCROD register has not acknowledged before timeout, send error to PC
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                end if;

            when WRITE_S => --TEMP allow cmd interpreter to write to both SCROD and DC registers simulataneously to test dual functionality
                v.timeoutCnt := r.timeoutCnt + 1;
                if (loadQB = '1') and (dc_id /= broadcastDC) then --if writing to DC: wait for them to repeat correct address
                    DC_cmdRespReq(dc_id-1) <= '1';
                    if DC_RESP(15 downto 0) = r.regAddr then --write operation is successful if DC repeats address, even if simultaneous SCROD register operation fails.
                        if r.noResponse = '1' then --skip response to PC if noResponse setting is on
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then --if the DC does not repeat register before timeout, error is raised (even if SCROD register is written successfully).
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;

                elsif (loadQB = '1') and (dc_id = broadcastDC) then
                    DC_cmdRespReq(num_DC downto 0) <= (others => '1');
                    if (DC_RESP(15 downto 0) = r.regAddr) then
                        if r.noResponse = '1' then --skip response to PC if noResponse setting is on
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then --if the DC does not repeat register before timeout, error is raised (even if SCROD register is written successfully).
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                    -----------------------------
                else
                    v.regOp      := '1'; --enable write to SCROD Register
                    v.regReq     := '1'; --start writing
                    if (regAck = '1') then --if not reading DC, make sure SCROD register write was successful.
                        v.regReq    := '0';
                        if r.noResponse = '1' then
                            v.state := IDLE_S; --CHECK_MORE_S, Mudit
                        else
                            v.checksum := (others => '0');
                            v.state    := WRITE_RESPONSE_S;
                        end if;
                    elsif r.timeoutCnt = TIMEOUT_G then
                        -- v.errFlags := r.errFlags + ERR_BIT_TIMEOUT_C; --Mudit
                        v.state    := IDLE_S; --ERR_RESPONSE_S, Mudit
                    end if;
                end if;

            when READ_RESPONSE_S =>
                DC_cmdRespReq <= (others =>'0');
                if regAck = '0' and r.regReq = '0' then
                    v.txDataValid := '1';
                    if (loadQB = '0')  then
                        case conv_integer(r.wordOutCnt) is
                            when 0 => v.txData := WORD_HEADER_C;
                            when 1 => v.txData := x"00000006";
                            when 2 => v.txData := WORD_ACK_C;
                            when 3 => v.txData := r.deviceID;
                            when 4 => v.txData := x"00" & r.commandId;
                            when 5 => v.txData := WORD_READ_C;
                            when 6 => v.txData := r.regRdData & r.regAddr;
                            when 7 => v.txData     := r.checksum;
                            v.txDataLast := '1';
                            v.state      := IDLE_S; --CHECK_MORE_S, Mudit
                        when others => v.txData := (others => '1');
                        end case;


                    elsif	(dc_id /= broadcastDC) and (loadQB = '1') then
                        case conv_integer(r.wordOutCnt) is
                            when 0 => v.txData := WORD_HEADER_C;
                            when 1 => v.txData := x"00000006";
                            when 2 => v.txData := WORD_ACK_C;
                            when 3 => v.txData := r.deviceID;
                            when 4 => v.txData := x"00" & r.commandId;
                            when 5 => v.txData := WORD_READ_C;
                            when 6 => v.txData := DC_RESP;
                            when 7 => v.txData     := r.checksum;
                            v.txDataLast := '1'; 
                            v.state      := IDLE_S; --CHECK_MORE_S, Mudit
                        when others => v.txData := (others => '1');
                        end case;
                    end if;
                    if txDataReady = '1' then
                        v.checksum   := r.checksum + v.txData;
                        v.wordOutCnt := r.wordOutCnt + 1;
                    end if;
                end if;

            when WRITE_RESPONSE_S =>
                DC_cmdRespReq <= (others=>'0');
                if regAck = '0' and r.regReq = '0' then
                    v.txDataValid := '1';
                    case conv_integer(r.wordOutCnt) is
                        when 0 => v.txData := WORD_HEADER_C;
                        when 1 => v.txData := x"00000006";
                        when 2 => v.txData := WORD_ACK_C;
                        when 3 => v.txData := r.deviceID;
                        when 4 => v.txData := x"00" & r.commandId;
                        when 5 => v.txData := r.commandType; --WORD_WRITE_C, r.commandType; earlier, by me
                        when 6 => v.txData := r.regWrData & r.regAddr; --for all cases, just repeat what you wrote.
                        when 7 => v.txData     := v.checksum;
                        v.txDataLast := '1';
                        v.state      := IDLE_S; --CHECK_MORE_S, Mudit
                    when others => v.txData := (others => '1');
                    end case;
                    if txDataReady = '1' then
                        v.checksum   := r.checksum + v.txData;
                        v.wordOutCnt := r.wordOutCnt + 1;
                    end if;
                end if;

            when PING_RESPONSE_S =>
                v.txDataValid := '1';
                case conv_integer(r.wordOutCnt) is
                    when 0 => v.txData := WORD_HEADER_C;
                    when 1 => v.txData := x"00000005";
                    when 2 => v.txData := WORD_ACK_C;
                    when 3 => v.txData := r.deviceID;
                    when 4 => v.txData := x"00" & r.commandId;
                    when 5 => v.txData := WORD_PING_C;
                    when 6 => v.txData     := v.checksum;
                    v.txDataLast := '1';
                    v.state      := IDLE_S; --CHECK_MORE_S, Mudit
                when others => v.txData := (others => '1');
                end case;
                if txDataReady = '1' then
                    v.checksum   := r.checksum + v.txData;
                    v.wordOutCnt := r.wordOutCnt + 1;
                end if;



            when others =>
                v.state := IDLE_S;
                DC_cmdRespReq <= (others => '1');
        end case;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        -- Reset logic
        if (usrRst = '1') then
            v := REG_INIT_C;
        end if;
        --Event Handling
        -- if EVNT_FLAG = '1' then
        -- 	v := REG_INIT_C;
        -- 	QB_RdEn <= (others => '0');
        -- else
        QB_RdEn <= DC_cmdRespReq;
        -- end if;
        -- Register interfaces
        regAddr     <= r.regAddr;
        regWrData   <= r.regWrData;
        regReq      <= r.regReq;
        regOp       <= r.regOp;
        DC_CMD      <= QB_loadReg(2);
        --rxdata, M
        --rxDataReady   <= '0';
        -- Assignment of combinatorial variable to signal
        rin <= v;

    end process;

 

    seq1 : process (usrClk) is
    begin
        if (rising_edge(usrClk)) then
            r <= rin after GATE_DELAY_G;
            if r.state = COMMAND_DATA_S and loadQB = '1' then
                QB_loadReg(0) <= r.commandType;
                start_load <= '1'; --by Mudit, earlier zero
            elsif r.state = COMMAND_CHECKSUM_S and loadQB = '1' then
                QB_loadReg(1) <= r.command;
                start_load1 <= '1';
            -- elsif r.state = CHECK_MORE_S or r.state = ERR_RESPONSE_S then
            -- 	start_load <= '0'; --Mudit
            end if;
                
            if dc_ack = '1' then
                start_load <= '0';
            end if;
            if dc_ack1 = '1' then
                start_load1 <= '0';
            end if;
        end if;
    end process;

        txData      <= r.txData;
        txDataValid <= r.txDataValid;
        txDataLast  <= r.txDataLast;



    QBload_reg1 : process (dataClk) is
    begin
        if(rising_edge(dataClk)) then
            if start_load = '1' then
                dc_ack <= '1';
                if dc_id = 10 then
                    QB_WrEn <= (others =>'1');
                else
                    QB_WrEn(dc_id-1) <= '1';
                end if;
                QB_loadReg(2) <= QB_loadReg(0);
                -- start_load <= '0';
            -- else
            -- 	dc_ack <= '0';
            --     QB_WrEn <= (others =>'0');
            -- end if;

            elsif start_load1 = '1' and start_load = '0' then
                dc_ack1 <= '1';
                if dc_id = 10 then
                    QB_WrEn <= (others =>'1');
                else
                    QB_WrEn(dc_id-1) <= '1';
                end if;
                QB_loadReg(2) <= QB_loadReg(1);
                -- start_load1 <= '0';
            else
                dc_ack1 <= '0';
                dc_ack <= '0';
                QB_WrEn <= (others =>'0');
            end if;    
        end if;
    end process;

data_fifo_out: entity work.fifo_data_w32_d32
    PORT MAP (
           S_ARESETN                 => not usrRst,
           M_AXIS_TVALID             => tvalid_fifo,
           M_AXIS_TREADY             => tready_fifo,
           M_AXIS_TDATA              => tdata_fifo,
           S_AXIS_TVALID             => DC_RESP_VALID_data(dc_id-1),
           S_AXIS_TREADY             => s_axis_tready,
           S_AXIS_TDATA              => DC_RESP_data,
           M_ACLK                    => usrClk,
           S_ACLK                    => dataClk);
end rtl;

