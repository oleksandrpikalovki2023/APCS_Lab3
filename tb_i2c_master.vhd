-------------------------------------------------------------------------------
-- Файл    : tb_i2c_master.vhd
-- Опис    : Тестбенч для i2c_master. Містить модель slave (адреса 0x50)
--           з одним регістром пам'яті та pull-up на лініях SCL/SDA.
-- Тести   : 1) запис 0xA5   2) читання (очікуємо 0xA5)
--           3) запис 0x3C   4) читання (очікуємо 0x3C)
--           5) неіснуюча адреса 0x2A -> очікуємо ack_error = '1'
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity tb_i2c_master is
end entity tb_i2c_master;

architecture sim of tb_i2c_master is

  constant CLK_PERIOD  : time := 250 ns;                  -- 50 МГц
  constant SLAVE_ADDR  : std_logic_vector(6 downto 0) := "1010000";  -- 0x50

  signal clk       : std_logic := '0';
  signal rst       : std_logic := '1';
  signal start     : std_logic := '0';
  signal rw        : std_logic := '0';
  signal addr      : std_logic_vector(6 downto 0) := (others => '0');
  signal data_in   : std_logic_vector(7 downto 0) := (others => '0');
  signal data_out  : std_logic_vector(7 downto 0);
  signal busy      : std_logic;
  signal done      : std_logic;
  signal ack_error : std_logic;

  signal scl       : std_logic;
  signal sda       : std_logic;
  signal slave_sda : std_logic := 'Z';
  signal slave_mem : std_logic_vector(7 downto 0) := x"00";  -- регістр slave

  signal sim_done  : boolean := false;
  signal errors    : natural := 0;

begin

  -- Pull-up резистори шини I2C
  scl <= 'H';
  sda <= 'H';
  sda <= slave_sda;

  -- Тактовий сигнал
  clk <= not clk after CLK_PERIOD / 2 when not sim_done;

  -- Перевіряємий модуль
  dut : entity work.i2c_master
  generic map (CLK_FREQ_HZ => 4_000_000, I2C_FREQ_HZ => 100_000)
    port map (
      clk => clk, rst => rst, start => start, rw => rw, addr => addr,
      data_in => data_in, data_out => data_out, busy => busy, done => done,
      ack_error => ack_error, scl => scl, sda => sda);

  ---------------------------------------------------------------------------
  -- Модель I2C slave
  ---------------------------------------------------------------------------
  slave : process
    variable sh      : std_logic_vector(7 downto 0);
    variable is_read : boolean;
  begin
    slave_sda <= 'Z';
    -- Очікуємо умову START (SDA падає при SCL = '1')
    wait until falling_edge(sda) and to_x01(scl) = '1';

    -- Приймаємо байт адреси
    for i in 7 downto 0 loop
      wait until rising_edge(scl);
      sh(i) := to_x01(sda);
    end loop;
    wait until falling_edge(scl);
    is_read := (sh(0) = '1');

    if sh(7 downto 1) = SLAVE_ADDR then
      slave_sda <= '0';                                  -- ACK адреси
      wait until falling_edge(scl);

      if is_read then
        -- Slave передає байт пам'яті
        for i in 7 downto 0 loop
          if slave_mem(i) = '0' then slave_sda <= '0'; else slave_sda <= 'Z'; end if;
          wait until falling_edge(scl);
        end loop;
        slave_sda <= 'Z';                                -- master відповідає NACK
      else
        -- Slave приймає байт і відповідає ACK
        slave_sda <= 'Z';
        for i in 7 downto 0 loop
          wait until rising_edge(scl);
          sh(i) := to_x01(sda);
        end loop;
        wait until falling_edge(scl);
        slave_mem <= sh;
        slave_sda <= '0';                                -- ACK даних
        wait until falling_edge(scl);
        slave_sda <= 'Z';
      end if;
    end if;                                              -- інакше NACK (SDA відпущено)

    -- Очікуємо умову STOP (SDA росте при SCL = '1')
    wait until rising_edge(sda) and to_x01(scl) = '1';
  end process slave;

  ---------------------------------------------------------------------------
  -- Стимули та перевірки
  ---------------------------------------------------------------------------
  stim : process

    procedure transaction (
      constant a : in std_logic_vector(6 downto 0);
      constant r : in std_logic;
      constant d : in std_logic_vector(7 downto 0)) is
    begin
      wait until rising_edge(clk);
      addr <= a; rw <= r; data_in <= d; start <= '1';
      wait until rising_edge(clk);
      start <= '0';
      wait until rising_edge(clk) and done = '1';
      wait for 5 us;                                     -- пауза між транзакціями
    end procedure;

    procedure check (constant cond : in boolean; constant msg : in string) is
    begin
      if cond then
        report "PASS: " & msg severity note;
      else
        report "FAIL: " & msg severity error;
        errors <= errors + 1;
      end if;
    end procedure;

  begin
    rst <= '1';
    wait for 10 * CLK_PERIOD;
    rst <= '0';
    wait for 10 * CLK_PERIOD;

    -- Тест 1: запис 0xA5
    transaction(SLAVE_ADDR, '0', x"A5");
    check(ack_error = '0',      "T1 write: ACK received");
    check(slave_mem = x"A5",    "T1 write: slave stored 0xA5");

    -- Тест 2: читання
    transaction(SLAVE_ADDR, '1', x"00");
    check(ack_error = '0',      "T2 read: ACK received");
    check(data_out = x"A5",     "T2 read: got 0xA5");

    -- Тест 3 і 4: інше значення
    transaction(SLAVE_ADDR, '0', x"3C");
    check(slave_mem = x"3C",    "T3 write: slave stored 0x3C");
    transaction(SLAVE_ADDR, '1', x"00");
    check(data_out = x"3C",     "T4 read: got 0x3C");

    -- Тест 5: неіснуюча адреса
    transaction("0101010", '0', x"FF");
    check(ack_error = '1',      "T5 wrong address: ack_error = 1");

    if errors = 0 then
      report "=== ALL TESTS PASSED ===" severity note;
    else
      report "=== TESTS FAILED: " & integer'image(errors) & " ===" severity error;
    end if;

    sim_done <= true;
    wait;
  end process stim;

end architecture sim;