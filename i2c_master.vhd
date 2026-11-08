-------------------------------------------------------------------------------
-- Файл    : i2c_master.vhd
-- Проєкт  : Лабораторна робота №3, варіант 17 (I2C, VHDL)
-- Опис    : I2C Master (Standard mode, 100 кГц за замовчуванням).
--           Виконує одну однобайтову транзакцію за імпульс start:
--             Запис : START | ADDR+W | ACK | DATA | ACK | STOP
--             Читання: START | ADDR+R | ACK | DATA | NACK | STOP
--           Лінії SCL/SDA - open-drain ('0' або 'Z'); потрібні pull-up резистори.
--           Clock stretching та multi-master НЕ підтримуються.
--
-- Часова діаграма одного біта (4 фази, кожна = DIV тактів clk):
--           фаза 0     фаза 1     фаза 2     фаза 3
--   SCL  ____________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________
--   SDA  <- виставляється тут | семплюється у фазі 2 ->
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity i2c_master is
  generic (
    CLK_FREQ_HZ : positive := 50_000_000;  -- частота системного тактового сигналу
    I2C_FREQ_HZ : positive := 100_000      -- бажана частота SCL
  );
  port (
    clk       : in    std_logic;                      -- системний такт
    rst       : in    std_logic;                      -- синхронний скид, активний '1'
    -- Інтерфейс керування
    start     : in    std_logic;                      -- імпульс 1 такт: запуск транзакції
    rw        : in    std_logic;                      -- '0' = запис, '1' = читання
    addr      : in    std_logic_vector(6 downto 0);   -- 7-бітна адреса slave
    data_in   : in    std_logic_vector(7 downto 0);   -- байт для запису
    data_out  : out   std_logic_vector(7 downto 0);   -- прочитаний байт
    busy      : out   std_logic;                      -- '1' поки триває транзакція
    done      : out   std_logic;                      -- імпульс 1 такт: транзакцію завершено
    ack_error : out   std_logic;                      -- '1' якщо slave не відповів ACK
    -- Шина I2C
    scl       : out   std_logic;                      -- open-drain
    sda       : inout std_logic                       -- open-drain
  );
end entity i2c_master;

architecture rtl of i2c_master is

  -- Кількість тактів clk на чверть періоду SCL
  constant DIV : positive := CLK_FREQ_HZ / (4 * I2C_FREQ_HZ);

  type state_t is (S_IDLE, S_START, S_SEND, S_ACK, S_READ, S_NACK, S_STOP);
  signal state : state_t := S_IDLE;

  signal div_cnt    : integer range 0 to DIV - 1 := 0;
  signal tick       : std_logic;                       -- імпульс на початку кожної фази
  signal ph         : unsigned(1 downto 0) := "00";    -- поточна фаза біта (0..3)

  signal scl_o      : std_logic := '1';                -- '0' = тягнемо SCL вниз
  signal sda_o      : std_logic := '1';                -- '0' = тягнемо SDA вниз

  signal shreg      : std_logic_vector(7 downto 0) := (others => '0');  -- зсувний регістр
  signal bit_cnt    : integer range 0 to 7 := 0;       -- лічильник бітів у байті
  signal addr_phase : std_logic := '1';                -- '1' = передаємо байт адреси
  signal addr_r     : std_logic_vector(6 downto 0) := (others => '0');
  signal data_r     : std_logic_vector(7 downto 0) := (others => '0');
  signal rw_r       : std_logic := '0';
  signal ack_seen   : std_logic := '0';                -- '1' якщо slave відповів ACK
  signal busy_r     : std_logic := '0';
  signal ack_err_r  : std_logic := '0';

begin

  assert DIV >= 2
    report "CLK_FREQ_HZ is too low for the requested I2C_FREQ_HZ" severity failure;

  -- Open-drain виходи: активний рівень лише '0', інакше лінія відпущена ('Z')
  scl <= '0' when scl_o = '0' else 'Z';
  sda <= '0' when sda_o = '0' else 'Z';

  busy      <= busy_r;
  ack_error <= ack_err_r;
  tick      <= '1' when div_cnt = DIV - 1 else '0';

  ---------------------------------------------------------------------------
  -- Основний автомат (FSM). Кожен стан виконується по 4 фази на біт.
  ---------------------------------------------------------------------------
  fsm : process (clk)
  begin
    if rising_edge(clk) then
      done <= '0';

      -- Дільник частоти: працює лише поза станом IDLE
      if div_cnt = DIV - 1 then
        div_cnt <= 0;
      else
        div_cnt <= div_cnt + 1;
      end if;

      if rst = '1' then
        state      <= S_IDLE;
        scl_o      <= '1';
        sda_o      <= '1';
        busy_r     <= '0';
        ack_err_r  <= '0';
        ph         <= "00";
        div_cnt    <= 0;
        ack_seen   <= '0';

      elsif state = S_IDLE then
        scl_o   <= '1';
        sda_o   <= '1';
        div_cnt <= 0;
        ph      <= "00";
        if start = '1' then
          addr_r     <= addr;
          data_r     <= data_in;
          rw_r       <= rw;
          addr_phase <= '1';
          busy_r     <= '1';
          ack_err_r  <= '0';
          state      <= S_START;
        end if;

      elsif tick = '1' then
        ph <= ph + 1;                       -- 3 -> 0 переповнення автоматичне

        case state is

          -- START: SDA падає при SCL = '1'
          when S_START =>
            case ph is
              when "00"   => sda_o <= '1'; scl_o <= '1';
              when "01"   => sda_o <= '0';                       -- умова START
              when "10"   => null;
              when others =>
                scl_o   <= '0';
                shreg   <= addr_r & rw_r;                        -- ADDR[6:0] & R/W
                bit_cnt <= 7;
                state   <= S_SEND;
            end case;

          -- Передача байта (адреса або дані), старший біт першим
          when S_SEND =>
            case ph is
              when "00"   => scl_o <= '0'; sda_o <= shreg(7);
              when "01"   => scl_o <= '1';
              when "10"   => null;
              when others =>
                scl_o <= '0';
                shreg <= shreg(6 downto 0) & '0';
                if bit_cnt = 0 then
                  state <= S_ACK;
                else
                  bit_cnt <= bit_cnt - 1;
                end if;
            end case;

          -- Прийом ACK від slave (SDA відпущено, slave тягне вниз)
          when S_ACK =>
            case ph is
              when "00"   => scl_o <= '0'; sda_o <= '1';
              when "01"   => scl_o <= '1';
              when "10"   =>
                if to_x01(sda) = '0' then ack_seen <= '1'; else ack_seen <= '0'; end if;
              when others =>
                scl_o <= '0';
                if ack_seen = '0' then                            -- NACK -> помилка
                  ack_err_r <= '1';
                  state     <= S_STOP;
                elsif addr_phase = '1' then
                  bit_cnt <= 7;
                  if rw_r = '1' then
                    state <= S_READ;
                  else
                    shreg      <= data_r;
                    addr_phase <= '0';
                    state      <= S_SEND;
                  end if;
                else
                  state <= S_STOP;                                -- дані записано
                end if;
            end case;

          -- Прийом байта від slave
          when S_READ =>
            case ph is
              when "00"   => scl_o <= '0'; sda_o <= '1';
              when "01"   => scl_o <= '1';
              when "10"   => shreg <= shreg(6 downto 0) & to_x01(sda);
              when others =>
                scl_o <= '0';
                if bit_cnt = 0 then
                  state <= S_NACK;
                else
                  bit_cnt <= bit_cnt - 1;
                end if;
            end case;

          -- Master відповідає NACK після останнього прочитаного байта
          when S_NACK =>
            case ph is
              when "00"   => scl_o <= '0'; sda_o <= '1'; data_out <= shreg;
              when "01"   => scl_o <= '1';
              when "10"   => null;
              when others => scl_o <= '0'; state <= S_STOP;
            end case;

          -- STOP: SDA росте при SCL = '1'
          when S_STOP =>
            case ph is
              when "00"   => scl_o <= '0'; sda_o <= '0';
              when "01"   => scl_o <= '1';
              when "10"   => sda_o <= '1';                        -- умова STOP
              when others =>
                busy_r <= '0';
                done   <= '1';
                state  <= S_IDLE;
            end case;

          when others => state <= S_IDLE;
        end case;
      end if;
    end if;
  end process fsm;

end architecture rtl;