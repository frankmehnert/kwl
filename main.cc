/**
 * Control Helios KWL EC270/370 Pro.
 *
 * Written by Frank Mehnert <frank.mehnert@gmail.com>
 */

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <term.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace {

#define USED    __attribute__((unused))
#define DEVICE  "ttyUSB0"

// Protocol:
//  [0]:    device (0x10 .. 0x13)
//  [1]:    0=read, 1=write, 5=acknowledge
//  [2]:    length of payload
//
// READ (to master / from master):
//  [3]:    variable index
//
// WRITE (to master):
//  [3]:    variable index
//  [4..7]: value (depends on variable size)
//
//   SET FAN:
//    [3]:    Var_35_fan_level
//    [4..5]: 0xaa 0x00 = set to manual
//            0xaa 0x01 = set to AUTO
//            0xaa 0xbb = no change
//            0    0xbb = manual level 0
//            1    0xbb = manual level 1
//            2    0xbb = manual level 2
//            3    0xbb = manual level 3
//
// ACKNOWLEDGE (from master):
//  [3]:    variable index
//  [4]:    0x55
//
// RJ12: GND = 0 (red)
//               (braun)
//               (grün)

static unsigned _maxy;
static bool     _terminate   = false;
static bool     _interactive = false;

enum
{
  Var_00_calendar_mon     = 0x00, // 24 x 8-bit
  Var_01_calendar_tue     = 0x01, // 24 x 8-bit
  Var_02_calendar_wed     = 0x02, // 24 x 8-bit
  Var_03_calendar_thu     = 0x03, // 24 x 8-bit
  Var_04_calendar_fri     = 0x04, // 24 x 8-bit
  Var_05_calendar_sat     = 0x05, // 24 x 8-bit
  Var_06_calendar_sun     = 0x06, // 24 x 8-bit (low/high half hour)
  Var_07_date_month_year  = 0x07, // 24-bit ([0]=day [1]=month [2]=year)
  Var_08_time_hour_min    = 0x08, // 16-bit ([0]=hour, [1]=minutes)
  Var_0d_back_up_heating  = 0x0d, //  8-bit (0=disabled, 1=enabled)
  Var_0e_preheat_temp     = 0x0e, // 16-bit (request at offset 0x50!)
  Var_0f_party_enabled    = 0x0f, //  8-bit (write-only: 0=disabled, 1=enabled)
  Var_10_party_curr_time  = 0x10, // 16-bit (minutes, current seleted)
  Var_11_party_time       = 0x11, // 16-bit (minutes, pre-selected)
  Var_14_ext_contact      = 0x14, //  8-bit
  Var_15_hours_on         = 0x15, // 32-bit (hours)
  Var_16_fan_1_voltage    = 0x16, // 2 x 16-bit (Zuluft/Abluft 10th Volt)
  Var_17_fan_2_voltage    = 0x17, // 2 x 16-bit (Zuluft/Abluft 10th Volt)
  Var_18_fan_3_voltage    = 0x18, // 2 x 16-bit (Zuluft/Abluft 10th Volt)
  Var_19_fan_4_voltage    = 0x19, // 2 x 16-bit (Zuluft/Abluft 10th Volt)
  Var_1a_vacation_start   = 0x1a, // 24-bit (day, month, year)
  Var_1b_vacation_end     = 0x1b, // 24-bit (day, month, year)
  Var_1c_unknown          = 0x1c, // 16-bit 0x20 0x03 (800)
  Var_1d_unknown          = 0x1d, //  8-bit 0x3c
  Var_1e_bypass1_temp     = 0x1e, // 16-bit (10th °C, Außenluftbegrenzung)
  Var_1f_frostschutz      = 0x1f, // 16-bit (10th °C)
  Var_20_unknown          = 0x20, //  8-bit 0x01
  Var_21_weekoffs_co2     = 0x21, //  8-bit (ppm)
  Var_22_weekoffs_humdty  = 0x22, //  8-bit (%)
  Var_23_weekoffs_temp    = 0x23, //  8-bit (°C)
  Var_35_fan_level        = 0x35, //  8-bit (0..4) -- READ ONLY
  Var_37_min_fan_level    = 0x37, //  8-bit (0..4)
  Var_38_change_filter    = 0x38, //  8-bit (months)
  Var_3a_sensors_temp     = 0x3a, // 10 x 16-bit
  Var_3b_sensors_co2      = 0x3b, //  4 x 16-bit
  Var_3c_sensors_humidity = 0x3c, //  4 x 16-bit
  Var_3f_unknown          = 0x3f, //  8-bit 0x00
  Var_40_unknown          = 0x40, //  8-bit 0x0a
  Var_41_unknown          = 0x41, //  8-bit 0x0a
  Var_42_party_level      = 0x42, //  8-bit (fan level)
  Var_43_unknown          = 0x43, //  8-bit 0x00
  Var_44_unknown          = 0x44, //  8-bit 0x00
  Var_45_zuluft_level     = 0x45, //  8-bit 0x02
  Var_46_abluft_level     = 0x46, //  8-bit 0x03
  Var_47_unknown          = 0x47, //  8-bit 0x00
  Var_48_software_version = 0x48, // 16-bit 0x83 0x00 (1.31)
  Var_49_nachlaufzeit     = 0x49, //  8-bit seconds
  Var_4a_unknown          = 0x4a, //  8-bit 0x3c
  Var_4b_unknown          = 0x4b, //  8-bit 0x3c
  Var_4c_unknown          = 0x4c, //  8-bit 0x02
  Var_4d_unknown          = 0x4d, //  8-bit 0x02
  Var_4e_vacation_enabled = 0x4e, //  8-bit (0=off, 1=on)
  Var_4f_preheat_enabled  = 0x4f, //  8-bit (0=off, 1=on)
  Var_50_preheat_temp     = 0x50, // 16-bit (10th °C)
  Var_51_unknown          = 0x51, //  8-bit 0x02
  Var_52_weekoffs_enabled = 0x52, //  8-bit (0=off, 1=on)
  Var_54_quiet_curr_time  = 0x54, // 16-bit (minutes, current)
  Var_55_quiet_enabled    = 0x55, //  8-bit (write-only: 0=disabled, 1=enabled)
  Var_56_quiet_time       = 0x56, //  8-bit (minutes)
  Var_57_quiet_level      = 0x57, //  8-bit (fan level)
  Var_58_unknown          = 0x58, // 26 x 8-bit 0x00 0x00 ...
  Var_59_unknown          = 0x59, // 26 x 8-bit 0x01 0x00 ...
  Var_5a_unknown          = 0x5a, // 26 x 8-bit 0x02 0x00 ...
  Var_5b_unknown          = 0x5b, // 26 x 8-bit 0x03 0x00 ...
  Var_5c_unknown          = 0x5c, // 26 x 8-bit 0x04 0x00 ...
  Var_5d_unknown          = 0x5d, // 26 x 8-bit 0x05 0x00 ...
  Var_5e_unknown          = 0x5e, // 26 x 8-bit 0x06 0x00 ...
  Var_5f_unknown          = 0x5f, //  8-bit 0x00
  Var_60_bypass2_temp     = 0x60, //  8-bit (Bypasstemperatur)
  Var_61_unknown          = 0x61, // 24-bit 0x1e 0x01 0x00
  Var_62_unknown          = 0x62, // 24-bit 0x05 0x00 0x00
  Var_63_unknown          = 0x63, // 24-bit 0x0f 0x01 0x00
  Var_64_unknown          = 0x64, // 24-bit 0x05 0x04 0x06
  Var_65_unknown          = 0x65, // 16-bit 0xa5 0x00 (165)
  Var_66_unknown          = 0x66, // 16-bit 0x90 0x01 (400)
  Var_67_unknown          = 0x67, // 32-bit 0x00 0x00 0x0f 0x0f
};

static char const *get_var_name(unsigned var)
{
  switch (var)
    {
    case Var_00_calendar_mon:     return "calendar monday";
    case Var_01_calendar_tue:     return "calendar tuesday";
    case Var_02_calendar_wed:     return "calendar wednesday";
    case Var_03_calendar_thu:     return "calendar thursday";
    case Var_04_calendar_fri:     return "calendar friday";
    case Var_05_calendar_sat:     return "calendar saturday";
    case Var_06_calendar_sun:     return "calendar sunday";
    case Var_07_date_month_year:  return "date month year";
    case Var_08_time_hour_min:    return "time hour min";
    case Var_0d_back_up_heating:  return "back up heating";
    case Var_0e_preheat_temp:     return "preheating temperatur";
    case Var_0f_party_enabled:    return "party enabled";
    case Var_10_party_curr_time:  return "party current time";
    case Var_11_party_time:       return "party time";
    case Var_14_ext_contact:      return "external contact";
    case Var_15_hours_on:         return "hours on";
    case Var_16_fan_1_voltage:    return "fan 1 voltage";
    case Var_17_fan_2_voltage:    return "fan 2 voltage";
    case Var_18_fan_3_voltage:    return "fan 3 voltage";
    case Var_19_fan_4_voltage:    return "fan 4 voltage";
    case Var_1a_vacation_start:   return "vacation start";
    case Var_1b_vacation_end:     return "vacation end";
    case Var_1e_bypass1_temp:     return "bypass1 temperature";
    case Var_1f_frostschutz:      return "frostschutz";
    case Var_21_weekoffs_co2:     return "week offset co2";
    case Var_22_weekoffs_humdty:  return "week offset humdty";
    case Var_23_weekoffs_temp:    return "week offset temp";
    case Var_35_fan_level:        return "fan level";
    case Var_37_min_fan_level:    return "minimum fan level";
    case Var_38_change_filter:    return "change filter";
    case Var_3a_sensors_temp:     return "sensors temperature";
    case Var_3b_sensors_co2:      return "sensors co2";
    case Var_3c_sensors_humidity: return "sensors humidity";
    case Var_42_party_level:      return "party level";
    case Var_45_zuluft_level:     return "zuluft level";
    case Var_46_abluft_level:     return "abluft level";
    case Var_48_software_version: return "software version";
    case Var_49_nachlaufzeit:     return "nachlaufzeit";
    case Var_4e_vacation_enabled: return "vacation enabled";
    case Var_4f_preheat_enabled:  return "preheating enabled";
    case Var_50_preheat_temp:     return "preheating temperature";
    case Var_52_weekoffs_enabled: return "week offset enabled";
    case Var_54_quiet_curr_time:  return "quiet current time";
    case Var_55_quiet_enabled:    return "quiet enabled";
    case Var_56_quiet_time:       return "quiet time";
    case Var_57_quiet_level:      return "quiet_level";
    case Var_60_bypass2_temp:     return "bypass2 temperature";
    default:                      return "unknown";
    }
};

static int64_t get_time()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (int64_t)now.tv_sec * 1000000000LL + now.tv_nsec;
}

static USED void sleep_ms(long ms)
{
  struct timespec t = { ms / 1000, ms * 1000000 };
  int ret;
  do
    {
      struct timespec r;
      ret = nanosleep(&t, &r);
      t = r;
    } while (ret == -1 && errno == EINTR);
}

static void get_y(unsigned *y)
{
  struct termios term, restore;
  tcgetattr(0, &term);
  restore = term;
  term.c_lflag &= ~(ICANON|ECHO);
  tcsetattr(0, TCSANOW, &term);
  write(1, "\033[6n", 4);
  *y = 0;
  bool done = false;
  for (unsigned i = 0; ; ++i)
    {
      char c;
      ssize_t ret = read(0, &c, 1);
      if (!ret || c == 'R')
        break;
      if (c == ';')
        done = true;
      if (i > 1 && !done)
        {
          *y *= 10;
          *y += c - '0';
        }
    }
  tcsetattr(0, TCSANOW, &restore);
}

static void set_y(unsigned y)
{ printf("\033[%u;1H", y); }

static void get_maxy(unsigned *maxy)
{
  struct winsize win;
  ioctl(0, TIOCGWINSZ, &win);
  *maxy = win.ws_row;
}

static void set_maxy(unsigned maxy, unsigned y = 0)
{
  if (maxy < _maxy && y == 0)
    get_y(&y);
  printf("\033[1;%ur", maxy);
  if (maxy < _maxy)
    {
      if (y >= maxy)
        y = maxy;
      set_y(y);
    }
}

static void signal_handler(int)
{ _terminate = true; }

static int try_getchar()
{
  char c = 0;
  if (read(0, &c, 1) < 0)
    perror ("read()");
  return c;
}

static bool peek_input(int *d1, int *d2, int *d3)
{
  struct termios term;
  tcgetattr(0, &term);
  struct termios restore = term;
  term.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOCTL);
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &term);
  *d2 = *d3 = 0;
  if ((*d1 = try_getchar()) == '\033')
    if ((*d2 = try_getchar()))
      *d3 = try_getchar();
  tcsetattr(0, TCSANOW, &restore);
  return d1 != 0;
}

class Paket
{
public:
  void new_paket(uint8_t const *buf, unsigned size)
  {
    _buf = buf;
    _size = size;
    if (_size >= 4 && _buf)
      {
        if (_size > 4U + buf[2])
          _size = 4U + buf[2];
        _is_valid = is_valid_checksum();
      }
    else
      _is_valid = false;
  }

  unsigned size() const
  { return _size; }

  uint8_t const *raw() const
  { return _buf; }

  uint8_t u8(unsigned i) const
  { return i < _size ? _buf[4 + i] : 0; }

  uint16_t u16(unsigned i) const
  { return i + 4 + 2*i < _size ? *(uint16_t*)&_buf[4 + 2*i] : 0; }

  uint32_t u32(unsigned i) const
  { return i + 4 + 4*i < _size ? *(uint32_t*)&_buf[4 + 4*i] : 0; }

  uint8_t dsize() const
  { return _buf[2]; }

  bool is_valid_checksum() const
  {
    uint8_t chksum = 0;
    for (unsigned i = 0; i < _size - 1; ++i)
      chksum += _buf[i];
    chksum += 1;
    return chksum == _buf[_size - 1];
  }

  bool is_valid() const
  { return _is_valid; }

  bool is_start_status() const
  { return _buf[0] == 0xff && _buf[1] == 0xff; }

  bool is_start_addr() const
  { return _is_valid && _buf[0] >= 0x10 && _buf[0] <= 0x13; }

  bool is_ping(uint8_t b0) const
  { return _is_valid && _buf[0] == b0 && _buf[1] == 0 && dsize() == 0; }

  bool is_status(uint8_t idx, uint8_t size) const
  { return _is_valid && _buf[1] == 1 && dsize() == size && _buf[3] == idx; }

  bool is_ack_10()
  {
    return _is_valid && _buf[0] == 0x10 && _buf[1] == 5
                     && dsize() == 2 && _buf[4] == 0x55;
  }

  bool is_request(uint8_t idx)
  { return _is_valid && _buf[1] == 0 && dsize() == 1 && _buf[3] == idx; }

  bool is_fan_no_change()
  {
    return _is_valid && _buf[1] == 1 && dsize() == 3
        && _buf[3] == Var_35_fan_level && _buf[4] == 0xaa && _buf[5] == 0xbb;
  }

  // must also work if invalid
  void print(char const *prefix, bool suppress_chksum) const
  {
    printf("%s", prefix);
    for (unsigned i = 0; i < _size - (unsigned)suppress_chksum; ++i)
      {
        if (i == 4)
          printf("[ ");
        printf("%02x ", _buf[i]);
        if (_size > 4 && i == _size - (unsigned)suppress_chksum - 1)
          printf("]");
      }
    printf("\033[m\n");
  }

  void print_got_temp() const
  {
    printf("\033[32mtemp\033[m ");
    for (unsigned i = 0; i < 10; ++i)
      {
        uint16_t v = *(uint16_t *)(_buf + 4 + 2*i);
        if (v != 9990)
          printf("%d.%d°C ", v / 10, v % 10);
      }
    putchar('\n');
  }

  void print_got_co2() const
  {
    printf("\033[32mCO₂\033[m ");
    for (unsigned i = 0; i < 4; ++i)
      {
        uint16_t v = *(uint16_t *)(_buf + 4 + 2*i);
        if (v != 9999)
          printf("%d.%d ", v / 10, v% 10);
      }
    putchar('\n');
  }

  void print_got_humidity() const
  {
    printf("\033[32mhumidity\033[m ");
    for (unsigned i = 0; i < 4; ++i)
      {
        uint16_t v = *(uint16_t *)(_buf + 4 + 2*i);
        if (v != 999)
          printf("%d.%d ", v / 10, v% 10);
      }
    putchar('\n');
  }

  void print_status(uint16_t *temp, uint16_t bypass, uint16_t party,
                    uint16_t quiet, bool at_bottom) const
  {
    if (_size != 27)
      { printf("\033[31mwrong broadcast size\033[m %d\n", _size); return; }

    static char const *const day_of_week[7] =
    { "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun" };
    static char const *temp_symbol[4] = { "↓", "←", "↑", "→" };
    static unsigned temp_idx[4] = { 0, 3, 1, 2 };

    static unsigned progress;
    ++progress;

    unsigned y;
    if (at_bottom)
      {
        get_y(&y);
        set_maxy(_maxy);
        set_y(_maxy);
      }
    char prbuf[3] = { at_bottom ? ".oOo"[progress % 4] : '\0', ' ', '\0' };
    printf("%s%s %02d.%02d.20%02d %d:%02d %s/%d ",
           prbuf, _buf[4] < 7 ? day_of_week[_buf[4]] : "???",
           _buf[3], _buf[5], _buf[6], _buf[7], _buf[8],
           _buf[10] ? "\033[32mauto\033[m" : "\033[31mMANUAL\033[m", _buf[9]);
    for (unsigned i = 0; i < 4; ++i)
      if (temp[i] != 9990)
        {
          unsigned j = temp_idx[i];
          printf("%s%d.%d°C ", temp_symbol[j], temp[j] / 10, temp[j] % 10);
        }
    if (bypass != 0xffff)
      printf("bypass %u.%u°C ", bypass / 10, bypass % 10);
    if (party != 0)
      printf("\033[33mparty %dmin\033[m ", party);
    if (quiet != 0)
      printf("\033[33mquiet %dmin\033[m ", quiet);

#if 0
    // print unknown data
    printf("-- ");
    for (unsigned i = 11; i < _size - 1; ++i)
      printf("%02x ", _buf[i]);
#endif

    printf("\033[K");
    if (at_bottom)
      {
        set_maxy(_maxy - 1, y);
        fflush(stdout);
      }
  }

private:
  uint8_t const *_buf = nullptr;
  unsigned      _size = 0;
  bool          _is_valid = false;
};

class Kwl
{
public:
  Kwl()
  {}

  ~Kwl()
  {
    if (_sp >= 0)
      close(_sp);
  }

  bool uart_open()
  {
    _sp = open("/dev/" DEVICE, O_RDWR);
    if (_sp < 0)
      {
        perror("open");
        return false;
      }
    return true;
  }

  bool uart_setup()
  {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(_sp, &tty) != 0)
      { perror("tcgetattr"); return false; }

    tty.c_cflag &= ~(PARENB|CSTOPB|CSIZE|CRTSCTS);
    tty.c_cflag |= CS8|CREAD|CLOCAL;

    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY);

    tty.c_oflag &= ~(OPOST|ONLCR);

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    cfsetispeed(&tty, B19200); // specific to KWL EC-270/370 Pro
    cfsetospeed(&tty, B19200); // specific to KWL EC-270/370 Pro

    if (tcsetattr(_sp, TCSANOW, &tty) != 0)
      { perror("tcsetattr"); return false; }

    return true;
  }

  bool uart_read(uint8_t *c)
  { return (read(_sp, c, 1) == 1); }

  void send_frame(uint8_t *buf, uint8_t size) const
  {
    if (size < 4)
      { printf("\033[31msnd_frame: Invalid size!\n"); return; }

    buf[2] = size - 4;
    uint8_t chksum = 0;
    for (uint8_t i = 0; i < size - 1; ++i)
      chksum += buf[i];
    chksum += 1;
    buf[size - 1] = chksum;
    sleep_ms(5);
    int written = write(_sp, buf, size);
    if (written != (ssize_t)size)
      {
        printf("\033[31msnd_frame: Sent %d != %d bytes!\n", written, size);
        return;
      }
    if (buf[3] != Var_3a_sensors_temp && false)
      _p.print("\033[1msuccessful sent: ", true);
  }

  void send_get_var(uint8_t idx) const
  {
    uint8_t snd[5] = { 0x13, 0, 1, idx };
    send_frame(snd, sizeof(snd));
  }

  void send_set_var_8bit(uint8_t idx, uint8_t val) const
  {
    uint8_t snd[6] = { 0x13, 1, 2, idx, val };
    send_frame(snd, sizeof(snd));
  }

  void send_set_var_16bit(uint8_t idx, uint16_t val) const
  {
    uint8_t snd[7] = { 0x13, 1, 3, idx,
                       (uint8_t)(val % 256), (uint8_t)(val / 256) };
    send_frame(snd, sizeof(snd));
  }

  void send_set_var_32bit(uint8_t idx, uint32_t val) const
  {
    uint8_t snd[9] = { 0x13, 1, 5, idx,
                       (uint8_t)(val & 0xff),
                       (uint8_t)((val >> 8) & 0xff),
                       (uint8_t)((val >> 16) & 0xff),
                       (uint8_t)((val >> 24) & 0xff) };
    send_frame(snd, sizeof(snd));
  }

  void print_calendar_level(unsigned level) const
  {
    switch (level)
      {
      case 1: printf("__"); break;
      case 2: printf("——"); break;
      case 3: printf("‾‾"); break;
      default: printf("% d", level); break;
      }
  }

  unsigned volume_flow(uint8_t v)
  {
    if (v <   1) return 0;
    if (v <  24) return 60;
    if (v <  30) return 80;
    if (v <  37) return 100;
    if (v <  43) return 120;
    if (v <  50) return 140;
    if (v <  56) return 160;
    if (v <  63) return 180;
    if (v <  69) return 200;
    if (v <  76) return 220;
    if (v <  82) return 240;
    if (v <  89) return 260;
    if (v <  95) return 280;
    if (v < 100) return 300;
    return 315;
  }

  void print_paket(uint64_t time)
  {
    if (!opt_verbose
        && (   _p.is_ping(0x10)
            || _p.is_ping(0x11)
            || _p.is_ping(0x12)
            || _p.is_ping(0x13)
            || _p.is_request(Var_35_fan_level)
            || _p.is_request(Var_3a_sensors_temp)
            || _p.is_request(Var_3b_sensors_co2)
            || _p.is_request(Var_3c_sensors_humidity)
            || _p.is_ack_10()
            || _p.is_fan_no_change()
           )
       )
      return;

    uint8_t const *buf = _p.raw();

    if (buf[1] == 5 && _p.dsize() == 2 && buf[4] == 0x55)
      printf("\033[32mack '%s' (%02x) written\033[m\n",
             get_var_name(buf[3]), buf[3]);

    else if (false) // print all pakets
      {
        printf("%4lldms (%02x) ", time / 1000000, buf[0]);
        _p.print("\033[1m", true); // uninterpreted
      }

    else if (buf[1] == 1 && _p.dsize() == 28 && buf[3] <= Var_06_calendar_sun)
      {
        for (unsigned i = 0; i < 24; ++i)
          {
            print_calendar_level(buf[i + 7] & 0xf);
            print_calendar_level(buf[i + 7] >> 4);
          }
        printf("\n");
        for (unsigned i = 0; i < 24; ++i)
          printf("%-4d", i);
        printf("\n");
      }

    else if (_p.is_status(Var_0e_preheat_temp, 3))
      printf("\033[32mpre-heating\033[m = %d.%d°C\n",
             _p.u16(0) / 10, _p.u16(0) % 10);

    else if (_p.is_status(Var_10_party_curr_time, 3))
      {
        _party = _p.u16(0);
        if (!_interactive)
          {
            if (_party == 0)
              printf("\033[32mparty disabled\033[m\n");
            else
              printf("\033[32mparty enabled for %dmin\n", _party);
          }
      }

    else if (_p.is_status(Var_11_party_time, 3))
      printf("\033[32mparty time\033[m = %dmin\n", _p.u16(0));

    else if (_p.is_status(Var_15_hours_on, 5))
      {
        uint32_t h = _p.u32(0);
        printf("\033[32mhours on\033[m = %dh (%d.%dyrs)\n",
               h, ((h * 10) / (365 * 24)) / 10, ((h * 10) / (365 * 24)) % 10);
      }

    else if (buf[1] == 1 && _p.dsize() == 5
             && buf[3] >= Var_16_fan_1_voltage
             && buf[3] <= Var_19_fan_4_voltage)
      printf("\033[32mvoltage fan %d\033[m = %d.%dV (%dm³/h) / %d.%dV (%dm³/h)\n",
             buf[3] - 0x15,
             _p.u16(0) / 10, _p.u16(0) % 10, volume_flow(_p.u16(0)),
             _p.u16(1) / 10, _p.u16(1) % 10, volume_flow(_p.u16(1)));

    else if (_p.is_status(Var_1e_bypass1_temp, 3))
      {
        _bypass = _p.u16(0);
        if (!_interactive)
          printf("\033[32mbypass1\033[m = %d.%d°C\n",
                 _bypass / 10, _bypass % 10);
      }

    else if (_p.is_status(Var_35_fan_level, 3))
      {
        if (_p.is_fan_no_change())
          printf("\033[32mfan no change\033[m\n");
        else if (_p.u8(0) == 0xaa && _p.u8(1) == 0)
          printf("\033[32mset fan \033[1mMANUAL\033[m\n");
        else if (_p.u8(0) == 0xaa && _p.u8(1) == 1)
          printf("\033[32mset fan \033[1mAUTO\033[m\n");
        else if (_p.u8(0) == 0xaa)
          printf("\033[32mset fan AUTO/MANUAL %d\n", _p.u8(1));
        else if (_p.u8(1) == 0xbb)
          printf("\033[32mset fan\033[m \033[31mLEVEL %d\033[m\n", _p.u8(0));
        else
          printf("\033[32mset fan\033[m \033[31mLEVEL %d\033[m %d\n",
                 _p.u8(0), _p.u8(1));
      }

    else if (_p.is_status(Var_38_change_filter, 2))
      printf("\033[32mchange filter\033[m = %dmth\n", _p.u8(0));

    else if (_p.is_status(Var_3a_sensors_temp, 21))
      {
        for (unsigned i = 0; i < 4; ++i)
          _temp[i] = _p.u16(1 + i);
      }

    else if (_p.is_status(Var_3b_sensors_co2, 9))
      _p.print_got_co2();

    else if (_p.is_status(Var_3c_sensors_humidity, 9))
      _p.print_got_humidity();

    else if (_p.is_status(Var_42_party_level, 2))
      printf("\033[32mparty level\033[m = %d\n", _p.u8(0));

    else if (_p.is_status(Var_49_nachlaufzeit, 2))
      printf("\033[32mnachlaufzeit\033[m = %ds\n", _p.u8(0));

    else if (_p.is_status(Var_4f_preheat_enabled, 2))
      printf("\033[32mpre-heating\033[m = %s\n",
             _p.u8(0) ? "enabled" : "disabled");

    else if (_p.is_status(Var_54_quiet_curr_time, 3))
      {
        _quiet = _p.u16(0);
        if (!_interactive)
          {
            if (_quiet == 0)
              printf("\033[32mquiet disabled\033[m\n");
            else
              printf("\033[32mquiet enabled for %dmin\n", _quiet);
          }
      }

    else if (_p.is_status(Var_55_quiet_enabled, 2))
      printf("\033[32mset quiet\033[m \033[1m%s\033[m (%d)\n",
             _p.u8(0) ? "enabled" : "disabled", _p.u8(0));

    else if (_p.is_status(Var_56_quiet_time, 2))
      printf("\033[32mquiet time\033[m = %dmin\n", _p.u8(0));

    else if (_p.is_status(Var_57_quiet_level, 2))
      printf("\033[32mquiet level\033[m = %d\n", _p.u8(0));

    else if (_p.is_status(Var_60_bypass2_temp, 2))
      printf("\033[32mbypass2\033[m = %d°C\n", _p.u8(0));

    else if (buf[0] == 0xff && buf[1] == 0xff)
      {
        _p.print_status(_temp, _bypass, _party, _quiet, time != 0);
        _fan_level = buf[9];
        _fan_auto  = buf[10] > 0;
      }

    else
      {
        printf("%4lldms (%02x) ", time / 1000000, buf[0]);
        _p.print("\033[1m", true); // uninterpreted
      }
  }

  void got_frame(int64_t time, uint8_t const *buf, unsigned size)
  {
    for (;; buf += _p.size(), size -= _p.size())
      {
        if (size < 4U)
          return; // normal end or drop remaining

        _p.new_paket(buf, size);
        if (!_p.is_valid())
          {
            if (!_first_frame)
              _p.print("\033[31mignoring ", false);
            else
              _first_frame = false;
            return;
          }

        _first_frame = false;
        ++_pakets_received;

        if (!_p.is_start_status() && !_p.is_start_addr())
          break;

        if (_p.is_start_status())
          memcpy(_status_buf, buf, sizeof(_status_buf));

        print_paket(time);
      }

    // unknown paket
    if (   !_p.is_ping(0x31) && !_p.is_ping(0x32)
        && !_p.is_ping(0x34) && !_p.is_ping(0x38)
        && !_p.is_ping(0x41) && !_p.is_ping(0x42)
        && !_p.is_ping(0x44) && !_p.is_ping(0x48)
        && !_p.is_ping(0x51) && !_p.is_ping(0x52)
        && !_p.is_ping(0x54) && !_p.is_ping(0x58))
      {
        printf("%4lldms ", time / 1000000);
        _p.print("\033[31munknown ", true);
      }
  }

  void our_turn()
  {
    static unsigned our_cnt = 0;
    ++our_cnt;
    if (our_cnt < 2)
      ;
    else if (initial_temp)
      {
        send_get_var(Var_3a_sensors_temp);
        initial_temp = false;
      }

    //
    // SETTER
    //
    else if (opt_set_time)
      {
        send_set_var_16bit(Var_08_time_hour_min, opt_set_time);
        opt_set_time = 0;
      }
    else if (opt_set_bypass)
      {
        if (opt_set_bypass == 0xffff) // toggle
          {
            if (_bypass == 0xffff)
              return; // wait until bypass temperature known
            opt_set_bypass = (_bypass < 200) ? 28 : 18;
          }
        send_set_var_16bit(Var_1e_bypass1_temp, opt_set_bypass * 10);
        opt_set_bypass = 0;
      }
    else if (opt_set_fan)
      {
        if (opt_set_fan == 0xe000 || opt_set_fan == 0xf000) // down/up
          {
            int change = (opt_set_fan == 0xf000) ? +1 : -1;
            if (_fan_level == -1)
              return; // wait until current fan level is available
            if (   (change == -1 && (_fan_level < 2 || _fan_level > 4))
                || (change == +1 && (_fan_level < 1 || _fan_level > 3)))
              opt_set_fan = 0;
            else
              {
                if (_fan_auto)
                  {
                    opt_set_fan = _fan_level + change;
                    send_set_var_16bit(Var_35_fan_level, 0x00aa);
                  }
                else
                  {
                    send_set_var_16bit(Var_35_fan_level,
                                       0xbb00 + _fan_level + change);
                    opt_set_fan = 0;
                  }
                _fan_level = -1; // re-read
                _fan_auto = -1;
              }
          }
        else if (opt_set_fan == 0xaa) // auto
          {
            send_set_var_16bit(Var_35_fan_level, 0x01aa);
            opt_set_fan = 0;
          }
        else if (opt_set_fan & 0x100) // first: disable auto
          {
            send_set_var_16bit(Var_35_fan_level, 0x00aa);
            opt_set_fan &= ~0x100;
          }
        else // set manual level
          {
            send_set_var_16bit(Var_35_fan_level, 0xbb00 + opt_set_fan);
            opt_set_fan = 0;
          }
      }
    else if (opt_set_party)
      {
        if (opt_set_party == 0xaa00)
          {
            send_set_var_8bit(Var_0f_party_enabled, 0);
            opt_set_party = 0;
          }
        else if (opt_set_party == 0xaa01)
          {
            send_set_var_8bit(Var_0f_party_enabled, 1);
            opt_set_party = 0;
          }
        else
          {
            send_set_var_16bit(Var_11_party_time, opt_set_party);
            opt_set_party = 0xaa01;
          }
      }
    else if (opt_set_quiet)
      {
        if (opt_set_quiet == 0xaa00)
          {
            send_set_var_8bit(Var_55_quiet_enabled, 0);
            opt_set_quiet = 0;
          }
        else if (opt_set_quiet == 0xaa01)
          {
            send_set_var_8bit(Var_55_quiet_enabled, 1);
            opt_set_quiet = 0;
          }
        else
          {
            send_set_var_16bit(Var_56_quiet_time, opt_set_quiet);
            opt_set_quiet = 0xaa01;
          }
      }
    else if (opt_set_voltage)
      {
        send_set_var_32bit(  Var_16_fan_1_voltage - 1
                           + (opt_set_voltage & 0xf),
                             (opt_set_voltage >> 16)
                           | (opt_set_voltage & 0xffff0000));
        opt_set_voltage = 0;
      }

    //
    // GETTER
    //
    else if (opt_get_bypass == 2)
      {
        send_get_var(Var_60_bypass2_temp);
        --opt_get_bypass;
      }
    else if (opt_get_bypass == 1)
      {
        send_get_var(Var_1e_bypass1_temp);
        --opt_get_bypass;
      }
    else if (opt_get_hours_on)
      {
        send_get_var(Var_15_hours_on);
        opt_get_hours_on = false;
      }
    else if (opt_get_voltage)
      {
        if (opt_get_voltage < 5)
          send_get_var(Var_19_fan_4_voltage + 1 - opt_get_voltage);
        --opt_get_voltage;
      }
    else if (opt_get_party_enabled)
      {
        send_get_var(Var_10_party_curr_time);
        opt_get_party_enabled = false;
      }
    else if (opt_get_party_time)
      {
        send_get_var(Var_11_party_time);
        opt_get_party_time = false;
      }
    else if (opt_get_party_level)
      {
        send_get_var(Var_42_party_level);
        opt_get_party_level = false;
      }
    else if (opt_get_quiet_enabled)
      {
        send_get_var(Var_54_quiet_curr_time);
        opt_get_quiet_enabled = false;
      }
    else if (opt_get_quiet_time)
      {
        send_get_var(Var_56_quiet_time);
        opt_get_quiet_time = false;
      }
    else if (opt_get_quiet_level)
      {
        send_get_var(Var_57_quiet_level);
        opt_get_quiet_level = false;
      }
    else if (opt_get_cal)
      {
        send_get_var(Var_00_calendar_mon + opt_get_cal - 1);
        opt_get_cal = 0;
      }
    else if (opt_get_preheating)
      {
        if (opt_get_preheating == 2)
          send_get_var(Var_4f_preheat_enabled);
        else if (opt_get_preheating == 1)
          send_get_var(Var_50_preheat_temp);
        --opt_get_preheating;
      }
    else if (opt_get_run_on_time)
      {
        send_get_var(Var_49_nachlaufzeit);
        opt_get_run_on_time = false;
      }
    else if (opt_get_filter_time)
      {
        send_get_var(Var_38_change_filter);
        opt_get_filter_time = false;
      }
    else if (!opt_do_loop)
      _terminate = true;

    //
    // Low-frequency GETTERS
    //
    else if ((our_cnt % 4) == 3)
      send_get_var(Var_3a_sensors_temp);
    else if (_party && ((our_cnt % 8) == 2))
      send_get_var(Var_10_party_curr_time);
    else if (_quiet && ((our_cnt % 8) == 2))
      send_get_var(Var_54_quiet_curr_time);
  }

  void print_last_status()
  {
    if (_status_buf[2] != 0)
      {
        _p.new_paket(_status_buf, sizeof(_status_buf));
        print_paket(0);
      }
  }

private:
  Paket _p;
  uint16_t _temp[4] =
  {
    9990, // ↓ Außen
    9990, // ← Abluft
    9990, // ↑ Fortluft
    9990  // → Zuluft
  };
  uint64_t _pakets_received = 0;
  int      _sp = -1;
  bool     _first_frame = true;
  uint8_t  _status_buf[27] = { 0, };
  int      _fan_level = -1;
  int      _fan_auto = -1;
  uint16_t _bypass = 0xffff;
  uint16_t _party = 0;
  uint16_t _quiet = 0;

public:
  bool     opt_do_loop = false;
  uint16_t opt_set_time = 0;
  uint16_t opt_set_bypass = 0;
  uint16_t opt_set_fan = 0;
  uint16_t opt_set_party = 0;
  uint16_t opt_set_quiet = 0;
  uint32_t opt_set_voltage = 0;
  unsigned opt_get_bypass = 0;
  unsigned opt_get_cal = 0;
  bool     opt_get_hours_on = false;
  bool     opt_get_party_enabled = false;
  bool     opt_get_party_time = false;
  bool     opt_get_party_level = false;
  bool     opt_get_quiet_enabled = false;
  bool     opt_get_quiet_time = false;
  bool     opt_get_quiet_level = false;
  unsigned opt_get_voltage = 0;
  unsigned opt_get_preheating = 0;
  bool     opt_get_run_on_time = false;
  bool     opt_get_filter_time = false;
  bool     opt_verbose = false;
  bool     initial_temp = true;
};

void print_help()
{
  printf(
    "\n"
    "kwl [OPTION...]\n"
    "\n"
    " -?, --help                show this help\n"
    " -l, --loop                loop execution until Ctrl-C / ESC\n"
    "\n"
    "     --get-bypass          get bypass temperatures (°C)\n"
    "     --get-calendar DAY    get calendar for a specific day (0/Mon..6/Sun)\n"
    "     --get-change-filter   get time before change filter (mth)\n"
    "     --get-hours-on        get number of hours (h)\n"
    "     --get-party-enabled   get remaining time for party\n"
    "     --get-party-time      get party time (min)\n"
    "     --get-party-level     get party level\n"
    "     --get-pre-heating     get pre-heating enabled/temperature (°C)\n"
    "     --get-quiet-enabled   get remaining quiet time\n"
    "     --get-quiet-time      get quiet time (min)\n"
    "     --get-quiet-level     get quiet level\n"
    "     --get-run-on-time     get run-on time (s)\n"
    "     --get-voltage         get voltage for all fan levels (V)\n"
    "\n"
    " -b, --set-bypass 0|1|TEMP set bypass temperature (disable|enable|°C)\n"
    " -f, --set-fan a|m:LEVEL   set fan level (auto or manual level 1..4)\n"
    " -p, --set-party 0|1|TIME  set party (disable/enable/time)\n"
    " -q, --set-quiet 0|1|TIME  set quiet (disable/enable/time)\n"
    " -t, --set-time HH:MM      set time of day\n"
    " -v, --set-voltage L:V     set voltage for a certain level\n"
    "\n"
    "     --verbose             show incoming pakets\n"
    );
}

static int scan_options(Kwl &kwl, int argc, char **argv)
{
  for (;;)
    {
      static struct option long_opts[] =
      {
        { "get-all",           no_argument,       0,   7 },
        { "get-bypass",        no_argument,       0,   1 },
        { "get-calendar",      required_argument, 0,   2 },
        { "get-change-filter", no_argument,       0,  12 },
        { "get-hours-on",      no_argument,       0,   3 },
        { "get-quiet-enabled", no_argument,       0,  14 },
        { "get-quiet-time",    no_argument,       0,   5 },
        { "get-quiet-level",   no_argument,       0,   9 },
        { "get-party-enabled", no_argument,       0,  13 },
        { "get-party-time",    no_argument,       0,   6 },
        { "get-party-level",   no_argument,       0,   8 },
        { "get-pre-heating",   no_argument,       0,  10 },
        { "get-voltage",       no_argument,       0,   4 },
        { "get-run-on-time",   no_argument,       0,  11 },
        { "help",              no_argument,       0, '?' },
        { "loop",              no_argument,       0, 'l' },
        { "interactive",       no_argument,       0, 'i' },
        { "set-bypass",        required_argument, 0, 'b' },
        { "set-fan",           required_argument, 0, 'f' },
        { "set-party",         required_argument, 0, 'p' },
        { "set-time",          required_argument, 0, 't' },
        { "set-voltage",       required_argument, 0, 'v' },
        { "verbose",           no_argument,       0,  15 }
      };

      int opts_index;
      int c = getopt_long(argc, argv, "?b:ilf:p:q:t:v:", long_opts, &opts_index);
      if (c == -1)
        break;

      unsigned long u0, u1, u2;
      char *nptr;
      switch (c)
        {
        case '?':
          print_help();
          return 2;

        case 2:
          u0 = strtoul(optarg, &nptr, 10);
          if (u0 > 6)
            { printf("get-calendar: wrong day\n"); return 1; }
          kwl.opt_get_cal = u0 + 1;
          break;

        case 3:
          kwl.opt_get_hours_on = true;
          break;

        case 'b':
          u0 = strtoul(optarg, &nptr, 10);
          if (u0 == 0)
            kwl.opt_set_bypass = 28;
          else if (u0 == 1)
            kwl.opt_set_bypass = 18;
          else if (u0 < 18 || u0 > 30)
            { printf("set-bypass: temperature out of range\n"); return 1; }
          kwl.opt_set_bypass = u0;
          // fall-through

        case 1:
          kwl.opt_get_bypass = 2;
          break;

        case 'i':
        case 'l':
          _interactive = true;
          kwl.opt_do_loop = true;
          kwl.opt_get_bypass = 1;
          kwl.opt_get_party_enabled = true;
          kwl.opt_get_quiet_enabled = true;
          printf(
            "In interactive mode -- abort with Ctrl-C or ESC.\n"
            "↑..increase fan, ↓..decrease fan, a..auto mode, b..toggle bypass.\n");
          break;

        case 'f':
          if (optarg[0] == 'a')
            kwl.opt_set_fan = 0xaa;
          else if (optarg[0] == 'm')
            {
              if (optarg[1] != ':')
                { printf("set-fan: wrong manual format\n"); return 1; }
              if (optarg[2] < '1' || optarg[3] > '4')
                { printf("set-fan: wrong manual level\n"); return 1; }
              kwl.opt_set_fan = (1U << 8) | (optarg[2] - '0');
            }
          else
            { printf("set-fan: a/m:<level>\n"); return 1; }
          break;

        case 'p':
          if (optarg[0] == '0' && optarg[1] == '\0')
            kwl.opt_set_party = 0xaa00;
          else if (optarg[0] == '1' && optarg[1] == '\0')
            kwl.opt_set_party = 0xaa01;
          else
            {
              u0 = strtoul(optarg, &nptr, 10);
              if (u0 > 120 || *nptr != '\0')
                { printf("set-party: wrong format\n"); return 1; }
              kwl.opt_set_party = u0;
            }
          break;

        case 'q':
          if (optarg[0] == '0')
            kwl.opt_set_quiet = 0xaa00;
          else if (optarg[0] == '1')
            kwl.opt_set_quiet = 0xaa01;
          else
            {
              u0 = strtoul(optarg, &nptr, 10);
              if (u0 > 120 || *nptr != '\0')
                { printf("set-quiet: wrong format\n"); return 1; }
              kwl.opt_set_quiet = u0;
            }
          break;

        case 't':
          u0 = strtoul(optarg, &nptr, 10);
          if (u0 > 23)
            { printf("set-time: wrong hour\n"); return 1; }
          if (*nptr != ':')
            { printf("set-time: wrong format\n"); return 1; }
          u1 = strtoul(nptr + 1, &nptr, 10);
          if (u1 > 59)
            { printf("set-time: wrong minutes\n"); return 1; }
          if (*nptr != '\0')
            { printf("set-time: wrong format %d\n", *nptr); return 1; }
          kwl.opt_set_time = (uint16_t)u0 + (uint16_t)(u1 << 8);
          break;

        case 'v':
          u0 = strtoul(optarg, &nptr, 10);
          if (u0 == 0 || u0 > 4)
            { printf("set-voltage: wrong level\n"); return 1; }
          if (*nptr != ':')
            { printf("set-voltage: format level:voltage\n"); return 1; }
          u1 = strtoul(nptr + 1, &nptr, 10);
          u1 *= 10;
          if (*nptr != '\0')
            {
              if (*nptr != '.' && *nptr != ',')
                { printf("set-voltage: wrong voltage separator\n"); return 1; }
              u2 = strtoul(nptr + 1, &nptr, 10);
              if (*nptr != '\0')
                { printf("set-voltage: unexpected characters\n"); return 1; }
              if (u2 > 9)
                { printf("set-voltage: wrong voltage lower part\n"); return 1; }
              u1 += u2;
            }
          if (u1 > 100)
            { printf("set-voltage: voltage too high\n"); return 1; }
          kwl.opt_set_voltage = u0 + (u1 << 16);
          // fall-through

        case 4:
          kwl.opt_get_voltage = 4;
          break;

        case 5:
          kwl.opt_get_quiet_time = true;
          break;

        case 6:
          kwl.opt_get_party_time = true;
          break;

        case 7:
          kwl.opt_get_quiet_enabled = true;
          kwl.opt_get_quiet_time = true;
          kwl.opt_get_quiet_level = true;
          kwl.opt_get_party_time = true;
          kwl.opt_get_party_level = true;
          kwl.opt_get_party_enabled = true;
          kwl.opt_get_hours_on = true;
          kwl.opt_get_voltage = 4;
          kwl.opt_get_bypass = 2;
          kwl.opt_get_preheating = 2;
          kwl.opt_get_run_on_time = true;
          kwl.opt_get_filter_time = true;
          break;

        case 8:
          kwl.opt_get_party_level = true;
          break;

        case 9:
          kwl.opt_get_quiet_level = true;
          break;

        case 10:
          kwl.opt_get_preheating = 2;
          break;

        case 11:
          kwl.opt_get_run_on_time = true;
          break;

        case 12:
          kwl.opt_get_filter_time = true;
          break;

        case 13:
          kwl.opt_get_party_enabled = true;
          break;

        case 14:
          kwl.opt_get_quiet_enabled = true;
          break;

        case 15:
          kwl.opt_verbose = true;
          break;

        default:
          printf("Unknown option '%c'\n", c);
          return 1;
        }
    }

  return 0;
}

} // namespace

int main(int argc, char **argv)
{
  unsigned y;
  get_maxy(&_maxy);
  get_y(&y);
  if (y >= _maxy)
    {
      putchar('\n');
      --y;
    }
  set_maxy(_maxy - 1, _maxy - 1);
  fflush(stdout);
  signal(SIGINT, signal_handler);

  printf("Helios KWL control\n");

  Kwl kwl;

  if (!kwl.uart_open())
    return 1;

  int retval = 0;
  if (open("/var/lock/" DEVICE, O_CREAT | O_EXCL | O_WRONLY, S_IWUSR) > 0)
    {
      if (kwl.uart_setup())
        {
          retval = scan_options(kwl, argc, argv);
          if (retval == 0)
            {
              unsigned idx = 0;
              uint8_t buf[128];

              do
                {
                  int64_t time = get_time();

                  uint8_t c;
                  if (!kwl.uart_read(&c))
                    continue;

                  if (_interactive)
                    {
                      int d1, d2, d3;
                      if (peek_input(&d1, &d2, &d3))
                        {
                          printf("\r\033[K");
                          fflush(stdout);
                          if (d1 == '\033')
                            {
                              if (d2 == '\0')
                                break; // single ESCape -- terminate
                              if (d2 == '[')
                                {
                                  if (d3 == 'A')
                                    kwl.opt_set_fan = 0xf000; // up
                                  else if (d3 == 'B')
                                    kwl.opt_set_fan = 0xe000; // down
                                }
                            }
                          else if (d1 == 'a') // set fan to auto
                            kwl.opt_set_fan = 0xaa;
                          else if (d1 == 'b') // toggle bypass
                            kwl.opt_set_bypass = 0xffff, kwl.opt_get_bypass = 1;
                        }
                    }

                  time = get_time() - time;
                  if (time >= 25000000)
                    {
                      kwl.got_frame(time, buf, idx);
                      idx = 0;
                    }

                  if (idx >= sizeof(buf))
                    {
                      printf("\033[31mBuffer overflow\033[m\n");
                      idx = 0;
                      continue;
                    }

                  buf[idx++] = c;

                  if (idx == 4 && *(uint32_t *)buf == 0x14000013)
                    kwl.our_turn();
                }
              while (!_terminate);
            }
        }
      else
        printf("Cannot setup /dev/" DEVICE "\n");

      unlink("/var/lock/" DEVICE);
    }
  else
    printf("Cannot lock /var/lock/" DEVICE "\n");

  get_y(&y);
  set_maxy(_maxy);
  set_y(_maxy);
  printf("\033[K"); // remove status
  get_maxy(&_maxy); // might have changed in the meantime
  set_maxy(_maxy);
  set_y(y);
  fflush(stdout);
  kwl.print_last_status();
  putchar('\n');

  return retval;
}
