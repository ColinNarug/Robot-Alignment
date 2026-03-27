#include "calibration_cpp/terminal_io.hpp"
#include <iostream>
#include <stdexcept>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>

namespace calibration_cpp
{
static termios* as_termios(void* p) { return reinterpret_cast<termios*>(p); }

TerminalRawMode::TerminalRawMode()
{
  old_termios_ = std::calloc(1, sizeof(termios));
  if (!old_termios_) return;

  if (tcgetattr(STDIN_FILENO, as_termios(old_termios_)) != 0) return;

  termios raw = *as_termios(old_termios_);
  raw.c_lflag &= static_cast<unsigned>(~(ECHO | ICANON));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;

  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
  ok_ = true;
}

TerminalRawMode::~TerminalRawMode()
{
  if (ok_ && old_termios_) (void)tcsetattr(STDIN_FILENO, TCSANOW, as_termios(old_termios_));
  if (old_termios_) std::free(old_termios_);
}

static bool stdin_has_data()
{
  fd_set set; FD_ZERO(&set); FD_SET(STDIN_FILENO, &set);
  timeval tv{}; tv.tv_sec = 0; tv.tv_usec = 0;
  const int rc = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
  return (rc > 0) && FD_ISSET(STDIN_FILENO, &set);
}

int read_key_nonblocking()
{
  if (!stdin_has_data()) return KEY_NONE;

  unsigned char c = 0;
  const ssize_t n = ::read(STDIN_FILENO, &c, 1);
  if (n <= 0) return KEY_NONE;

  if (c != 27) return static_cast<int>(c);

  if (!stdin_has_data()) return KEY_ESC;
  unsigned char seq[2] = {0,0};
  if (::read(STDIN_FILENO, &seq[0], 1) != 1) return KEY_ESC;
  if (::read(STDIN_FILENO, &seq[1], 1) != 1) return KEY_ESC;

  if (seq[0] == '[') 
  {
    switch (seq[1]) 
    {
      case 'A': return KEY_UP;
      case 'B': return KEY_DOWN;
      case 'C': return KEY_RIGHT;
      case 'D': return KEY_LEFT;
      default:  return KEY_ESC;
    }
  }
  return KEY_ESC;
}

static void clear_screen() { std::cout << "\033[2J\033[H"; }

Resolution select_resolution_menu(const std::vector<Resolution>& options, const std::string& title)
{
  if (options.empty()) throw std::runtime_error("No resolution options provided");
  TerminalRawMode raw;
  if (!raw.ok()) throw std::runtime_error("Failed to enter terminal raw mode (STDIN not a TTY?)");

  int idx = 0;
  while (true) 
  {
    clear_screen();
    std::cout << title << "\n";
    std::cout << "Use UP/DOWN and ENTER to select. ESC to cancel.\n\n";
    for (int i = 0; i < (int)options.size(); ++i) 
    {
      const auto &r = options[(size_t)i];
      const bool sel = (i == idx);
      std::cout << (sel ? " > " : "   ") << r.width << " x " << r.height << "\n";
    }

    const int k = read_key_nonblocking();
    if (k == KEY_NONE) { usleep(10 * 1000); continue; }
    if (k == KEY_UP) idx = (idx - 1 + (int)options.size()) % (int)options.size();
    else if (k == KEY_DOWN) idx = (idx + 1) % (int)options.size();
    else if (k == '\n' || k == '\r') return options[(size_t)idx];
    else if (k == KEY_ESC) throw std::runtime_error("Resolution selection canceled");
  }
}

void print_key_help()
{
  std::cout << "\nKeyboard controls:\n"
            << "  c : capture a sample\n"
            << "  x : compute + save calibration and exit\n"
            << "  q or ESC : cancel (save nothing) and exit\n";
}
}  // namespace calibration_cpp