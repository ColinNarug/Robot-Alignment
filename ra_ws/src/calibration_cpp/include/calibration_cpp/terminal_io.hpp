#pragma once
#include <string>
#include <vector>

namespace calibration_cpp
{
struct Resolution { int width{0}; int height{0}; };

class TerminalRawMode
{
public:
  TerminalRawMode();
  ~TerminalRawMode();
  TerminalRawMode(const TerminalRawMode&) = delete;
  TerminalRawMode& operator=(const TerminalRawMode&) = delete;
  bool ok() const { return ok_; }
private:
  bool ok_{false};
  void* old_termios_{nullptr};
};

enum KeyCode : int
{
  KEY_NONE  = 0,
  KEY_ESC   = 27,
  KEY_UP    = 1000,
  KEY_DOWN  = 1001,
  KEY_LEFT  = 1002,
  KEY_RIGHT = 1003,
};

int read_key_nonblocking();
Resolution select_resolution_menu(const std::vector<Resolution>& options, const std::string& title);
void print_key_help();
}  // namespace calibration_cpp