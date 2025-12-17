#include <signal.h>
#include <stdio.h>
#include <functional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>

#ifdef _WIN32
# include <windows.h>  // NO LINT
#else
# include <termios.h>  // NO LINT
# include <unistd.h>   // NO LINT
#endif

// Keys
static constexpr char KEYCODE_SPACE = 0x20;
static constexpr char KEYCODE_q = 0x71;
static constexpr char KEYCODE_Q = 0x51;
static constexpr char KEYCODE_x = 0x78;
static constexpr char KEYCODE_X = 0x58;

static std::atomic<char> running{true};

class KeyboardReader final 
{
public:
  KeyboardReader() 
  {
    #ifdef _WIN32
      hstdin_ = GetStdHandle(STD_INPUT_HANDLE);
      if (hstdin_ == INVALID_HANDLE_VALUE) throw std::runtime_error("Failed to get stdin handle");
      if (!GetConsoleMode(hstdin_, &old_mode_)) throw std::runtime_error("Failed to get console mode");
      DWORD new_mode = ENABLE_PROCESSED_INPUT;  // Ctrl-C processing
      if (!SetConsoleMode(hstdin_, new_mode)) throw std::runtime_error("Failed to set console mode");
    #else
      if (tcgetattr(0, &cooked_) < 0) throw std::runtime_error("Failed to get console mode");
      struct termios raw;
      memcpy(&raw, &cooked_, sizeof(struct termios));
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      raw.c_cc[VTIME] = 1; // deciseconds (poll every 100ms)
      raw.c_cc[VMIN] = 0;  // non-blocking
      if (tcsetattr(0, TCSANOW, &raw) < 0) throw std::runtime_error("Failed to set console mode");
      #endif
  }

  char readOne() 
  {
    char c = 0;
    #ifdef _WIN32
    INPUT_RECORD record;
    DWORD num_read;
    switch (WaitForSingleObject(hstdin_, 100)) 
    {
      case WAIT_OBJECT_0:
        if (!ReadConsoleInput(hstdin_, &record, 1, &num_read)) throw std::runtime_error("Read failed");
        if (record.EventType == KEY_EVENT && record.Event.KeyEvent.bKeyDown) 
        {
          auto vk = record.Event.KeyEvent.wVirtualKeyCode;
          if (vk == VK_SPACE) c = KEYCODE_SPACE;
          else if (vk == 0x51) c = KEYCODE_Q; // 'Q'
          else if (vk == 0x51 + 32) c = KEYCODE_q; // 'q'
        }
        break;
      case WAIT_TIMEOUT:
        break;
    }
    #else
      int rc = read(0, &c, 1);
      if (rc < 0) throw std::runtime_error("read failed");
    #endif
      return c;
  }

  ~KeyboardReader() 
  {
    #ifdef _WIN32
      SetConsoleMode(hstdin_, old_mode_);
    #else
      tcsetattr(0, TCSANOW, &cooked_);
    #endif
  }

private:
  #ifdef _WIN32
    HANDLE hstdin_;
    DWORD old_mode_;
  #else
    struct termios cooked_;
  #endif
};

class TeleopURKey final 
{
public:
  TeleopURKey() : node_(std::make_shared<rclcpp::Node>("teleop_ur_key")), input_()
  {
    // Parameters
    node_->declare_parameter<std::string>("key_topic", "/teleop/last_key");

    rclcpp::QoS qos(1);    // QoS: keep last message for late-joiners
    qos.reliable().transient_local();

    std::string topic = node_->get_parameter("key_topic").as_string();
    key_pub_ = node_->create_publisher<std_msgs::msg::Char>(topic, qos);

    // Initial state
    publish_key(' ');

    std::thread{[this](){ rclcpp::spin(node_); }}.detach();
  }

  int keyLoop() 
  {
    puts("---------------------------");
    puts("Control the Robot!");
    puts("---------------------------");
    puts("x or X:         START Alignment");
    puts("Spacebar:       STOP Alignment");
    puts("Esc or q or Q:  Quit");

    while (running.load()) 
    {
      char c = 0;
      try { c = input_.readOne(); }
      catch (const std::runtime_error &) { perror("read():"); return -1; }

      switch (c) 
      {
        // Ignore empty polls
        if (c == 0) break;

        // x or X => start (send_velocities=true)
        case KEYCODE_x:
        case KEYCODE_X:
          publish_key('X');
          break;

        // SPACE => stop (send_velocities=false)
        case KEYCODE_SPACE:
          publish_key(' ');
          break;

        // q / Q / ESC => quit: publish once, then exit this process
        case KEYCODE_q:
        case KEYCODE_Q:
        case 0x1B: // Esc
          publish_key(c == 0x1B ? 0x1B : 'Q');  // normalize ok either way
          rclcpp::sleep_for(std::chrono::seconds(1));
          running.store(false);
          break;

        default:
          // ignore any other keys
          break;
      }
    }
    return 0;
  }

private:
  void publish_key(char k) 
  {
    last_key_ = k;
    std_msgs::msg::Char m;
    m.data = static_cast<uint8_t>(last_key_);
    key_pub_->publish(m);
    RCLCPP_INFO(node_->get_logger(), "last_key = 0x%02X ('%c')", (int)last_key_, (last_key_ ? last_key_ : ' '));
  }


  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr key_pub_;
  KeyboardReader input_;
  char last_key_= ' '; 

};

#ifdef _WIN32
BOOL WINAPI quitHandler(DWORD) { running.store(false); return TRUE; }
#else
void quitHandler(int) { running.store(false); }
#endif

int main(int argc, char ** argv) 
{
  rclcpp::init(argc, argv);
  #ifdef _WIN32
    SetConsoleCtrlHandler(quitHandler, TRUE);
  #else
    signal(SIGINT, quitHandler);
  #endif
    TeleopURKey teleop;
    int rc = teleop.keyLoop();
    rclcpp::shutdown();
    return rc;
}
