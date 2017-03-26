#include "arduino_motors.h"

#include <fcntl.h>
#include <unistd.h>
#include <asm/termios.h>

namespace term_nm {
#include <sys/ioctl.h>
}  // term_nm

namespace sailbot {

namespace {

void WriteServoVal(int fd, char type, int val) {
  val = std::min(179, std::max(1, val));
  char buf[5];
  int i = 0;
  buf[i++] = type;
  if (val >= 100) {
    buf[i++] = '1';
    val -= 100;
  }
  buf[i++] = val / 10;
  buf[i++] = val % 10;
  buf[i++] = '\n';

  write(fd, buf, i);
}

bool IsDigit(char c) {
  return c >= '0' && c <= '9';
}

constexpr int kMinPot = 34;
constexpr int kMaxPot = kMinPot + 100 * 7;
float WinchPotToAngle(float pot_val) {
  constexpr int kPotRange = kMaxPot - kMinPot;
  return (1. - (pot_val - kMinPot) / kPotRange) * M_PI / 2;
}

}  // namespace

ArduinoMotors::ArduinoMotors(const char *port)
   : Node(0.0), port_name_(port), state_queue_("internal_state", true),
     state_msg_(AllocateMessage<msg::InternalBoatState>()) {
  Init();
  RegisterHandler<msg::SailCmd>("sail_cmd", [this](const msg::SailCmd &cmd) {
    float volts = cmd.voltage();
    // Don't allow it to run amok outside of bounds
    {
      std::unique_lock<std::mutex> lck(state_msg_mut_);
      float pos = state_msg_->sail();
      if (pos > M_PI / 2. - 0.1) {
        volts = std::min(volts, (float)0.);
      } else if (pos < 0.1) {
        volts = std::max(volts, (float)0.);
      }
    }
    int raw_val = volts / 12. * 90. + 90;

    raw_winch_ = raw_val;
  });
  RegisterHandler<msg::RudderCmd>("rudder_cmd", [this](const msg::RudderCmd &cmd) {
    int raw_val = cmd.pos() / M_PI * 180. + 90;
    {
      std::unique_lock<std::mutex> lck(state_msg_mut_);
      state_msg_->set_rudder(cmd.pos());
    }

    raw_rudder_ = raw_val;
  });
}

void ArduinoMotors::Iterate() {
  char start_char = '\0';
  while (start_char != 'a') {
    PLOG_IF(ERROR, read(fd_, &start_char, 1) == -1) << "Read of start byte failed";
    if (util::IsShutdown()) return;
  }

  // All right. We now ignore any nondigits and read until hitting the end of
  // the digits

  int val = 0;
  bool started = false;
  char digit = '\0';
  while (true) {
    PLOG_IF(ERROR, read(fd_, &digit, 1) == -1) << "Read failed";
    if (IsDigit(digit)) {
      started = true;
      val *= 10;
      val += digit - '0';
    } else if (started) {
      break; // We have finished the actual digits.
    }
  }
  VLOG(2) << "Analog in: " << val << " to winch: " << WinchPotToAngle(val);

  {
    std::unique_lock<std::mutex> lck(state_msg_mut_);
    state_msg_->set_sail(WinchPotToAngle(val)); // TODO(james): Add sign
    state_queue_.send(state_msg_);
  }

  WriteServoVal(fd_, 'w'/*winch*/, raw_winch_);
  WriteServoVal(fd_, 'r', raw_rudder_);
}

void ArduinoMotors::Init() {
  // Set Baud rate to 100000: Lots of complicate
  fd_ = open(port_name_, O_RDWR); // change!
  PCHECK(fd_ != -1) << "Error opening serial port " << port_name_;

  termios2 tio;
  PCHECK(term_nm::ioctl(fd_, TCGETS2, &tio) == 0)
      << "Error getting ioctl for serial port";

  tio.c_cflag &= ~(CBAUD | PARODD | CSIZE | CSTOPB | PARENB);
  tio.c_cflag |= BOTHER /*| CSTOPB | PARENB */| CS8;
  // Not sure these flags actually do anything useful, but:
  // IGNPAR: Ignore parity/stop errors; probably not an issue when
  //   I geneerally turn off parity anyways.
  // BRKINT: Something abotu breaks in the input?
  tio.c_iflag &= ~(IXON);// | IGNPAR;
  tio.c_iflag |= BRKINT;// | IGNPAR;
  // None of the output flags are useful; in particular, ONLCR will
  // map NL to CR-NL, which geerates excessive new lines and is irritating.
  tio.c_oflag = 0;
  // These are far more critical (although the biggest issue lies in ECHO).
  // Does magic stuff to make things work.
  tio.c_lflag &= ~(ISIG | ICANON | ECHO);
  // Not strictly necessary, but helpful:
  // When read() is called without ICANON set, its behavior depends on these
  // settings:
  // MIN = TIME = 0: read() returns immediately with the maximum number of
  //   bytes available, up to what was requested.
  // MIN > 0, TIME = 0: read() returns once either MIN or the requested number
  //   of bytes are available.
  // MIN = 0, TIME > 0: TIME = timer in tenths of second; read() returns either
  //   when one or more bytes are available, or when the timer expires.
  // MIN, TIME > 0: TIME is timer between individual bytes of data; once either
  //   MIN bytes have been read, the request is fulfilled, or too much TIME
  //   passes between receiving bytes, read() will return.
  tio.c_cc[VMIN] = 100;
  tio.c_cc[VTIME] = 10;

  // Baud rates to set.
  tio.c_ispeed = kBaudRate;
  tio.c_ospeed = kBaudRate;

  PCHECK(term_nm::ioctl(fd_, TCSETS2, &tio) == 0) << "Error setting baud rate";

  // Also, default initialize message fields.
  state_msg_->set_sail(0);
  state_msg_->set_rudder(0);
}

}  // namespace sailbot
