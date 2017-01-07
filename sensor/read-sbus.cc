#include "read-sbus.h"
#include "sensor/sbus.pb.h"

#include <fcntl.h>
#include <unistd.h>
#include <asm/termios.h>

namespace term_nm {
#include <sys/ioctl.h>
}  // term_nm

namespace sailbot {

void ReadSBUS::Init() {
  VLOG(3) << "Initializing";
  // Set Baud rate to 100000: Lots of complicate
  fd = open(port_name_, O_RDONLY); // change!
  PCHECK(fd != -1) << "Error opening serial port " << port_name_;

  termios2 tio;
  PCHECK(term_nm::ioctl(fd, TCGETS2, &tio) == 0)
      << "Error getting ioctl for serial port";

  tio.c_cflag &= ~(CBAUD | PARODD | CSIZE);
  tio.c_cflag |= BOTHER | CSTOPB | PARENB | CS8;
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
  tio.c_cc[VTIME] = 0;

  // Baud rates to set.
  tio.c_ispeed = kBaudRate;
  tio.c_ospeed = kBaudRate;

  PCHECK(term_nm::ioctl(fd, TCSETS2, &tio) == 0) << "Error setting baud rate";

  // Also, set up message fields.
  for (int i = 0; i < 16; ++i) {
    msg_->add_channel(0);
  }
  msg_->set_ch17(false);
  msg_->set_ch18(false);
  msg_->set_frame_lost(false);
  msg_->set_failsafe(false);
}

void ReadSBUS::Iterate() {
  char start_byte = 0x0;
  /*
  while (!util::IsShutdown()) {
    PCHECK(read(fd, &start_byte, 1) != -1) << "Read of start byte failed";
    VLOG(2) << "Byte: " << (int)start_byte;
  }
  return;
  */
  // Figure out which the start byte really is.
  while (start_byte != 0x0F) {
    PCHECK(read(fd, &start_byte, 1) != -1) << "Read of start byte failed";
    //PCHECK(-1 != write(fd, &start_byte, 1)) << "Write failed";
    VLOG(2) << "Checking start byte: " << (int)start_byte;
    if (util::IsShutdown()) return;
  }

  // Alright, we have the start byte. Now, read 22 bytes of data, where every 11
  // bits are for each channel, then the flags byte, and the end byte.
  uint8_t data[24];
  int n = read(fd, data, 24);
  PCHECK(n != -1) << "Read of data failed";
  if (n < 24) {
    LOG(ERROR) << "Didn't read enough data...";
    return;
  }
  size_t cur_bit = 0;
  for (int i = 0; i < 16; ++i) {
    size_t cur_byte = cur_bit / 8;
    size_t start_bit = cur_bit % 8;
    size_t end_bit = (cur_bit + 11) % 8;
    int bits = ((data[cur_byte] >> start_bit) & 0xFF); // Lowest order bits
    ++cur_byte;
    if (end_bit < 3) {
      bits += (int)data[cur_byte] << (8-start_bit); // Mid order bits
      ++cur_byte;
    }
    if (end_bit) {
      bits += ((((int)data[cur_byte]) << (8 - end_bit)) & 0xFF)
              << 3;
    }
    msg_->set_channel(i, bits);
    cur_bit += 11;
  }
  uint8_t flags = data[22];
  msg_->set_ch17(0x80 & flags);
  msg_->set_ch18(0x40 & flags);
  msg_->set_frame_lost(0x20 & flags);
  msg_->set_failsafe(0x10 & flags);
  queue_.send(msg_);
#define PB(i) (int)msg_->channel(i) << ", "
  VLOG(2) << "Channels: " << PB(0) << PB(1) << PB(2) << PB(3) << PB(4) << PB(5)
          << PB(6) << PB(7);
#undef PB
}

}  // sailbot
