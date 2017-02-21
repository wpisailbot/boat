#pragma once

#include "util/node.h"
#include "rigid_wing/rigid_wing.pb.h"

namespace sailbot {

class RigidWing : public Node {
 public:
  RigidWing();
  ~RigidWing() {
    if (conn_fd_ != -1) {
      close(conn_fd_);
    }
    if (sock_fd_ != -1) {
      close(sock_fd_);
    }
  }

  /**
   * @param cmd The message to put out into the buffer
   * @param buf Buffer to write message into
   * @param buf_len Length of the buffer
   * @return Bytes written to buffer
   */
  static int ConstructMessage(const msg::RigidWingCmd &cmd, char *buf,
                              size_t buf_len);
  /**
   * @param buf The buffer to read from
   * @param buf_len Length of said buffer
   * @param feedback The message to fill from the buffer
   *   In feedback, clears fields that failed to parse properly
   */
  static void ParseMessage(const char *buf, size_t buf_len,
                           msg::RigidWingFeedback *feedback);

 private:
  void Iterate() override;
  void ConstructAndSend(const msg::RigidWingCmd &cmd);

  // Handle one-time initialization of socket (for constructor)
  void InitSocket();
  // Handle accepting connection from rigid wing
  // Returns true if accepted a socket, false if error/timeout
  bool AcceptConnection();
  // Wait for change in FD read state using select. Returns return value of
  // select.
  static int WaitForChange(int fd, struct timeval tv);

  ProtoQueue<msg::RigidWingFeedback> feedback_queue_;
  msg::RigidWingFeedback *feedback_msg_;
  std::atomic<int> sock_fd_; // The fd returned from socket()
  std::atomic<int> conn_fd_; // The fd returned from accept()
};

}  // namespace sailbot
