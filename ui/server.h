#pragma once

#include "util/node.h"
#include "uWS.h"
#include <mutex>
#include <map>
#include <string>

namespace sailbot {

class WebSocketServer : public Node {
 /**
  * Handles the UI. Also tells us whether the UI is connected
  */
 public:
  WebSocketServer(int port=13000);
  ~WebSocketServer();

 private:
  void RegisterQueueHandler(const std::string name);
  void ProcessQueue(const std::string name);
  void ProcessSocket(uWS::WebSocket<uWS::SERVER> *ws, char *message,
                     size_t length, uWS::OpCode opcode);
  std::string GetCurrentVal(const std::string &msg);
  void Iterate() override;

  uWS::Hub hub_;
  const int port_;
  std::mutex map_mutex_;
  std::map<std::string, const msg::LogEntry*> msgs_;
  std::mutex send_map_mutex_;
  std::map<std::string, std::unique_ptr<Queue>> send_msgs_;
  std::vector<std::thread> threads_;
  // Mutex so that we don't destroy the data that ProcessSocket needs too early
  std::mutex process_mutex_;
  // The last time that we processed a socket request
  // TODO(james): Initialize apopriately
  std::mutex last_conn_mut_;
  util::monotonic_clock::time_point last_conn_;
  ::sailbot::msg::ConnectionStatus *conn_status_;
  ProtoQueue<msg::ConnectionStatus> conn_queue_;
};

}  // namespace sailbot
