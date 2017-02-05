#include "server.h"

int main(int argc, char *argv[]) {
  sailbot::util::Init(argc, argv);
  sailbot::WebSocketServer server;
  server.Run();
}
