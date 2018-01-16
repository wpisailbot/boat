#include "util/msg.pb.h"

namespace sailbot {
namespace deprecation {

// Checks a LogEntry proto to see if it is a boat_state message
// with an old (single-preciison floating point) position vector
// and updates it if so, to handle pre-Jan 2018 logs.
void HandleOldPos(msg::LogEntry* entry, char *buf, int *n);

}  // namespace deprecation
}  // namespace sailbot
