#include "util/log_deprecation.h"
#include "util/logger.h"

namespace sailbot {
namespace deprecation {

void HandleOldPos(msg::LogEntry* entry, char *buf, int *n) {
  if (!entry->has_boat_state()) {
    // If this queue doesn't even have the boat state queue, it shouldn't be
    // called
    return;
  }
  if (!entry->boat_state().has_pos() && entry->boat_state().has_pos_float()) {
    // We must copy pos_float over to pos.
    msg::Vector3d *pos = entry->mutable_boat_state()->mutable_pos();
    const msg::Vector3f &pos_float = entry->boat_state().pos_float();
    pos->set_x(pos_float.x());
    pos->set_y(pos_float.y());
    pos->set_z(pos_float.z());
    if (entry->ByteSize() > Logger::MAX_BUF) {
      // We just made the message too big to serialize
      return;
    }
    *n = entry->ByteSize();
    entry->SerializeToArray(buf, Logger::MAX_BUF);
  }
}

}  // namespace deprecation
}  // namespace sailbot
