#include "fake_internal_sensors.h"

namespace sailbot {
namespace sim {

FakeInternalSensors::FakeInternalSensors()
    : Node(1), output_queue_("internal_state", true) {
  RegisterHandler<msg::BoatState>("sim_true_boat_state",
                                  [this](const msg::BoatState &msg) {
    if (msg.has_internal()) {
      output_queue_.send(&msg.internal());
    }
  });
}

}  // namespace sim
}  // namespace sailbot
