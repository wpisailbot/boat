//
// Created by Sierra Palmer on 1/28/19.
//

#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "rigid_wing/rigid_wing.pb.h"
#include <mutex>

#ifndef BOAT_RUDDERTEST_H
#define BOAT_RUDDERTEST_H

#endif //BOAT_RUDDERTEST_H

namespace sailbot {
    namespace control {
        class RudderTest : public Node {
        public:
            void simpleRudder();
        };
    }
}
