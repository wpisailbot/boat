
// Created by Sydney Fisher on 2/1/19.
//

#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "rigid_wing/rigid_wing.pb.h"
#include <mutex>

#ifndef BOAT_BALLASTTEST_H
#define BOAT_BALLASTTEST_H

#endif //BOAT_BALLASTTEST_H

namespace sailbot {
    namespace control {
        class BallastTest : public Node {
        public:
            void simpleBallast();

        private:
            const bool ballast = true;
        };
    }
}
