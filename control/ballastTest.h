
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
            BallastTest();
            void simpleBallast();

        private:
            const bool ballast;
            msg::
        };
    }
}

// class RudderTest : public Node {
//        public:
//            RudderTest(bool do_rudder=false);
//            void simpleRudder(bool do_rudder = true);
//            const bool rudderNew = true;
//
//        private:
//            const bool do_rudder_;
//            msg::RudderCmd *rudder_msg_;
//            ProtoQueue<msg::RudderCmd> rudder_cmd_;
//	        msg::ControllerConstants *consts_msg_;
//        };
