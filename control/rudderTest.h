
//
// Created by Sierra Palmer and Sydney Fisher on 1/28/19.
//

//#pragma once
#include "util/node.h"
#include "control/actuator_cmd.pb.h"
#include "rigid_wing/rigid_wing.pb.h"
#include <mutex>

//class SimpleControl;

//#ifndef BOAT_RUDDERTEST_H
//#define BOAT_RUDDERTEST_H

//#endif //BOAT_RUDDERTEST_H

namespace sailbot {
    namespace control {
        class RudderTest : public Node {
        public:
            RudderTest(bool do_rudder=false);
            void simpleRudder(bool do_rudder = true);
            const bool rudderNew = true;
	    
        private:
            const bool do_rudder_;
            msg::RudderCmd *rudder_msg_;
            ProtoQueue<msg::RudderCmd> rudder_cmd_;
	    msg::ControllerConstants *consts_msg_;
        };
    }
}
