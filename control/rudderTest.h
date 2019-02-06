//
// Created by Sierra Palmer on 1/28/19.
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
        class RudderTest {
                //: public Node {
        public:
            void SimpleControl(bool do_rudder=false);
            void simpleRudder(bool do_rudderTest = true);
            const static bool rudderNew = true;

            RudderTest(bool whetherTrue, rudder_msg_(AllocateMessage<msg::RudderCmd()) newMessage, rudder_cmd_("rudder_cmd", true))= newCommand){
                rudderNew = whetherTrue;
                *rudder_msg_ = newMessage;
                rudder_cmd_ = newCommand;
            }

        private:
            msg::RudderCmd *rudder_msg_;
            ProtoQueue<msg::RudderCmd> rudder_cmd_;
        };
    }
}
