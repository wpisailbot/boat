//
// Created by Sierra Palmer on 1/28/19.
//

// A lot of this code is pulled from simple.cc in order to get the rudders to run correctly and autonomously

#include "simple.h"
#include "rudderTest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "control/util.h"

namespace sailbot {
namespace control{

    void RudderTest::simpleRudder(){
            if (rudder){
                control::msg::RudderCmd->rudder_msg_->set_pos(0.75);
                control::msg::rudder_cmd_.send(rudder_msg_);
            }
            else {
                control::msg::RudderCmd->rudder_msg_->set_pos(0);
        }

    }
} //control
} //sailbot
