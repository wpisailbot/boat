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
namespace control {

     void RudderTest::SimpleControlTest(bool do_rudderTest,: Node(0.01),
          do_rudderTest_(do_rudderTest),
          rudder_msg_(AllocateMessage<msg::RudderCmd>()),
          rudder_cmd_("rudder_cmd", true) {
          
     consts_msg_->set_max_rudder(0.5);
     consts_msg_->set_rudder_kp(0.5);
     consts_msg_->set_rudder_ki(0.02);
     
     }
    
    void RudderTest::simpleRudder(bool rudder = true, rudder_msg_(AllocateMessage<msg::RudderCmd()) = message, rudder_cmd_("rudder_cmd", true) = command){
        if (rudder){

            sailbot::msg::RudderCmd  message->set_pos(0.75);

            //rudder_msg_->set_pos(0.75); // error with needing an initializer
            sailbot::ProtoQueue<msg::RudderCmd> command.send(rudder_msg_);
        }
        else {
            sailbot::msg::RudderCmd message->set_pos(0);
        }
    } 
} //control
} //sailbot

