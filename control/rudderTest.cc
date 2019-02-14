
//
// Created by Sierra Palmer and Sydney Fisher on 1/28/19. 
//

// A lot of this code is pulled from simple.cc in order to get the rudders to run correctly and autonomously

#include "simple.h"
#include "rudderTest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "control/util.h"
#include <iostream>

namespace sailbot {
namespace control {

     RudderTest::RudderTest(bool do_rudder)
        : Node(0.01),
          do_rudder_(do_rudder),
          rudder_msg_(AllocateMessage<msg::RudderCmd>()),
          rudder_cmd_("rudder_cmd", true),
	  consts_msg_(AllocateMessage<msg::ControllerConstants>())
                                                                   {
          
     //consts_msg_->set_max_rudder(0.5);
     //consts_msg_->set_rudder_kp(0.5);
     //consts_msg_->set_rudder_ki(0.02);
     
}
    
  void RudderTest::Iterate() {
    counter += 0.001;
    if (counter > 1.0) {
      counter = 0.0;
    }
    rudder_msg_->set_pos(counter);
    rudder_cmd_.send(rudder_msg_);
    LOG(INFO) << "Send rudder command:" << rudder_msg_->ShortDebugString();
  }

} //control
} //sailbot
