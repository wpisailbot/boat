//
// Created by Sydney Fisher on 2/1/19.
//

// A lot of this code is pulled from simple.cc in order to get the rudders to run correctly and autonomously

#include "simple.h"
#include "ballastTest.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "control/util.h"

namespace sailbot {
namespace control{

    BallastTest::BallastTest(bool ballast)
            : Node(0.01),

    void BallastTest::simpleBallast(){
            if (ballast){
                control::SimpleControl::msg::BallastCmd::ballast_msg_->set_pos(0.75);
                control::SimpleControl::msg::BallastCmd::ballast_cmd_.send(ballast_msg_);
            }
            else {
                msg::BallastCmd::ballast_msg_->set_pos(0);
        }

    }
} //control
} //sailbot

//RudderTest::RudderTest(bool do_rudder)
//        : Node(0.01),
//          do_rudder_(do_rudder),
//          rudder_msg_(AllocateMessage<msg::RudderCmd>()),
//          rudder_cmd_("rudder_cmd", true),
//	  consts_msg_(AllocateMessage<msg::ControllerConstants>())
//                                                                   {
//
//     consts_msg_->set_max_rudder(0.5);
//     consts_msg_->set_rudder_kp(0.5);
//     consts_msg_->set_rudder_ki(0.02);
//
//}
//
//    void RudderTest::simpleRudder(bool rudder){
//    //,  rudder_msg_(AllocateMessage<msg::RudderCmd>()), rudder_cmd_("rudder_cmd", true)
//        if (RudderTest::rudderNew){
//
//
//            rudder_msg_->set_pos(0.75); // error with needing an initializer
//            //sailbot::ProtoQueue<msg::RudderCmd>
//            rudder_cmd_.send(rudder_msg_);
//        }
//        else {
//            //sailbot::msg::RudderCmd
//            rudder_msg_->set_pos(0);
//        }
//    }