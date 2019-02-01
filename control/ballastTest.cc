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
