//
// Created by Sierra Palmer on 1/28/19.
//

#include "simple.h"
#include "rudderTest.h"

int main(int argc, char *argv[]){
    sailbot::util::SetCurrentThreadRealtimePriority(10);
    sailbot::util::Init(argc, argv);

    void simpleRudder(){
        if (rudder){
            sailbot::msg::RudderCmd rudder_msg_->set_pos(0.75); // error with needing an initializer
            sailbot::ProtoQueue<msg::RudderCmd> rudder_cmd_.send(rudder_msg_);
        }
        else {
            sailbot::msg::RudderCmd rudder_msg_->set_pos(0);
        }

    }
}