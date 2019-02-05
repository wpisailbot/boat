//
// Created by Sierra Palmer on 1/28/19.
//

#include "simple.h"
#include "rudderTest.h"

int main(int argc, char *argv[]){
    sailbot::util::SetCurrentThreadRealtimePriority(10);
    sailbot::util::Init(argc, argv);

    sailbot::control::RudderTest::simpleRudder(argc,argv);
}

