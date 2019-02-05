//
// Created by Sierra Palmer on 1/28/19.
//

#include "simple.h"
#include "rudderTest.h"
//package RudderTest;

int main(int argc, char *argv[]){
    //RudderTest rt;
    const bool rudderTest = true;

    sailbot::util::SetCurrentThreadRealtimePriority(10);
    sailbot::util::Init(argc, argv);

    sailbot::control::RudderTest::simpleRudder(rudderTest);
}

