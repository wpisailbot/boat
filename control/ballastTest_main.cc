//
// Created by Sydney Fisher on 2/1/19.
//

#include "simple.h"
#include "ballastTest.h"

int main(int argc, char *argv[]){
    sailbot::util::SetCurrentThreadRealtimePriority(10);
    sailbot::util::Init(argc, argv);

    sailbot::control::BallastTest bt;
    bt.Run();
}

//sailbot::util::SetCurrentThreadRealtimePriority(10);
//    sailbot::util::Init(argc, argv);
//
//    sailbot::control::RudderTest rt(true);
//    const bool rudderTest = true;
//    rt.Run();
