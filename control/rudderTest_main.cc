//
// Created by Sierra Palmer and Sydney Fisher on 1/28/19.
//

#include "simple.h"
#include "rudderTest.h"
//package RudderTest;
#include <iostream>

int main(int argc, char *argv[]){
    sailbot::util::SetCurrentThreadRealtimePriority(10);
    sailbot::util::Init(argc, argv);
    
    sailbot::control::RudderTest rt(true);
    const bool rudderTest = true;
    rt.Run();
}//

