#include "search.h"

void sailToBuoy(){
  while (buoy == true){
    if (atBuoy() == false){
      sailForward();
      buoy = seeBuoy();
    }
    else{
      signal();
    }
  }
  while (buoy == false){
    buoy = findBuoy();
  }
}

bool searchPattern(){
  // set waypoint gate 100m away
  // check wind direction
  // sail leftsided close hauled 
  // once reaches 100m waypoint, tack
}

void sailForward(){
  // lane following algorithm
}

bool seeBuoy(){
  // camera stuff
}

bool findBuoy(){
  // check wind direction
  // make sure by turning the boat will not turn into Irons
  // turn boat towards buoy
  // check if you see buoy
  // if see buoy then return true, otherwise return false
}

bool atBuoy(){
  //return true if at buoy (camera stuff)
  // otherwise return false
}

void signal(){
  // turn on white strobe light or signal to shore station
  // check wind direction
  // turn into irons
}
        
