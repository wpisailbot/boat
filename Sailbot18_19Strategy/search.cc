/* Search
*
* As taken from the sailbot 2018 events website:
* Challenge goal: To demonstrate a boat's ability to autonomously locate an
* object
*
* Description: An orange buoy will be placed somewhere within 100 m of a reference
* position. The boat must locate and signal(!) such within five minutes of entering
* the search area. RC is not allowed after entering the search area. (! Signal means
* 1) white strobe on boat and/or signal to a shore station and 2) either turn into 
* wind or assume station-keeping mode)
*
* This code was written based on the Depth First Search (DFS) algorithm page on Wikipedia
*
* This code was written by Sydney Fisher and Sierra Palmer as part of the WPI 
* 2018-2019 Sailbot team to accomplish this task
*/


double time = 0;
bool buoyFound = false;
double time = 300  // number of seconds in 5mins
double area = 100  // number of meters in the search area
  
while(time < 300){
  
  if (buoyFound == true){
    sailToBuoy();
  }
  else {
    buoyFound = searchPattern();
  }
  
  time++; // don't think this is how timer works, will have to figure out timing on boat
}

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
        
