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

#include "search.h"


int main(){
  while(time < 300){
  
  if (buoyFound == true){
    sailToBuoy();
  }
  else {
    buoyFound = searchPattern();
  }
  
  time++; // don't think this is how timer works, will have to figure out timing on boat
  }
}
