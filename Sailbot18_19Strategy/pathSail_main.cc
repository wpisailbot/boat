//
// Created by Sierra Palmer on 1/14/19.
//

#include "pathSail.h"

int main(int argc, char* argv){
    mapShort mS;
    mapLong mL;
    point s, e(7,7);
    aStar as;

    // Start out by converting the start point of the boat to meters
    startPointLatLongConvert();

    // Set the buoy goal, and then set the goal point's x and y values
    // as the end points
    buoyGoal();
    goalPoint[0][0] = end.x;
    goalPoint[0][1] = end.y;

    // While the path has not ended, convert the current position into meters
    // and then commence a search depending on the heading of the boat.
    while(path != end){
        latLongConvert();
        if(heading == north || south) {
            as.search(s, e, mL);
        } else{
            as.search(s,e,mS);
        }
    }
    return 0;

}