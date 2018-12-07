//
// This was created to start the development process for sailing along a path.
// This will be further integrated into the rest of the events in the future, but
// this is just to get ideas down and get started on thinking of the logic.
//
// Created by Sierra Palmer on 12/4/18.
//

// 1. At the start point, take in GPS location, and set it as the start point

// 2. Check the latitude and longitude data in order to determine how close to land to get
//  2a. Set the latitude/longitude line that's as close to land we want to get as an obstacle line

// 3. Set a waypoint within 10 yards starboard of the buoy (need to keep track of lane lengths, esp. for endurance)
//  3a. Set it so that there's a range about the waypoint that allows for it to not have to sail directly through the point
//  3b. Once the robot is within buoy detection range, the camera takes over

// 4. The boat will need to sail following along certain heuristics
//  4a. Wind direction
//      4a1. Use the airmar for wind direction
//      4a2. Set the waypoint for the buoy so that we don't have to sail into the wind to hit it
//  4b. Lane width navigation to sail through
//      4b1. Once the start point is determined, add "x" degrees of latitude/longitude so that there are 6-8 meters of
//            sailable space on either side of the boat
//      4b2. Set up the right edge of the lane to cost less so that the boat "right wall follows", or will tend to sail
//            towards the right in order to be more in line with the waypoint on the starboard side of the buoy
//  4c. Distance to waypoint
//      4c. Use distance formula to find the straight line distance to the waypoint
//

float waypoint[1][3];
float startLocation[1][3];

double longSideDist = 40;
double shortSideDist = 20;




