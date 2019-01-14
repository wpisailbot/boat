//
// This was created to start the development process for sailing along a path.
// This will be further integrated into the rest of the events in the future, but
// this is just to get ideas down and get started on thinking of the logic.
//
// Created by Sierra Palmer on 12/4/18.
//

// Based on notes from RBE3002 taught by Michalson:
// in order to plan a path from init to goal, there are four fundamental questions:
//      1. How is a plan represented?
//      2. How is a plan computed?
//      3. What does the plan achieve?
//      4. How do we evaluate a plan's quality?

// Maybe consider quadtree since we don't have many obstacles to encounter, and could speed up processing time

// f(n) = g(n) + h(n) where
//      g(n) is the cost so far to reach the node
//      h(n) is the estimated cost from n to goal
//      f(n) is the estimated total cost through n to goal

// 1. At the start point, take in GPS location, and set it as the start point

// 2. Check the latitude and longitude data in order to determine how close to land to get
//  2a. Set the latitude/longitude line that's as close to land we want to get as an obstacle line

// 3. Set a waypoint within 10 yards starboard of the buoy (need to keep track of lane lengths, esp. for endurance)
//  3a. Set it so that there's a range about the waypoint that allows for it to not have to sail directly through the point
//  3b. Once the robot is within buoy detection range, the camera takes over

// 4. The boat will need to sail following along certain heuristics (creates h(n))
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

// Needed measurements:
//      - each degree of latitude is approximately 69 miles (111 km) [each 60th of a degree is a nautical mile]
//      - each degree of longitude is cosine(latitude in decimal degrees) * length of degree (miles) at equator

// Long side of the rectangular path is going to be about 40 m
// Short side of the rectangular path is going to be about 20 m

// The sailing lane should be 12 m wide, with 6 m on either side of the starting point of the boat
// Therefore, each grid square should be 2 meters squared

#include <iostream>
#include <string>
#include <cmath>
#include <list>
#include <algorithm>
#include "pathSail.h"

using namespace std;

long goalPoint[1][2];
std::string startLocation[1][2]; // [latitude, longitude]
std::string currentLocation[1][2]; // [latitude, longitude]
std::string currentWindDirection;  // considering it to be a vector(?)
                                    // OR should it be an angle?
long startLocationMeters[1][2]; // converted to meters, where latitude becomes x, and longitude becomes y
long currentLocationMeters[1][2]; // converted to meters, where latitude becomes x, and longitude becomes y

int degreeLatDistMeters = 111000;
int degreesToRadians = 0174533;
long equatorDegrees = 69.172;

long longLaneDist = 40;
long shortLaneDist = 20;
long distToGoal;
long laneCost;
long windCost;
long n; // using this as "node" for right now
long totalCost;

long astarShort::laneCost_Creation(){
        long rightBound; // Right bound of the lane
        long leftBound; // Left bound of the lane

        if(heading == north || south) {
            // Need to determine north and south, how heading is done, and the range for what is considered north and south
            rightBound = startLocationMeters[0][1] + 6; // Want the lane to be a total of 12 meters, with 6 meters on
            leftBound = startLocationMeters[0][1] - 6;  // either side
        }
        else{
            rightBound = startLocationMeters[0][0] + 6;
            leftBound = startLocationMeters[0][0] - 6;
        }

        if(n < leftBound || n > rightBound){
            laneCost = 10000000; // random high cost number so that it doesn't go outside of the lane
            return laneCost;
        }
        else{
            laneCost = 0;
            return laneCost;
        }
    }

    // Need to determine a cost for the distance to the goal based upon the current location and the location of the next
    // node "n" by using the distance equation
    void astarShort::distCost() {
        long xDist = goalPoint[0][0] - currentLocationMeters[0][0]; // calculate the difference in x's
        long yDist = goalPoint[0][1] - currentLocationMeters[0][1]; // calculate the difference in y's

        xDist = pow(xDist,2);
        yDist = pow(yDist,2);

        long radicandSum = xDist+yDist;

        distToGoal = sqrt(radicandSum);
    }

    // Need to determine the cost based upon the direction the wind is blowing assuming it's considered an angle
    long astarShort::calcWindCost(){
        long currentWindDirectionDegrees = std::stol(currentWindDirection, 10);
        if(currentWindDirectionDegrees < 280 && currentWindDirectionDegrees > 260){ // Assuming that 270 is directly into the boat
            windCost = 100000000; // Can't sail into irons, or we're stuck
            return windCost;
        }
        else if(currentWindDirectionDegrees < 100 && currentWindDirectionDegrees > 80){ // Assuming that 90 degrees is directly downwind
            windCost = 30; // Want to try and sail downwind whenever possible to cut down on the amount of tacking
            // Need to figure out what would be a good cost to make sure this does pretty well, but doesn't overwrite
            // any other option
            return windCost;
        } else{
            windCost = 200; // Another random number chosen. Just need to test in order to find a good cost to choose
            return windCost;
        }
    }

    long astarShort::costTotaled(){
        // calculates the total cost of a decision based on the lane and the wind direction
        totalCost = windCost + laneCost;
    }

    bool astarShort::pointValid(point& p ) {
        // Assume that the long map is utilized when the boat is pointing north or south
        if (heading == north || south){
            // check to see if the point is actually within the range of movement to one of
            // eight positions surrounding the current and within the long grid range
            return (p.x > -1 && p.y > -1 && p.x < mL.w && p.y < mL.h);
        } else {
            // check to see if the point is actually within the range of movement to one of
            // eight positions surrounding the current and within the short grid range
            return (p.x > -1 && p.y > -1 && p.x < mS.w && p.y < mS.h);
        }
    }

    bool astarShort::pointExist( point& p ){
        std::list<node>::iterator i;
        i = std::find( closed.begin(), closed.end(), p);
        // Check to see if the point is the end point by iterating through the list of
        // closed points. If not, then check to see if there's a
        // distance to be traveled to the end point to see if the point exists.
        if( i != closed.end()) {
            if(totalCost + distToGoal > totalCost){
                return true;
            }else{
                // If the point is the end point, erase the input point, and then return that the
                // point does not exist
                closed.erase(i);
                return false;
            }
        }
        i = std::find(open.begin(), open.end(), p);
        // Check to see if the point is the end point by iterating through the list of
        // open points. If not, check to see if there's a distance to be traveled to
        // the end point to see if the point exists.
        if(i != open.end() ){
            if(totalCost + distToGoal > totalCost){
                return true;
            }
            else{
                open.erase(i);
                return false;
            }
        }
    }

    bool astartShort::fillOpenNode( node& node ){
        // Set up each open node to contain all of the required info,
        // such as neighbor nodes, parent nodes, cost, and distance
        int stepCost, nc, dist;
        point neighbor;

        for( int x = 0; x <8; x++){
            // x = 0 is to the left, 1 is in front, 2 is to the right, 3 is behind,
            // 4 is "SW", 5 is "NW", 6 is "NE", and 7 is "SE"
            if (x == 5 || x == 1 || x == 6 |) {
                stepCost = 1;
                // The "quickest" way to get to the end goal would be able to move forward, as
                // it makes it easier for the robot to manuever in that way
            } else if (x == 0 || x == 2) {
                // Going directly to either side costs slightly more
                stepCost = 5;
            } else{
                // Going backwards costs a lot more because the robot would have to turn around
                // to access it
                stepCost = 100;
            }
            // creates a neighbor for the current point
            neighbor = n.pos + neighbor[x];
            if(neighbor == end){
                // If the neighbor is the endpoint, then return true because the
                // open node has already been filled
                return true;
            }
            // The open nodes have to be filled depending on the heading of the boat
            // to make sure that both the long map and the short map are filled
            if(heading == north || south){
                if(pointValid(neighbor) && mL(neighbor.x, neighbor.y)!=1){
                    nc = stepCost + n.cost;
                    dist = distToGoal;
                    if (!pointExist(neighbor, nc + dist)){
                        node mL;
                        mL.cost = nc;
                        mL.dist = dist;
                        mL.pos = neighbor;
                        mL.parent = n.pos;
                        open.push_back(mL);
                    }
                }
            } else if (heading == east || west){
                if(pointValid(neighbor) && mS(neighbor.x, neighbor.y)!=1){
                    nc = stepCost + n.cost;
                    dist = distToGoal;
                    if (!pointExist(neighbor, nc + dist)){
                        node mS;
                        mS.cost = nc;
                        mS.dist = dist;
                        mS.pos = neighbor;
                        mS.parent = n.pos;
                        open.push_back(mS);
                    }
                }

            } else{
                // If nothing else works, return false
                return false;
            }

        }
    }

    bool astarShort::searchLong(point& s, point& e, mapLong& mapL1){
            // Creates the search path while navigating the long map in order to
            // check open nodes, search them, and then close them
            node n; end = e; start = s; mapLong = mapL1;
            n.cost = 0; n.pos = s; n.parent = 0; n.dist = distToGoal(s);
            open.push_back(n);
            while( != open.empty()){
                node n = open.front();
                open.pop_front();
                closed.push_back(n);
                if(fillOpenNode(n)){
                    return true;
                }
            }
            return false;
    }

    bool astarShort::searchShort(point& s, point& e, mapShort& mapS1){
            // Creates the search path while navigating the short map in order to
            // check open nodes, search them, and then close them
            node n; end = e; start = s; mapShort = mapS1;
            n.cost = 0; n.pos = s; n.parent = 0; n.dist = distToGoal(s);
            open.push_back(n);
            while( != open.empty()){
                node n = open.front();
                open.pop_front();
                closed.push_back(n);
                if(fillOpenNode(n)){
                    return true;
                }
            }
            return false;
    }

    int astarShort::path( std::list<point>& path){
            // Iterate through the list of nodes to find the cost of each of the
            // paths by going adding up the costs
            path.push_front(end);
            int cost = 1 + closed.back().cost;
            path.push_front(closed.back().pos);
            point parent = closed.back().parent;

            for(std::list<node>::reverse_iterator i = closed.rbegind(); i != closed.rend(); i++){
                if((i*).pos == parent && !((*i).pos == start)){
                    path.push_front((*i).pos);
                    parent = (*i).parent;
                }
            }
            path.push_front(start);
            return cost;
    }


// Converts the current point from degrees, minutes, seconds to meters to use in calculations
void pathSail::latLongConvert(){
    std::string degrees;
    std::string minutes;
    std::string seconds;
    long longitude;
    // Converts the latitude and longitude to meters to help determine distance to goal for
    // determining h(n)
    // This is all assuming that all coordinates are received as "degrees.minutes.seconds"
    for (unsigned int i = 0; i < 2; i++;){
        for (unsigned int j = 0; j < currentLocation[0][i].length(); j++;){
            //iterate through the string to split up degrees, minutes, and seconds
        }
        long degreesLong = std::stol(degrees, 10);
        long minutesLong = std::stol(minutes, 10);
        long secondsLong = std::stol(seconds, 10);

        minutesLong = minutesLong / 60;
        secondsLong = secondsLong / 3600;

        long decimalDegrees = degreesLong + minutesLong + secondsLong;

        if (i = 0) {
            currentLocationMeters[0][i] = decimalDegrees / degreeLatDistMeters;
        } else {
            latRads = decimalDegrees * degreesToRadians;
            longitude = cosine(latRads) * equatorDegrees;
            currentLocation[0][i] = decimalDegrees / longitude;
        }
    }
}

// Converts the starting point from degrees, minutes, seconds to meters to use in calculations
void pathSail::startPointLatLongConvert(){
    std::string degrees;
    std::string minutes;
    std::string seconds;
    long longitude;
    // Converts the latitude and longitude to meters to help determine distance to goal for
    // determining h(n)
    // This is all assuming that all coordinates are received as "degrees.minutes.seconds"
    for (unsigned int i = 0; i < 2; i++;){
        for (unsigned int j = 0; j < startLocation[0][i].length(); j++;){
            //iterate through the string to split up degrees, minutes, and seconds
        }
        long degreesLong = std::stol(degrees, 10);
        long minutesLong = std::stol(minutes, 10);
        long secondsLong = std::stol(seconds, 10);

        minutesLong = minutesLong / 60;
        secondsLong = secondsLong / 3600;

        long decimalDegrees = degreesLong + minutesLong + secondsLong;

        if (i = 0) {
            startLocationMeters[0][i] = decimalDegrees / degreeLatDistMeters;
        } else {
            latRads = decimalDegrees * degreesToRadians;
            longitude = cosine(latRads) * equatorDegrees;
            startLocationMeters[0][i] = decimalDegrees / longitude;
        }
    }
}


// Creates the goal for the robot to sail towards for when the camera should take over on navigation
// Need to check heading direction to determine how to solve for x and y coordinates of the goals
void pathSail::buoyGoal(){
    if(heading == north || south && lane == short){
        // add 15 meters in the positive x direction in the event that the boat is slightly to the left of the buoy
        goalPoint[0][0] = startLocationMeters[0][0] + 15;
        goalPoint[0][1] = startLocationMeters[0][1] + shortLaneDist;
    }
    else if(heading == north || south && lane == long){
        // if the lane length is long (assuming the rectangular course), make sure to add lane length to the y direction
        goalPoint[0][0] = startLocationMeters[0][0] + 15;
        goalPoint[0][1] = startLocationMeters[0][1] + longLaneDist;
    }
    else if(heading == east || west && lane == short){
        goalPoint[0][0] = startLocationMeters[0][1] + 15;
        goalPoint[0][1] = startLocationMeters[0][0] + shortLaneDist;
    } else{
        goalPoint[0][0] = startLocationMeters[0][1] + 15;
        goalPoint[0][1] = startLocationMeters[0][0] + longLaneDist;
    }
}

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
