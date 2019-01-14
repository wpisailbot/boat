//
// Created by Sierra Palmer on 1/14/19.
//

#ifndef BOAT_PATHSAIL_H
#define BOAT_PATHSAIL_H

#endif //BOAT_PATHSAIL_H

namespace sailbot{
namespace control{

    class pathSail{
    public:

        // Converts the current point from degrees, minutes, seconds to meters to use in calculations
        void latLongConvert(){}

        // Converts the starting point from degrees, minutes, seconds to meters to use in calculations
        void startPointLatLongConvert(){}


        // Creates the goal for the robot to sail towards for when the camera should take over on navigation
        // Need to check heading direction to determine how to solve for x and y coordinates of the goals
        void buoyGoal(){}

    };

    // Creates a point to note where the boat is
    class point {
    public:
        point( int a = 0, int be = 0) { x = a; y = b; }
        bool operator == ( const point& o){return o.x == x && o.y == y;}
        point operator +( const point& o ){return point(o.x + x, o.y +y); }
        int x, y;
    };

    // Creates a class for the map if the path is along the short side of the course
    class mapShort{
    public:
        mapShort(){
            char t[6][10] = {
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}
            };
            wS = 6; // width of the map is 12 meters split up into 2 meter squares, making 6
            hS= 10; // height of map is 20 meters split up into 2 meter squares, making 10
            for (int r = 0; r < hS; r++)
                for( int s = 0; s<wS; s++)
                    mS[s][r] = t[r][s];
        }
        int operator() (int x, int y){return m[x][y];}
        char mS[6][10];
        int wS, hS;
    };

    // Creates a class for the map if the path is along the long side of the course
    class mapLong{
    public:
        mapLong(){
            char t[6][20] = {
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}
            };
            wL = 6; // width of the map is 12 meters split up into 2 meter squares, making 6
            hL= 20; // height of map is 40 meters split up into 2 meter squares, making 20
            for (int r = 0; r < hL; r++)
                for( int s = 0; s<wL; s++)
                    mL[s][r] = t[r][s];
        }
        int operator() (int x, int y){return m[x][y];}
        char mL[6][10];
        int wL, hL;
    };

    // Creates each node for the boat to check
    class node{
    public:
        bool operator == (const node& o){ return pos == o.pos; }
        bool operator == (const point& o){ return pos == o; }
        bool operator < (const node& o){return dist + cost < o.dist + o.cost; }
        point pos, parent;
        int dist, cost;
    };

    // Starting point for A* for the short side length
    class aStarShort{
    public:
        astarShort() {
            // The boat can move in 8 different directions in order to get to the goal, which can be calculated
            // as a neighbor point
            neighbors[0] = point (-1, -1); neighbors[1] = point (1, -1);
            neighbors[2] = point (-1, 1);  neighbors[3] = point (1, 1);
            neighbors[4] = point (0, -1);  neighbors[5] = point (-1, 0);
            neighbors[6] = point(0, 1);    neighbors[7] = point (1, 1);
        }

        // Creates the "sailing lane" needed to navigate to the goal point and then returns the cost to make sure that the
        // boat is staying within the lane
        long laneCost_Creation(){}

        // Need to determine a cost for the distance to the goal based upon the current location and the location of the next
        // node "n" by using the distance equation
        void distCost() {}

        // Need to determine the cost based upon the direction the wind is blowing assuming it's considered an angle
        long calcWindCost(){}

        long costTotaled(){}

        bool pointValid(point& p ) {}

        bool pointExist( point& p ){}

        bool fillOpenNode( node& node ){}

        bool searchLong(point& s, point& e, mapLong& mapL1){}

        bool searchShort(point& s, point& e, mapShort& mapS1){}

        int path( std::list<point>& path){}

        mapLong mL; mapShort mS; point end, start;
        point neighbors[8];
        std::list<node> open;
        std::list<node> closed;
    };




    }
}
