
int buoyCount = 0; // Counts the number of buoys that the robot sails around
double R; // the radius from the buoy centroid to the outer point of the buoy
float waypoints[2][3];
float endGatePos_Midpoint[2][3];
float obstacleGate[4][3];

bool buoyDetect();
void buoyLocate();
void findCentroid();
void setObstacleGate();
void sailTo();
void resetPoints();
void findEndGateCentroid();
void sailThroughGate();
