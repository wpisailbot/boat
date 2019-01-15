double time; // elapsed time in seconds
double endRaceTime = 25200; // number of seconds in 7 hours
double R; // the radius from the buoy centroid to the outer point of the buoy
float waypoints[2][3]; 
float obstacleGate[4][3];

void clockTimer();
bool buoyDetect();
void findCentroid();
void setObstacleGate();
void sailTo();
void resetPoints();
