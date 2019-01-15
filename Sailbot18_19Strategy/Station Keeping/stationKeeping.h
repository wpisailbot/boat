
double time; // Elapsed time during the duration of the event
double keepingTime = 300; // The number of seconds in five minutes
double exitTime = 30; // The number of seconds allotted in order to leave the box
float waypoints[4][3];
float endGatePos_Midpoint[2][3];
int buoyCount = 0;

void clockTimer();
void findEndGateCentroid();
void resetPoints();
void sailThroughGate();
void findCentroid();
void createObstacleFence();
void centerTackAndLocate();
void boxSailing();
