
#include "stationKeeping.h"

double time; // Elapsed time during the duration of the event
double keepingTime = 300; // The number of seconds in five minutes
double exitTime = 30; // The number of seconds allotted in order to leave the box
float waypoints[4][3];
float endGatePos_Midpoint[2][3];
int buoyCount = 0;

int main(){
	// Set the clock to 0 and have it start running for the duration of the 
	// event
	clockTimer();
	sailThroughGate(); // sail into the box

	while(timer < keepingTime){ // Has the boat count down the time to stay inside box
		resetPoints();
		centerTackAndLocate();
		createObstacleFence();
		boxSailing();
	}

	time = 0;
	resetPoints();

	while(timer < exitTime){
		sailThroughGate();
	}
}
