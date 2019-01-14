
#include "stationKeeping.h"

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
