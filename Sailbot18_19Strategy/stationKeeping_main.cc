/* Station Keeping
*
* As taken from the sailbot 2018 events website:
* Challenge goal: To demonstrate the ability of the boat to remain close to one position
* and respond to time-based commands.
*
* Description: The boat will enter a 40x40 m box adn attempt to stay inside the box for 
* 5 minutes. It must then exit within 30 seconds to avoid a penalty.
*
* This code was written by Sydney Fisher and Sierra Palmer as part of the WPI 
* 2018-2019 Sailbot team to accomplish this task
*/


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
