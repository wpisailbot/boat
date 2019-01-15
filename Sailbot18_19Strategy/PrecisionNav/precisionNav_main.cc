/* Precision Navigation
*
* As taken from the sailbot 2018 events website:
* Challenge goal: To demonstrate the boat's ability to autonomously navigate a 
* course within tight tolerances
*
* Description: The boat will start between two buoys and then will autonomously
* sail a course around two buoys and then returen between the two start buoys
*
* This code was written by Sydney Fisher and Sierra Palmer as part of the WPI 
* 2018-2019 Sailbot team to accomplish this task
*/

#include "precisionNav.h"

int main(){
	// First locate the buoys and move towards the most starboard one
	buoyLocate();

	while(buoyCount<2){
		// Navigate around the two buoys at the base of the triangular course
		findCentroid();
		setObstacleGate();
		sailTo();
		resetPoints();
	}

	// Find the midpoint between the two gate buoys, and sail through them to 
	// end the competition
	sailThroughGate();	
}
