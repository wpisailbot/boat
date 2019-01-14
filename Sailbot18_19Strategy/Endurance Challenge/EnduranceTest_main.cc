/*Endurance/Long Distance event
*
* As taken from the sailbot 2018 events website:
* Challenge goal: To demonstarate the boat's durability and capabitlity to sail 
* some distance
*
* Description: The boats will sail around 4 buoys (passing within 10 m inside of 
* a buoy is OK) for up to 7 hours
*
* This code was written by Sydney Fisher and Sierra Palmer as part of the WPI 
* 2018-2019 Sailbot team to accomplish this task
*/

int main(){
	// Set the clock to 0 and have it start running for the duration of the 
	// event
	clockTimer();
	
	// continue sailing until the end of the race
	while(time < endRaceTime){
		// keep having the boat sail forward unitl there is a buoy detected, then 
		// keep moving through the code
		while(buoyDetect()!= TRUE){
			buoyDetect();
		}
		findCentroid();
		setObstacleGate();
		sailTo();
		resetPoints();
	}
}
