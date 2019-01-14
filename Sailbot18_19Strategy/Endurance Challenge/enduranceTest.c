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

double time; // elapsed time in seconds
double endRaceTime = 25200; // number of seconds in 7 hours
double R; // the radius from the buoy centroid to the outer point of the buoy
float waypoints[2][3]; 
float obstacleGate[4][3];

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

void clockTimer(){
	// Use some built in clock, set it to 0 at the start, and then check 
	// intermittently to see when the race is ending
	time = 0;
	runTimer();
}

boolean buoyDetect(){
	// The robot will sail forward at all times, but once the camera detects a
	// buoy, the boat should sail towards the buoy. Otherwise, it needs to 
	// keep sailing forward
	sailForward();
	if(buoy == detected){
		return TRUE;
	}
	return FALSE;
}

void findCentroid(){
	// The boat's camera will find the centroid of the buoy, and 
	// then set the centroid as a waypoint
		
	// Some distance calculations integrated with the MATLAB
	// color sensing library and produces the radius of the 
	// buoy
		
	// Determine the GPS coordinates of the boat at that instance
	// to determine the distance from the boat to the buoy. 
	// Then add calcuations to find the actual centroid in terms of
	// the GPS coordinates
	float xVal = xVal + 0;
	float yVal = yVal - R;
	float zVal = zVal + R;
		
	// Set each of the GPS coordinates in the waypoint array in order
	// to create a waypoint for the boat to know where the buoy is
	xVal = waypoint[0][0];
	yVal = waypoint[0][1];
	zVal = waypoint[0][2];
}

void setObstacleGate(){
	// Once the centroid is determined as a waypoint, an "obstacle gate" 
	// needs to be created at a radius of 3 meters + the radius of the 
	// buoy to make sure that the sailbot will not get too close to the 
	// buoy while continuing on the course (Still need to figure out math
	// for this)
	
	// First need to calculate all of the x,y, and z coordinates based upon
	// the distance from the centroid and its GPS coordinates
	float aftObstacleX = waypoint[0][0];
	float aftObstacleY = waypoint[0][1] - (3 + R);
	float aftObstacleZ = waypoint[0][2];
	
	float starboardObstacleX = waypoint[0][0] + (3+R);
	float starboardObstacleY = waypoint[0][1];
	float starboardObstacleZ = waypoint[0][2];
	
	float forwardObstacleX = waypoint[0][0];
	float forwardObstacleY = waypoint[0][1]+(3+R);
	float forwardObstacleZ = waypoint[0][2];
	
	float portObstacleX = waypoint[0][0]-(3+R);
	float portObstacleY = waypoint[0][1];
	float porstObstacleZ = waypoint[0][2];
	
	
	// Now all of these points need to be set as part of the obstacle gate,
	// so they'll be input into the obstacle gate array
	
	aftObstacleX = obstacle[0][0];
	aftObstacleY = obstacle[0][1];
	aftObstacleZ = obstacle[0][2];
	
	starboardObstacleX = obstacle[1][0];
	starboardObstacleY = obstacle[1][1];
	starboardObstacleZ = obstacle[1][2];
	
	forwardObstacleX = obstacle[2][0];
	forwardObstacleY = obstacle[2][1];
	forwardObstacleZ = obstacle[2][2];
	
	portObstacleX = [3][0];
	portObstacleY = [3][1];
	porstObstacleZ = [3][2];
	
	// Create a full circle using all of the obstacle points (need to figure out
	// math)
	
	// The boat needs to go about the starboard side of the buoy, so the 
	// starboard obstacle point also needs to be set as a waypoint for the 
	// boat to sail towards
	
	starboardObstacleX = starboardWaypointX;
	starboardObstacleY = starboardWaypointY;
	starboardObstacleZ = starboardWaypointZ;
	
	starboardWaypointX = waypoint[1][0];
	starboardWaypointY = waypoint[1][1];
	starboardWaypointZ = waypoint[1][2];
	
}

void sailTo(){
	// Have the boat turn towards the waypoint set to the starboard side of the
	// buoy
	
	// Command "turn" takes in the three values of the waypoint and has the boat
	// move turn towards that direction
	turn(waypoint[1][0],waypoint[1][1], waypoint[1][2]);
	
	// Use the buoyDetect function to keep checking that the buoy is within camera 
	// range. Once the camera no longer detects, the boat should tack to go towards
	// the new buoy
	if(buoyDetect == TRUE){
		sailForward();
	}
	else{
		tackBoat();
	}
}	

void resetPoints(){
	// Reset all the points so that the boat can do the same operation at each buoy
	
	for(i=0;i<2;i++){
		for(j=0;j<3;j++){
			waypoint[i][j]= 0;
		}
	}
	
	for(k=0;k<4;k++){
		for(l=0;l<3;l++){
			obstacleGate[k][l]=0;
		}
	}
}
