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

void clockTimer(){
	// Use some built in clock, set it to 0 at the start, and then check 
	// intermittently to see when the race is ending
	time = 0;
	runTimer(); // Start the timer counting up 
}

void findEndGateCentroid(){
	// The boat's camera will find the centroid of the buoy, and 
	// then set the centroid as a waypoint
		
	// Some distance calculations integrated with the MATLAB
	// color sensing library and produces the radius of the 
	// buoy
		
	// Determine the GPS coordinates of the boat at that instance
	// to determine the distance from the boat to the buoy. 
	// Then add calcuations to find the actual centroid of the port
	// buoy in terms of the GPS coordinates
	float xValP = xValP + 0;
	float yValP = yValP - R;
	float zValP = zValP + R;
		
	// Set each of the GPS coordinates in the waypoint array in order
	// to create a waypoint for the boat to know where the buoy is
	xValP = endGatePos_Midpoint[0][0];
	yValP = endGatePos_Midpoint[0][1];
	zValP = endGatePos_Midpoint[0][2];

	// Find the centroid fo the starboard buoy in terms of the GPS
	// coordinates 
	float xValS = xValS + 0;
	float yValS = yValS - R;
	float zValS = zValS + R;

	// Set each of the GPS coordinates in the waypoint array in order
	// to create a waypoint for the boat to know where the buoy is
	xValS = endGatePos_Midpoint[1][0];
	yValS = endGatePos_Midpoint[1][1];
	zValS = endGatePos_Midpoint[1][2];

	// Find the midpoint between the port and starboard buoys to 
	// create a waypoint for the robot to sail towards
	midpointX = abs(endGatePos_Midpoint[1][0] - endGatePos_Midpoint[0][0]);
	midpointY = abs(endGatePos_Midpoint[1][1] - endGatePos_Midpoint[0][1]);
	midpointZ = abs(endGatePos_Midpoint[1][2] - endGatePos_Midpoint[0][2]);

	waypoint[1][0] = midpointX;
	waypoint[1][1] = midpointY;
	waypoint[1][2] = midpointZ;
}

void resetPoints(){
	// Reset all the points so that the boat can do the same operation at each buoy
	
	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			waypoint[i][j]= 0;
		}
	}
	
	for(int k=0;k<4;k++){
		for(int l=0;l<3;l++){
			obstacleGate[k][l]=0;
		}
	}
}

void sailThroughGate(){
	// Find the centroids of the two buoys making up the gate, find the midpoint
	// of the distance between them, set it as a waypoint, and then sail to that
	// waypoint in order to sail between the gates

	findEndGateCentroid();
	turn(waypoint[1][0], waypoint[1][1], waypoint[1][2]);
	sailForward();
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
	int m = 0;
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
	xVal = waypoint[m][0];
	yVal = waypoint[m][1];
	zVal = waypoint[m][2];

	m++;
}

void createObstacleFence(){
	// Use the waypoints in order to to create an obstacle fence
	// Create obstacle points between the shortest distance between buoys
	// Need to figure out how to best implement this
}

void centerTackAndLocate(){
	// Tack in the center to locate each of the four buoys. Then create the centroid
	// location of each 
	while(buoyCount<5){
		tackBoat();
		buoyDetect();
		if(buoyDetect == TRUE){
			findCentroid();
			buoyCount++;
		}
	}
}

void boxSailing(){
	// Sail straight and tack around the box for the duration of the time
	// within the obstacle fence
	while(boatLocation){ //is within the obstacle fence
		tackBoat();
		sailForward();
	}

}

