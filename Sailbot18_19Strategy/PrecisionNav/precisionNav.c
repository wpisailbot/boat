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

int buoyCount = 0; // Counts the number of buoys that the robot sails around
double R; // the radius from the buoy centroid to the outer point of the buoy
float waypoints[2][3];
float endGatePos_Midpoint[2][3];
float obstacleGate[4][3];

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

void buoyLocate(){
	// Detect the two buoys at the base of the triangular course, and then 
	// move towards the buoy farthest starboard

	if(buoyDetect == TRUE){
		// Calculate the distances between the boat and the two buoys, and turn
		// towards the buoy that is the most starboard
		sailForward(); 

	}
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

	buoyCount++;
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

void sailThroughGate(){
	// Find the centroids of the two buoys making up the gate, find the midpoint
	// of the distance between them, set it as a waypoint, and then sail to that
	// waypoint in order to sail between the gates

	findEndGateCentroid();
	turn(waypoint[1][0], waypoint[1][1], waypoint[1][2]);
	sailForward();
}

