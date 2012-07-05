/*
collisionAvoidance
This is where students will be able to program in a collision avoidance algorithm. The telemetry callback
is already setup along with a dummy version of how the service request would work.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <map>
#include <math.h>
#include <vector>

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

//Algorithm Headers
#include "RRTstar.h"

// Constant Definitions
#define rho 15/(22.5*(M_PI/180.0))
#define TIMESTEP 1
#define UAV_AIRSPEED 11.176
#define WEST_MOST_LONGITUDE (-85.490356)
#define NORTH_MOST_LATITUDE 32.606573
#define METERS_TO_LATITUDE (1.0/110897.21)
#define METERS_TO_LONGITUDE 0.00001065
#define EARTH_RADIUS 6371000.0 //meters
#define DEGREES_TO_RADIANS (M_PI/180.0)
#define RADIANS_TO_DEGREES (180.0/M_PI)
#define MAX_TURN_RAD (22.5*DEGREES_TO_RADIANS)
#define LOOP_RADIUS ((UAV_AIRSPEED*sin((M_PI-MAX_TURN_RAD)/2))/sin(MAX_TURN_RAD))


//ROS service client for calling a service from the coordinator
ros::ServiceClient client;
ros::ServiceClient getWaypointsClient;

//keeps count of the number of services requested
int count;

std::map<int, UAV*> uavMap;

/*give an x,y coordinate and get a Latitude/Longitude
This only works for latitudes around 32.6 degrees north*/
rrt::latlong getLatitudeLongitude(rrt::coord xycoord) 
{
	rrt::latlong result;
	result.latitude = NORTH_MOST_LATITUDE+(xycoord.yval*METERS_TO_LATITUDE);
	result.longitude = (METERS_TO_LONGITUDE*xycoord.xval)+WEST_MOST_LONGITUDE;
	return result;
}


/*takes in a latitude and longitude and gives an x,y coordinate
for that lat/long.
*/
rrt::coord findXYCoordinate(rrt::latlong convert) 
{
    double lat1 = convert.latitude * DEGREES_TO_RADIANS;
    double lat2 = NORTH_MOST_LATITUDE*DEGREES_TO_RADIANS;
    double long1 = convert.longitude * DEGREES_TO_RADIANS;
    double long2 = WEST_MOST_LONGITUDE*DEGREES_TO_RADIANS;
        
    double deltaLat = lat2-lat1;
    double deltaLong= long2-long1;
    
    double x = pow(sin(0 / 2.0),2);
    x = x + cos(lat2)*cos(lat2)*pow(sin(deltaLong/2.0),2);
    x = 2.0 * asin(sqrt(x));
    
    double y = pow(sin(deltaLat/2.0),2);
    y = y + cos(lat1)*cos(lat2)*pow(sin(0/2.0),2);
    y = 2.0 * asin(sqrt(y));
    
    double result[2] = {EARTH_RADIUS*x,EARTH_RADIUS*y};
    rrt::coord ans;
    ans.xval = result[0];
    if(convert.latitude>NORTH_MOST_LATITUDE) {
        ans.yval = result[1];
   }
    else {
        ans.yval = -result[1];
    }
    return ans;
}

/* This function will fill a UAVs waypoint vector up
with all waypoints that need to be met,
returns true if no error, false if there was error
Might not help b/c currentWaypointIndex is always zero
*/
bool fillWaypoints(UAV* myUAV, int myPlaneID) 
{
	int j = 0;
	double latcheck = 0;
	while (latcheck>-1) 
	{
		Point newWaypoint = Point();
		AU_UAV_ROS::RequestWaypointInfo srv;
		srv.request.planeID=myPlaneID;
		srv.request.isAvoidanceWaypoint = false;
		srv.request.positionInQueue = j;
		if(getWaypointsClient.call(srv)) 
		{
			newWaypoint.setY(srv.response.latitude);
			latcheck = srv.response.latitude;
			newWaypoint.setX(srv.response.longitude);
			//ROS_INFO("Plane %i, waypoint %i...x: %f y: %f",myPlaneID,j,srv.response.latitude,srv.response.longitude);
		}
		else 
		{
			ROS_INFO("CALL FAIL");
			return false;
		}
		myUAV->waypoints.push_back(newWaypoint);
		j++;
	}
	//ROS_INFO("before pop %f", myUAV->waypoints.back().getX());
	myUAV->waypoints.pop_back();
	return true;
}

/* This function will fill a UAVs avoidancepoint vector up
with all collision avoidance waypoints that are added,
returns true if no error, false if there was error
*/
bool fillCollisionWaypoints(UAV* myUAV, int myPlaneID) 
{
	int j = 0;
	double latcheck = 0;
	while (latcheck>-1) 
	{
		Point newAvoidpoint = Point();
		AU_UAV_ROS::RequestWaypointInfo srv;
		srv.request.planeID=myPlaneID;
		srv.request.isAvoidanceWaypoint = true;
		srv.request.positionInQueue = j;
		if(getWaypointsClient.call(srv)) 
		{
			newAvoidpoint.setY(srv.response.latitude);
			latcheck = srv.response.latitude;
			newAvoidpoint.setX(srv.response.longitude);
			//ROS_INFO("Plane %i, waypoint %i...x: %f y: %f",myPlaneID,j,srv.response.latitude,srv.response.longitude);
		}
		else 
		{
			ROS_INFO("CALL FAIL");
			return false;
		}
		myUAV->avoidancepoints.push_back(newAvoidpoint);
		j++;
	}
	//ROS_INFO("before pop %f", myUAV->waypoints.back().getX());
	myUAV->waypoints.pop_back();
	return true;
}

Point getWaypoint(UAV *myUav, int myPlaneID) 
{
    Point myPoint = Point();
    return myPoint;
}

/*This function handles the RRTpath*/

void enactRRTstar(UAV *currentUAV, int myPlaneID, rrt::coord target)
{
	// Set Problem Situation TODO: Make these actually serve a purpose!
	int fieldsize = 2000; 

	// Fill Vector with UAVs to avoid
	std::vector<UAV*> otheruav;
	otheruav.clear();
	int j = 0;
	typedef std::map<int, UAV*> uavs;
	for(uavs::const_iterator iterator = uavMap.begin(); iterator != uavMap.end(); iterator++) 
	{
		UAV* adding = iterator->second;
		// Make sure UAV has path to be avoided and UAV doesn't try to avoid itself
		if ((adding->avoidancepoints.size() >0) && (iterator->first != myPlaneID))
		{
			otheruav.push_back(adding);
			ROS_INFO("Added UAV %i", iterator->first);
			j++;
		}
	}

	rrt::coord currentpos;
	currentpos.xval = currentUAV->getX(); currentpos.yval = currentUAV->getY();

	ROS_INFO("Rocking plane %i x=%f y=%f targetWaypointx=%f targetWaypointY=%f",
	 myPlaneID, currentUAV->getX(), currentUAV->getY(), target.xval, target.yval);
		
	// Draw Path From current to goal
	std::vector<rrt::coord> path = rrt::RRTstar(currentUAV,target,fieldsize,otheruav);

	// For First Point, True to Clear Queue
	bool first = true;

	// Loop Through Path
	for (int i=0; i<path.size(); i++)
	{
		// Get Path Point
		rrt::coord curr = path.at(i);
		// Convert to lat/long
		rrt::latlong currconv = getLatitudeLongitude(curr);
		double lat = currconv.latitude;
		double lon = currconv.longitude;
		
		// Set Points to Send
		AU_UAV_ROS::GoToWaypoint srv;
		srv.request.planeID = myPlaneID;
		srv.request.latitude = lat;
		srv.request.longitude = lon;
		srv.request.altitude = 400;

		//Set as Avoidance Waypoint
		srv.request.isAvoidanceManeuver = true;
		if (first) // Only clear if this is first waypoint introduced
		{
			srv.request.isNewQueue = true;
			first = false;
		}
		else // After first, want to add others without clearing
		{
			srv.request.isNewQueue = false;
		}

		//check to make sure the client call worked (regardless of return values from service)
		if(client.call(srv))
		{
			ROS_INFO("Sent avoidance point %i: x=%f, y=%f, lat=%f, long=%f",i,curr.xval,curr.yval, 	
			currconv.latitude,currconv.longitude);
		}
		else
		{
			ROS_ERROR("Did not receive response");
		}
	}
	fillCollisionWaypoints(currentUAV,myPlaneID);
	//delete path;
}	

/*
Checks if UAV is stuck in loop around waypoint
TODO make this work....
*/
bool loopcheck(UAV* currentUAV)
{
	return false;
}


/*called whenever a telemetry update is made*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
    
	//this will be replaced by students to do more than just send a string
	 //std::stringstream ss;
	//ss << "Sending service request " << count;
    
    UAV* currentUAV;
	rrt::latlong currentlatlong;
	currentlatlong.latitude = msg->currentLatitude;
	currentlatlong.longitude = msg->currentLongitude;
    rrt::coord currentpos = findXYCoordinate(currentlatlong);
    
    //see if our uav is in map
    if (uavMap.count(msg->planeID)==0) 
	{
        	currentUAV = new UAV;
       		uavMap[msg->planeID] = currentUAV;
      		currentUAV->setXVel(0);
        	currentUAV->setYVel(UAV_AIRSPEED);
        	currentUAV->setBearing(90.);
        	//ROS_INFO("1. current bearing %f", currentUAV->getBearing());
        	//ROS_INFO("check %f",uavMap[msg->planeID]->getBearing());
    	}
    else 
	{
        	//set currentuav pointer and the uav's bearing
        	currentUAV = uavMap[msg->planeID];
        	double deltaX = currentpos.xval-currentUAV->getX();
        	double deltaY = currentpos.yval-currentUAV->getY();
        	if(msg->currentWaypointIndex!=-1) 
		{
            		//ROS_INFO("if statement triggered");
            		currentUAV->setBearing(atan2(deltaY, deltaX));
            		//ROS_INFO("delta y = %f, delta x = %f",deltaY, deltaX);
        	}
        	//ROS_INFO("2. current bearing %f", currentUAV->getBearing());
   
    	}

    //if no waypoints yet, fill it up for our uav
   // if(currentUAV->waypoints.size()<1) 
	//{  FILL NO MATTER WHAT so we know when we only have 1 left
        	fillWaypoints(currentUAV,msg->planeID);
    	//}


    // Set Current Values for UAV
    currentUAV->setX(currentpos.xval);
    currentUAV->setY(currentpos.yval);

	// Find Destination Values
	rrt::latlong destlatlong;
	AU_UAV_ROS::RequestWaypointInfo srv2;
        srv2.request.planeID=msg->planeID;
        srv2.request.isAvoidanceWaypoint = false;
        srv2.request.positionInQueue = 0;
        if(getWaypointsClient.call(srv2))
	{
		destlatlong.longitude = srv2.response.longitude;
		destlatlong.latitude = srv2.response.latitude;
	}
    	rrt::coord targetpos = findXYCoordinate(destlatlong);

	//ROS_INFO("Update sent from plane %i, currentx=%f, currenty=%f, destinationx=%f, destinationy=%f", msg->planeID, currentpos.xval,
		//currentpos.yval, targetpos.xval, targetpos.yval);
    
    //if no path exists, and we have a place to go, create the path
    
	// Grab Next waypoint from Queue
    AU_UAV_ROS::RequestWaypointInfo service;
    service.request.planeID=msg->planeID;
    service.request.isAvoidanceWaypoint = true;
    service.request.positionInQueue = 0;
    if(getWaypointsClient.call(service)) {
         //ROS_INFO("Received response from service request %d", (count++));
        // ROS_INFO("target latitude is %f longitude is %f ",service.response.latitude, service.response.longitude);
     }
    else {
         ROS_ERROR("Did not receive response");
     }

	// Are the UAVs ready to begin?
	bool start = (destlatlong.latitude>-1);
    
	// Set to run when Avoidance cue is clear, when more than 1 waypoint is left, and when UAVs are ready
	if(service.response.latitude<0 && currentUAV->waypoints.size()>1 && start)
	{
		enactRRTstar(currentUAV, msg->planeID, targetpos);		
	}
	else
	{
		// If not filling avoidance cue, check if plane is stuck in a loop of death =P
		if (loopcheck(currentUAV))
		{
			//TODO something....
		}
	}
}


int main(int argc, char **argv) {
//standard ROS startup
ros::init(argc, argv, "collisionAvoidance");
ros::NodeHandle n;
//subscribe to telemetry outputs and create client for the avoid collision service
ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
    getWaypointsClient = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
    
    
    
//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
srand(time(NULL));
//initialize counting
count = 0;
    
    
    
//needed for ROS to wait for callbacks
ros::spin();



return 0;
}
