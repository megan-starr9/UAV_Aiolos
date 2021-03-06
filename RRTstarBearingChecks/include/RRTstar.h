#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <vector>
#include "UAV.h"

namespace rrt
{

// define structure for transferring from ROS to OMPL
struct coord
{
	double xval;
	double yval;
	double heading;
};

struct latlong
{
	double latitude;
	double longitude;
};

std::vector<coord> RRTstar(UAV* currentUAV, coord finish, int fieldsize, std::vector<UAV*> obstacles);

}

#endif
