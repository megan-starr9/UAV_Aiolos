/*
	My OMPL RRTstar implementation!
	Megan Lyle
	(In demos for now...)
*/

#include "RRTstar.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/contrib/rrt_star/BallTreeRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <vector>

// CONTROL VARIABLES
//Time index control (buffer of points at any time)
#define TIME_INDEX 3
// Collision buffer zone around avoidance point
#define BUFFER_ZONE 40
// Speed of UAV (meters/sec)
#define SPEED 11.176
// Length between generated points
#define SEGMENT_LENGTH 40
// Goal Bias
#define GOAL_BIAS .07
// Runtime for algorithm
#define RUNTIME .06
// Max Turn
#define MAXTURN (SEGMENT_LENGTH/SPEED)*(M_PI/8)

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;


// COLLISION CHECK will go here
bool isStateValid(std::vector<UAV*> otheruavs,UAV* currentUAV,og::RRTstar* planner, const ompl::base::State *state)
{
	// Cast state type and set values to compare against
   	const ompl::base::SE2StateSpace::StateType *s = state->as<ompl::base::SE2StateSpace::StateType>();
   	double x=s->getX(), y=s->getY();

	// Fetch parent motion
	og::RRTstar::Motion* parent = planner->getparent();

	int newindex = (parent == NULL) ? 1 : parent->node_index();

	// set boolean accumulation variable
	bool result = true;

	// Fetch Array
	// Compare against array
	for (unsigned int i=0; i < otheruavs.size(); i++)
	{
	UAV* curruav = otheruavs.at(i);
	std::vector<Point> capoints = curruav->avoidancepoints;
	// Initiate time index
	int index = 0;
		for(unsigned int j=0; j < capoints.size(); j++)
		{
		Point currpoint = capoints.at(j);
		double currx = currpoint.getX();
		double curry = currpoint.getY();

		double dist = sqrt((x-currx)*(x-currx)+(y-curry)*(y-curry));
		bool currresult = (dist > BUFFER_ZONE);
		
		if (!currresult)
		{
			currresult = (abs(index-newindex) > TIME_INDEX);
		}

		result = currresult && result;
		
		index++;
		}
	}

    return result;
}


// Main implementation
std::vector<rrt::coord> rrt::RRTstar(UAV* currentUAV, rrt::coord end, int fieldsize, std::vector<UAV*> otheruavs)
{

    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::DubinsStateSpace(MAXTURN, false));

    // set the bounds for the R^3 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-fieldsize);
    bounds.setHigh(fieldsize);

    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

	// initialize Simple Setup
	og::SimpleSetup ss(space);

    // create a random start state
    ob::ScopedState<> start(space);
    start[0]=currentUAV->getX();
    start[1]=currentUAV->getY();
	start[2] = currentUAV->getBearing();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal[0]=end.xval;
    goal[1]=end.yval;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // create a planner for the defined space
	og::RRTstar* planner = new og::RRTstar(ss.getSpaceInformation());
	planner->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
	planner->setGoalBias(GOAL_BIAS);
	planner->setRange(SEGMENT_LENGTH);
	//planner->setTurnAngle(MAXTURN);
	//planner->setInitialBearing(currentUAV->getBearing());

	ss.setPlanner(ompl::base::PlannerPtr(planner));

	base::PlannerTerminationCondition stop = base::timedPlannerTerminationCondition(RUNTIME);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid,otheruavs,currentUAV, planner, _1));
 
    // attempt to solve the problem within one second of planning time
    bool solved = ss.solve(stop);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        ss.getSolutionPath().print(std::cout);

	//Solution path ^^
	og::PathGeometric path = ss.getSolutionPath();

	int vertices = path.getStateCount();

	std::vector<rrt::coord> solutionpath(vertices);
	for (int i=0; i<vertices; i++)
	{
		rrt::coord newstruct;
		ob::ScopedState<> curr(space);
		curr = path.getState(i);
		newstruct.xval = curr[0];
		newstruct.yval = curr[1];
		newstruct.heading = curr[2];

		solutionpath.at(i) = newstruct;
	}
		return solutionpath;
    }
    else
	{
	std::vector<rrt::coord> null;
	return null;
        std::cout << "No solution found" << std::endl;
	}
}
