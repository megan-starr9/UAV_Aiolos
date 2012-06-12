
#include "ompl/src/ompl/control/SpaceInformation.h"
#include "ompl/src/ompl/base/GoalState.h"
#include "ompl/src/ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/src/ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/src/ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/src/ompl/control/planners/rrt/RRT.h"
#include "ompl/src/ompl/base/spaces/SE2StateSpace.h"
#include "ompl/src/ompl/control/SimpleSetup.h"
#include "ompl/src/ompl/config.h.in"
#include <iostream>
#include <stdlib.h>

void rrtstar(void)
{
	namespace og = ompl::geometric;
	namespace ob = ompl::base;
	namespace oc = ompl::control;

	/*ob::StateSpacePtr stateSpace(new ob::DubinsStateSpace);
	og::SimpleSetup setup(stateSpace);
	og::RRTConnect* planner = new og::RRTConnect(setup.getSpaceInformation());
	planner->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
	setup.setPlanner(ompl::base::PlannerPtr(planner));*/

	std::cout<<"Hello! \n";
	
	return;
}
	
int main()
{
	rrtstar();

	std::cout<<"Obviously it runs! (hopefully) \n";
	
	return 0;
}
