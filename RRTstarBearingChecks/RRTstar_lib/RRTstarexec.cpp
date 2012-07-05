#include <iostream>
#include "RRTstar.h"

int main()
{
	// Instantiate for test purposes
	rrt::coord start, end;
	start.xval = 3.75;
	start.yval = 0;
	start.heading = 0.0;
	end.xval = 60;
	end.yval = 90;
	end.heading = 0.0;

	rrt::RRTstar(start,end);
	
	return 0;
}
