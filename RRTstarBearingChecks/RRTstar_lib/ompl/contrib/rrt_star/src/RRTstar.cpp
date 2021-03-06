/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#include "ompl/contrib/rrt_star/RRTstar.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include <algorithm>
#include <limits>
#include <map>

ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    ballRadiusMax_ = 0.0;
    ballRadiusConst_ = 1.0;
    delayCC_ = true;
	parent_ = NULL;
	maxTurnAngle_ = 0.0;
	initialBearing_ = 0.0;


    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange);
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias);
    Planner::declareParam<double>("ball_radius_constant", this, &RRTstar::setBallRadiusConstant, &RRTstar::getBallRadiusConstant);
    Planner::declareParam<double>("max_ball_radius", this, &RRTstar::setMaxBallRadius, &RRTstar::getMaxBallRadius);
    Planner::declareParam<bool>("delay_cc", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC);
}

ompl::geometric::RRTstar::~RRTstar(void)
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    ballRadiusMax_ = si_->getMaximumExtent();
    ballRadiusConst_ = maxDistance_ * sqrt((double)si_->getStateSpace()->getDimension());

    delayCC_ = true;

    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
    nn_->setDistanceFunction(boost::bind(&RRTstar::distanceFunction, this, _1, _2));
}

void ompl::geometric::RRTstar::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

bool ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
	parent_ = NULL;

    if (!goal)
    {
        msg_.error("Goal undefined");
        return false;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
	// TODO set this based on initial bearing
	motion->set_bearing(initialBearing_);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    msg_.inform("Starting with %u states", nn_->size());

    Motion *solution       = NULL;
    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion     = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    std::vector<Motion*> solCheck;
    std::vector<Motion*> nbh;
    std::vector<double>  dists;
    std::vector<int>     valid;
    unsigned int         rewireTest = 0;

    while (ptc() == false)
    {
	bool validneighbor = false;
	Motion *nmotion;
	base::State *dstate;
		//ADDED
	parent_ = NULL;
	//while (!validneighbor)
	//{
		if (ptc())
		{
			msg_.error("Loop failed to find proper states!");
			return false;
		}
	        // sample random state (with goal biasing)
	        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
	            goal_s->sampleGoal(rstate);
	        else
	            sampler_->sampleUniform(rstate);

	        // find closest state in the tree
	        nmotion = nn_->nearest(rmotion);

	        dstate = rstate;

        	// find state to add
        	double d = si_->distance(nmotion->state, rstate);
        	if (d > maxDistance_)
        	{
            		si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            		dstate = xstate;
       		}
		
		validneighbor = check_bearing(nmotion, dstate);
	//} 
 	
	parent_ = nmotion;
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
		//ADDED
            double distN = si_->distance(dstate, nmotion->state);
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
		motion->set_bearing();
            motion->cost = nmotion->cost + distN;

            // find nearby neighbors
            double r = std::min(ballRadiusConst_ * (sqrt(log((double)(1 + nn_->size())) / ((double)(nn_->size())))),
                                ballRadiusMax_);

            nn_->nearestR(motion, r, nbh);
            rewireTest += nbh.size();

            // cache for distance computations
            dists.resize(nbh.size());
            // cache for motion validity
            valid.resize(nbh.size());
            std::fill(valid.begin(), valid.end(), 0);

            if(delayCC_)
            {
                // calculate all costs and distances
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                    nbh[i]->cost += si_->distance(nbh[i]->state, dstate);

                // sort the nodes
                std::sort(nbh.begin(), nbh.end(), compareMotion);

                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    dists[i] = si_->distance(nbh[i]->state, dstate);
                    nbh[i]->cost -= dists[i];
                }

                // collision check until a valid motion is found
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        double c = nbh[i]->cost + dists[i];
                        if (c < motion->cost)
                        {
				//ADDED
				parent_ = nbh[i];
                            if (si_->checkMotion(nbh[i]->state, dstate) /*&& check_bearing(nbh[i], motion->state)*/)
                            {
                                motion->cost = c;
                                motion->parent = nbh[i];
				motion->set_bearing();
                                valid[i] = 1;
                                break;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        valid[i] = 1;
                        dists[i] = distN;
                        break;
                    }
                }
            }
            else
            {
                // find which one we connect the new state to
                for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        dists[i] = si_->distance(nbh[i]->state, dstate);
                        double c = nbh[i]->cost + dists[i];
                        if (c < motion->cost)
                        {
				// ADDED
				parent_ = nbh[i];
                            if (si_->checkMotion(nbh[i]->state, dstate) /*&& check_bearing(nbh[i], dstate)*/)
                            {
                                motion->cost = c;
                                motion->parent = nbh[i];
				motion->set_bearing();
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        valid[i] = 1;
                        dists[i] = distN;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            solCheck.resize(1);
            solCheck[0] = motion;

            // rewire tree if needed
            for (unsigned int i = 0 ; i < nbh.size() ; ++i)
                if (nbh[i] != motion->parent)
                {
                    double c = motion->cost + dists[i];
                    if ((c < nbh[i]->cost))
                    {
			parent_ = motion;
                        bool v = valid[i] == 0 ? si_->checkMotion(nbh[i]->state, dstate) : valid[i] == 1;
                        if (v /*&& check_bearing(motion, nbh[i]->state)*/)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[i]);
                            double delta = c - nbh[i]->cost;

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->cost = c;
				nbh[i]->set_bearing();
                            nbh[i]->parent->children.push_back(nbh[i]);
                            solCheck.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i], delta);
                        }
                    }
                }

            // check if we found a solution
            for (unsigned int i = 0 ; i < solCheck.size() ; ++i)
            {
                double dist = 0.0;
                bool solved = goal->isSatisfied(solCheck[i]->state, &dist);
                sufficientlyShort = solved ? goal->isPathLengthSatisfied(solCheck[i]->cost) : false;

                if (solved)
                {
                    if (sufficientlyShort)
                    {
                        solution = solCheck[i];
                        break;
                    }
                    else if (!solution || (solCheck[i]->cost < solution->cost))
                    {
                        solution = solCheck[i];
                    }
                }
                else if (!solution && dist < approximatedist)
                {
                    approximation = solCheck[i];
                    approximatedist = dist;
                }
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    double solutionCost;
    bool approximate = (solution == NULL);
    bool addedSolution = false;
    if (approximate)
    {
        solution = approximation;
        solutionCost = approximatedist;
    }
    else
        solutionCost = solution->cost;

    if (solution != NULL)
    {
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *path = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);
        goal->addSolutionPath(base::PathPtr(path), approximate, solutionCost);
        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    msg_.inform("Created %u states. Checked %lu rewire options.", nn_->size(), rewireTest);

    return addedSolution;
}

// MY FUNCTIONS
bool ompl::geometric::RRTstar::check_bearing(Motion *parent, const base::State *child)
{
	double thetamin = parent->bearing - maxTurnAngle_;
	double thetamax = parent->bearing + maxTurnAngle_;
	const ompl::base::SE2StateSpace::StateType *p = parent->state->as<ompl::base::SE2StateSpace::StateType>();
	const ompl::base::SE2StateSpace::StateType *c = child->as<ompl::base::SE2StateSpace::StateType>();
	double dx=(c->getX())-(p->getX());
	double dy=(c->getY())-(p->getY());

	bool ans = ((atan2(dy,dx)>thetamin) && (atan2(dy,dx)<thetamax));

	return ans;
}

// END

void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    std::vector<Motion*>::iterator it = m->parent->children.begin ();
    while (it != m->parent->children.end ())
    {
        if (*it == m)
        {
            it = m->parent->children.erase(it);
            it = m->parent->children.end ();
        }
        else
            ++it;
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m, double delta)
{
    for (size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost += delta;
        updateChildCosts(m->children[i], delta);
    }
}

void ompl::geometric::RRTstar::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}
