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

/* Author: Ryan Luna */

#include "ompl/control/planners/est/EST.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::control::EST::EST(const SpaceInformationPtr &si) : base::Planner(si, "EST")
{
    specs_.approximateSolutions = true;
    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    siC_ = si.get();

    Planner::declareParam<double>("range", this, &EST::setRange, &EST::getRange);
    Planner::declareParam<double>("goal_bias", this, &EST::setGoalBias, &EST::getGoalBias);
}

ompl::control::EST::~EST(void)
{
    freeMemory();
}

void ompl::control::EST::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::control::EST::clear(void)
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    pdf_.clear ();
}

void ompl::control::EST::freeMemory(void)
{
    for (Grid<MotionInfo>::iterator it = tree_.grid.begin(); it != tree_.grid.end() ; ++it)
    {
        for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
        {
            if (it->second->data[i]->state)
                si_->freeState(it->second->data[i]->state);
            if (it->second->data[i]->control)
                siC_->freeControl(it->second->data[i]->control);
            delete it->second->data[i];
        }
    }
}

bool ompl::control::EST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    // Initializing tree with start state(s)
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        addMotion(motion);
    }

    if (tree_.grid.size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return false;
    }

    // Ensure that we have a state sampler AND a control sampler
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    msg_.inform("Starting with %u states", tree_.size);

    Motion *solution = NULL;
    double   slndist = std::numeric_limits<double>::infinity();
    Motion  *rmotion = new Motion(siC_);
    bool      solved = false;

    while (!ptc())
    {
        // Select a state to expand the tree from
        Motion *existing = selectMotion();
        assert (existing);

        // sample a random state (with goal biasing) near the state selected for expansion
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rmotion->state);
        else
        {
            if (!sampler_->sampleNear(rmotion->state, existing->state, maxDistance_))
                continue;
        }

        // Extend a motion toward the state we just sampled
        unsigned int duration = controlSampler_->sampleTo(rmotion->control, existing->control,
                                                          existing->state, rmotion->state);

        // Propagate the system from the state selected for expansion using the control we
        // just sampled for the given duration.  Save the resulting state into rmotion->state.
        duration = siC_->propagateWhileValid(existing->state, rmotion->control, duration, rmotion->state);

        // If the system was propagated for a meaningful amount of time, save into the tree
        if (duration >= siC_->getMinControlDuration())
        {
            // create a motion to the resulting state
            Motion *motion = new Motion(siC_);
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rmotion->control);
            motion->steps = duration;
            motion->parent = existing;

            // save the state
            addMotion(motion);

            // Check if this state is the goal state, or improves the best solution so far
            double dist = 0.0;
            solved = goal->isSatisfied(motion->state, &dist);
            if (solved || dist < slndist)
            {
                slndist = dist;
                solution = motion;

                if (solved)
                    break;
            }
        }
    }

    bool addedSolution = false;

    // Constructing the solution path
    if (solution != NULL)
    {
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        PathControl *path = new PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        addedSolution = true;
        goal->addSolutionPath(base::PathPtr(path), !solved, slndist);
    }

    // Cleaning up memory
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    msg_.inform("Created %u states in %u cells", tree_.size, tree_.grid.size());

    return addedSolution;
}

ompl::control::EST::Motion* ompl::control::EST::selectMotion(void)
{
    GridCell* cell = pdf_.sample(rng_.uniform01());
    return cell && !cell->data.empty() ? cell->data[rng_.uniformInt(0, cell->data.size() - 1)] : NULL;
}

void ompl::control::EST::addMotion(Motion *motion)
{
    Grid<MotionInfo>::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    GridCell* cell = tree_.grid.getCell(coord);
    if (cell)
    {
        cell->data.push_back(motion);
        pdf_.update(cell->data.elem_, 1.0/cell->data.size());
    }
    else
    {
        cell = tree_.grid.createCell(coord);
        cell->data.push_back(motion);
        tree_.grid.add(cell);
        cell->data.elem_ = pdf_.add(cell, 1.0);
    }
    tree_.size++;
}

void ompl::control::EST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<MotionInfo> motions;
    tree_.grid.getContent(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
        for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
            data.recordEdge(motions[i][j]->parent ? motions[i][j]->parent->state : NULL, motions[i][j]->state);
}
