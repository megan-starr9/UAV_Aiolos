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

/* Author: Matt Maly */

#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"

void ompl::control::SyclopRRT::setup(void)
{
    Syclop::setup();
    sampler_ = si_->allocStateSampler();
    controlSampler_ = siC_->allocDirectedControlSampler();

    // Create a default GNAT nearest neighbors structure if the user doesn't want
    // the default regionalNN check from the discretization
    if (!nn_ && !regionalNN_)
    {
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
        nn_->setDistanceFunction(boost::bind(&SyclopRRT::distanceFunction, this, _1, _2));
    }
}

void ompl::control::SyclopRRT::clear(void)
{
    Syclop::clear();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::control::SyclopRRT::getPlannerData(base::PlannerData& data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);
    if (PlannerData *cpd = dynamic_cast<control::PlannerData*>(&data))
    {
        const double delta = siC_->getPropagationStepSize();

        for (std::vector<Motion*>::const_iterator i = motions.begin(); i != motions.end(); ++i)
        {
            const Motion* m = *i;
            if (m->parent)
                cpd->recordEdge(m->parent->state, m->state, m->control, m->steps * delta);
            else
                cpd->recordEdge(NULL, m->state, NULL, 0.);
        }
    }
    else
    {
        for (std::vector<Motion*>::const_iterator i = motions.begin(); i != motions.end(); ++i)
        {
            const Motion* m = *i;
            data.recordEdge(m->parent ? m->parent->state : NULL, m->state);
        }
    }
}

ompl::control::Syclop::Motion* ompl::control::SyclopRRT::addRoot(const base::State* s)
{
    Motion* motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);

    if (nn_)
        nn_->add(motion);
    return motion;
}

void ompl::control::SyclopRRT::selectAndExtend(Region& region, std::vector<Motion*>& newMotions)
{
    Motion* rmotion = new Motion(siC_);
    base::StateSamplerPtr sampler(si_->allocStateSampler());
    decomp_->sampleFromRegion(region.index, sampler, rmotion->state);

    Motion* nmotion;
    if (regionalNN_)
    {
        /* Instead of querying the nearest neighbors datastructure over the entire tree of motions,
         * here we perform a linear search over all motions in the selected region and its neighbors. */
        std::vector<int> searchRegions;
        decomp_->getNeighbors(region.index, searchRegions);
        searchRegions.push_back(region.index);

        std::vector<Motion*> motions;
        for (std::vector<int>::const_iterator i = searchRegions.begin(); i != searchRegions.end(); ++i)
        {
            const std::vector<Motion*>& regionMotions = getRegionFromIndex(*i).motions;
            motions.insert(motions.end(), regionMotions.begin(), regionMotions.end());
        }

        std::vector<Motion*>::const_iterator i = motions.begin();
        nmotion = *i;
        double minDistance = distanceFunction(rmotion, nmotion);
        ++i;
        while (i != motions.end())
        {
            Motion* m = *i;
            const double dist = distanceFunction(rmotion, m);
            if (dist < minDistance)
            {
                nmotion = m;
                minDistance = dist;
            }
            ++i;
        }
    }
    else
    {
        assert (nn_);
        nmotion = nn_->nearest(rmotion);
    }

    base::State* newState = si_->allocState();

    unsigned int duration = controlSampler_->sampleTo(rmotion->control, nmotion->control, nmotion->state, rmotion->state);

    duration = siC_->propagateWhileValid(nmotion->state, rmotion->control, duration, newState);

    if (duration >= siC_->getMinControlDuration())
    {
        Motion* motion = new Motion(siC_);
        si_->copyState(motion->state, newState);
        siC_->copyControl(motion->control, rmotion->control);
        motion->steps = duration;
        motion->parent = nmotion;
        newMotions.push_back(motion);
        if (nn_)
            nn_->add(motion);
    }

    si_->freeState(rmotion->state);
    siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(newState);
}

void ompl::control::SyclopRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (std::vector<Motion*>::iterator i = motions.begin(); i != motions.end(); ++i)
        {
            Motion* m = *i;
            if (m->state)
                si_->freeState(m->state);
            if (m->control)
                siC_->freeControl(m->control);
            delete m;
        }
    }
}
