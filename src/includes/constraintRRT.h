/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
 
/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_CONSTRAINTRRT_
#define OMPL_CONTROL_CONSTRAINTRRT_

 
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include "Constraint.h"
#include "World.h"
 
namespace ompl
{
    namespace control
    {
        class constraintRRT : public base::Planner
        {
        public:
            constraintRRT(const SpaceInformationPtr &si);
 
            ~constraintRRT() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
  
            void clear() override;
 
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }
 
            double getGoalBias() const
            {
                return goalBias_;
            }
 
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }
 
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }
 
            void getPlannerData(base::PlannerData &data) const override;
 
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }
 
            void setup() override;

            void provideAgent(Agent *a) {a_ = a;};

            // TODO: need an update constraints function that can be called from KD-CBS
            void updateConstraints(const std::vector<const Constraint*> c);
            // {
            //     for (const Constraint *con: c)
            //         constraints_.push_back(con);
            // }
 
        protected:
            class Motion
            {
            public:
                Motion() = default;
 
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }
 
                ~Motion() = default;
 
                base::State *state{nullptr};
 
                Control *control{nullptr};
 
                unsigned int steps{0};
 
                Motion *parent{nullptr};
            };
 
            void freeMemory();
 
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            bool satisfiesConstraints(const Motion *n) const;

            Agent *a_{nullptr};

            std::vector<const Constraint*> constraints_ = {};

            base::StateSamplerPtr sampler_;
 
            DirectedControlSamplerPtr controlSampler_;
 
            const SpaceInformation *siC_;
 
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
 
            double goalBias_{0.05};
 
            bool addIntermediateStates_{false};
 
            RNG rng_;
 
            Motion *lastGoalMotion_{nullptr};
        };
    }
}
#endif
