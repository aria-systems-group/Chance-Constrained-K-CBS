/*********************************************************************
* ARIA SYSTEMS RESEARCH GROUP
* 
* The main header file of KD-CBS. 
* 
* KD-CBS works by planning individually for many agents, locating 
* conflicts (collisions) in the resulting plan, generating constraints
* based on those conflicts, and invoking the low-level planner to 
* resolve such conflicts. The process repeats till a valid plan is 
* found.
*********************************************************************/
 
/* Author: Justin Kottinger */

#pragma once
#include "../includes/World.h"
#include "../includes/Constraint.h"
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include "../includes/collisionChecking.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/tools/config/SelfConfig.h>
#include <math.h>
#include <stdlib.h>


namespace ompl
{
    namespace control
    {
        typedef std::vector<PathControl> Plan;

        /** \brief Kinodynamic Conflict Based Search */
        class KD_CBS : public base::Planner
        {
        public:
            /** \brief Constructor */
            KD_CBS(const std::vector<SimpleSetup> mmpp);

            ~KD_CBS() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            // void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */

            // void getPlannerData(base::PlannerData &data) const override;

            // void setup() override;

            void setReplanAgent(int a) {replanningAgent_ = a;};

            void resetReplanAgent() {replanningAgent_ = 0;};

            void setWorld(World *world) {w_ = world;};

            /* OMPL returns PathControl objects with different
            control durations. This makes it difficult to evaluate conflicts
            since time is not syncronized between agents. OMPL provides a PathControl
            method that interpolates the trajectories but the function 
            does not allow the user to use such interpolation. This method 
            borrows much of that code but is implemented in such a way that enables
            the user to use the interpolated trajectory. */
            void interpolate(PathControl &p);
            
            /*method that checks for conflicts (collisions) within the plan*/
            std::unordered_set <Conflict> validatePlan(Plan pl);

            /*method deletes outdated state validity function and 
            creates a new one with updated constraints c for simplesetup 
            object cooresponding to agentIdx */
            void updateConstraints(std::vector<int> c, int agentIdx)
            {
                SimpleSetup ss = (mmpp_)[agentIdx];
                ss.getStateValidityChecker()->~StateValidityChecker();
                ss.setStateValidityChecker(std::make_shared<isStateValid_2D_Test>
                    (ss.getSpaceInformation(), w_, w_->getAgents()[agentIdx], c));
            }

            /* Replan for agent agentIdx for new set of constraints c */
            PathControl replanSingleAgent(std::vector<int> c, int agentIdx, PathControl solution, bool &isUpdated)
            {
                SimpleSetup ss = mmpp_[agentIdx];
                updateConstraints(c, agentIdx);
                base::PlannerStatus solved = ss.solve(planningTime_);
                if (solved)
                {
                    solution = ss.getSolutionPath();
                    isUpdated = true;
                }
                return solution;
            }

        protected:
            /** \brief Representation of a conflict node

                This only contains pointers to parent nodes as we
                only need to go backwards in the tree. */
            class conflictNode
            {
            public:
                conflictNode() = default;

                ~conflictNode() = default;

                // update plan and cost at same time to avoid bad bookkeeping
                void updatePlanAndCost(Plan p) 
                {
                    plan_ = p; 
                    double total = 0;
                    for (PathControl traj: plan_)
                    {
                        const std::vector<double> times = traj.getControlDurations();
                        for (double dt: times)
                            total += dt;
                    }
                    cost_ = total;
                };

                // update parent node
                void updateParent(conflictNode* &c) {parent_ = c;};

                // get the plan, but cannot change it
                Plan getPlan() const {return plan_;};

                // get the parent, but no not change it
                const conflictNode* getParent() const {return parent_;};

                // get the cost, but do not alter it
                const double getCost() const {return cost_;};
            
            private:
                /** \brief The state contained by the motion */
                Plan plan_;

                /** \brief The parent motion in the exploration tree */
                conflictNode *parent_{nullptr};

                /** \brief The total length (sum of cost) of the plan */
                double cost_{std::numeric_limits<double>::infinity()};
            };

            // function that orders priority queue
            struct LessThanNodeK
            {
                bool operator()(const conflictNode *n1, const conflictNode *n2) const
                {
                    return n1->getCost() < n2->getCost();
                }
            };

            // define the queue -- using multiset since it is more versatile than priority queue
            using Queue = std::multiset<conflictNode*, LessThanNodeK>;

            // Queue utilities
            conflictNode* popHead()
            {
                conflictNode *n = *queue_.begin();
                queue_.erase(queue_.begin());
                return n;
            }

            /** \brief Free the memory allocated by this planner */
            // void freeMemory();

            /** \brief The vector of control::SpaceInformation, for convenience */
            // this is the Multi-agent motion planning problem
            std::vector<SimpleSetup> mmpp_;

            Queue queue_;

            int replanningAgent_{0};

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            conflictNode *lastGoalNode_{nullptr};

            World *w_{nullptr};

            double planningTime_{1};
        };
    }
}