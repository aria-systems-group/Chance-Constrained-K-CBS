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
#include "World.h"
#include "Constraint.h"
#include "constraintRRT.h"
#include "collisionChecking.h"
#include "multiAgentSetUp.h"
#include "postProcess.h"
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/tools/config/SelfConfig.h>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <stdlib.h>
#include <memory.h>
#include <vector>
#include <chrono>

typedef std::pair< std::shared_ptr<oc::SpaceInformation>, 
        std::shared_ptr<ob::ProblemDefinition> > problem;


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
            KD_CBS(const std::vector<problem> mmpp);

            ~KD_CBS() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const 
                base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            // void clear() override;

            void freeMemory();

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
            
            /*method that checks for conflicts (collisions) within the plan*/
            std::vector <Conflict> validatePlan(Plan pl);

            bool shouldMerge(
                std::vector< std::pair< std::pair<int, int>, int> > &conf_cntr, 
                const int agent1, const int agent2);

            World* composeSystem(const int idx1, const int idx2);

            const double getSolveTime() const {return solveTime_;};

            void resetSolveTime() {solveTime_ = 0.0;};

        protected:
            /** \brief Representation of a conflict node

                This only contains pointers to parent nodes as we
                only need to go backwards in the tree. */
            class conflictNode
            {
            public:
                conflictNode() = default;

                conflictNode(conflictNode const &c)
                {
                    // create a copy of conflict node
                    this->plan_ = c.getPlan();
                    this->parent_ = c.getParent();
                    this->cost_ = c.getCost();
                    this->constraint_ = c.getConstraint();
                    this->motions_ = c.getMotions();
                }

                ~conflictNode()
                {
                    // printf("in here\n");
                    motions_.clear();
                    plan_.clear();
                    // delete constraint_;
                    // constraint_ = nullptr;
                }

                // update plan and cost at same time to avoid bad bookkeeping
                void updatePlanAndCost(Plan &p)
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
                void updateParent(const conflictNode *c) {parent_ = c;};

                // add constraint
                void addConstraint(Constraint *constraint) {constraint_ = constraint;};

                // get the plan, but cannot change it
                Plan getPlan() const {return plan_;};

                // get the parent, but no not change it
                const conflictNode* getParent() const {return parent_;};

                // get the cost, but do not alter it
                const double getCost() const {return cost_;};

                // get the constraint within a node
                const Constraint* getConstraint() const {return constraint_;};

                void fillMotions(std::vector<constraintRRT::Motion*> newMotions)
                {
                    motions_ = newMotions;
                };

                const std::vector<constraintRRT::Motion*> getMotions() const
                {
                    return motions_;
                };

            private:
                /** The state contained by the motion */
                Plan plan_;

                /* The parent motion in the exploration tree */
                const conflictNode *parent_{nullptr};

                /* The total length (sum of cost) of the plan */
                double cost_{std::numeric_limits<double>::infinity()};

                /* the constraint that the node was created to resolve */
                const Constraint *constraint_{nullptr};

                /* list of motions--only filled if node fails to create plan_ */
                std::vector<constraintRRT::Motion*> motions_;
            };

            // function that orders priority queue
            struct Compare
            {
                bool operator()(const conflictNode n1, const conflictNode n2) const
                {
                    /* Note that the Compare parameter is defined such 
                    that it returns true if its first argument comes before 
                    its second argument in a weak ordering. But because the 
                    priority queue outputs largest elements first, the 
                    elements that "come before" are actually output last. 
                    That is, the front of the queue contains the "last" 
                    element according to the weak ordering imposed by Compare. */
                    return n1.getCost() > n2.getCost();
                }
            };

            /** \brief Free the memory allocated by this planner */
            // void freeMemory();

            /** \brief The vector of control::SpaceInformation, for convenience */
            // this is the Multi-agent motion planning problem
            const std::vector<problem> mmpp_;
            // std::vector<oc::constraintRRT*>

            /* Flag that tracks which agent we are replanning for */
            int replanningAgent_{0};

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            conflictNode *lastGoalNode_{nullptr};

            World *w_{nullptr};

            double planningTime_{3};  // seconds

            const int B_{10};

            double solveTime_{0.0};
        };
    }
}