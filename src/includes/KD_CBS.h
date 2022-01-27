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
            // void interpolate(PathControl &p);
            
            /*method that checks for conflicts (collisions) within the plan*/
            std::vector <Conflict> validatePlan(Plan pl);

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
                }

                ~conflictNode() = default;
                // {
                //     printf("destructor called! \n");
                // }

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
            
            private:
                /** \brief The state contained by the motion */
                Plan plan_;

                /** \brief The parent motion in the exploration tree */
                const conflictNode *parent_{nullptr};

                /** \brief The total length (sum of cost) of the plan */
                double cost_{std::numeric_limits<double>::infinity()};

                /** \brief the constraint that the node was created to resolve*/
                const Constraint *constraint_{nullptr}; 
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

            // define the queue -- using multiset since it is more versatile than priority queue
            // using Queue = std::multiset<conflictNode*, LessThanNodeK>;

            
            /* Replan for agent agentIdx for new set of constraints c */
            void replanSingleAgent(conflictNode* &n, const int agentIdx, PathControl &solution)
            {
                /* SAVE -- HOW TO USE LOW-LEVEL SEARCH INSIDE KD-CBS
                // attempt to solve the problem within one second of planning time
                ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

                if (solved)
                {
                    // get the goal representation from the problem definition (not the same as the goal state)
                    // and inquire about the found path
                    oc::PathControl path = static_cast<oc::PathControl &>(*pdef->getSolutionPath());
                    std::cout << "Found solution:" << std::endl;
                    path.printAsMatrix(std::cout);
                }
                else
                    std::cout << "No solution found" << std::endl;
                */



                // SimpleSetup ss = mmpp_[agentIdx];
                // // get all agent specific constratints within a branch
                // std::vector<const Constraint*> c{};
                // const conflictNode *nCopy = n;
                // printf("Adding constraints to the following time range. \n");
                // while (nCopy->getParent())
                // {
                //     if (nCopy->getConstraint()->getAgent() == agentIdx)
                //     {
                //         std::cout << nCopy->getConstraint()->getTimeRange()->first << ", " << 
                //             nCopy->getConstraint()->getTimeRange()->second << std::endl;
                //         c.push_back(nCopy->getConstraint());
                //     }
                //     nCopy = nCopy->getParent();
                // }
                // printf("Done!\n");
                // std::cout << ss.getPlanner().get() << std::endl;
                // // delete old low-level planner
                // ss.getPlanner().get()->clear();
                // // create new planner
                // auto planner(std::make_shared<oc::constraintRRT>(ss.getSpaceInformation()));
                // // planner->provideAgent(a);
                // // planner->updateConstraints(c);
                // // ss.setPlanner(planner);
                // // ss.setup();
                // // ss.getPlanner()->setup();
                
                exit(1);
                // updateOMPLConstraints(ss, c, agentIdx);
                // // solve new problem
                // base::PlannerStatus solved = ss.solve(planningTime_);
                // if (solved)
                //     solution = ss.getSolutionPath();
            }

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

            double planningTime_{5};
        };
    }
}