#pragma once
#include "utils/Conflict.h"
#include "Constraints/BeliefConstraint.h"
#include "utils/MultiRobotProblemDefinition.h"
#include "Planners/ConstraintRespectingPlanner.h"
#include <boost/serialization/export.hpp>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/util/ClassForward.h>


namespace oc = ompl::control;
typedef std::vector<oc::PathControl> Plan;


OMPL_CLASS_FORWARD(ConstraintRespectingPlanner);
OMPL_CLASS_FORWARD(MultiRobotProblemDefinition);
namespace ompl
{
    namespace control
    {
        /** \brief Kinodynamic Conflict Based Search */
        class KCBS : public base::Planner
        {
        public:
            /** \brief Constructor */
            KCBS(const MultiRobotProblemDefinitionPtr mrmp_pdef);

            ~KCBS() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // /** \brief Clear datastructures. Call this function if the
            //     input data to the planner has changed and you do not
            //     want to continue planning */
            // // void clear() override;

            // /** In the process of randomly selecting states in the state
            //     space to attempt to go towards, the algorithm may in fact
            //     choose the actual goal state, if it knows it, with some
            //     probability. This probability is a real number between 0.0
            //     and 1.0; its value should usually be around 0.05 and
            //     should not be too large. It is probably a good idea to use
            //     the default value. */

            // // void getPlannerData(base::PlannerData &data) const override;

            // // void setup() override;

            // void setReplanAgent(int a) {replanningAgent_ = a;};

            // void resetReplanAgent() {replanningAgent_ = 0;};

            // void setInstance(Instance *mrmp_instance) {mrmp_instance_ = mrmp_instance;};
            
            // /*method that checks for conflicts (collisions) within the plan*/
            // std::vector <Conflict> validatePlan(Plan pl);

            // Plan lowLevelSearch(oc::ConstraintRRT* p, std::vector<const Constraint*> &constraints, const Plan parentPlan, bool replanning);

            // Instance* composeSystem(const int idx1, const int idx2);

            // const double getSolveTime() const {return solveTime_;};

            // void resetSolveTime() {solveTime_ = 0.0;};

            void setMergeBound(int b) {B_ = b;};

            void setLowLevelPlanningTime(const double t) {mp_comp_time_ = t;};

            // void resetComputationTime() {computation_time_ = 0;};

            double getComputationTime() const {return computation_time_;};

            double getSolutionSOC() const {return soc_;};

            // void performBypassing() {bypass_ = true;};

            // std::vector<std::pair<int, int>> getMergers() const {return merger_count_;};

        protected:
            /** \brief Representation of a conflict node

                This only contains pointers to parent nodes as we
                only need to go backwards in the tree. */
            class KCBSNode
            {
            public:
                int id=-1;
                KCBSNode(){};

                KCBSNode(KCBSNode const &n)
                {
                    // create a copy of conflict node
                    this->id = n.id;
                    this->plan_ = n.getPlan();
                    this->parent_ = n.getParent();
                    this->cost_ = n.getCost();
                    this->constraint_ = n.getConstraint();
                    this->planner_ = n.getPlanner();
                }

                ~KCBSNode()
                {
                    // motions_.clear();
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

                void printPlan()
                {
                    for (int r = 0; r < plan_.size(); r++) {
                       std::cout << "Agent " << r << std::endl;
                       plan_[r].print(std::cout);
                    }
                }

                // update parent node
                void updateParent(const KCBSNode *n) {parent_ = n;};

                // add constraint
                void addConstraint(ConstraintPtr constraint) {constraint_ = constraint;};

                // get the plan, but cannot change it
                Plan getPlan() const {return plan_;};

                // get the parent, but no not change it
                const KCBSNode* getParent() const {return parent_;};

                // get the cost, but do not alter it
                const double getCost() const {return cost_;};

                // get the constraint within a node
                ConstraintPtr getConstraint() const {return constraint_;};

                void savePlanner(PlannerPtr planner) {
                	planner_ = planner;
                }

                PlannerPtr getPlanner() const {return planner_;};

                // void fillMotions(std::vector<ConstraintRRT::Motion*> newMotions)
                // {
                //     motions_ = newMotions;
                // };

                // const std::vector<ConstraintRRT::Motion*> getMotions() const
                // {
                //     return motions_;
                // };

            private:
                /** The state contained by the motion */
                Plan plan_;

                /* The parent motion in the exploration tree */
                const KCBSNode* parent_{nullptr};

                /* The total length (sum of cost) of the plan */
                double cost_{std::numeric_limits<double>::infinity()};

                /* the constraint that the node was created to resolve */
                ConstraintPtr constraint_{nullptr};

                PlannerPtr planner_{nullptr};

                /* list of motions--only filled if node fails to create plan_ */
                // std::vector<ob::Planner::Motion*> motions_;
            };

            void freeMemory_();

            void setUp_();

            oc::PathControl* calcNewPath_(PlannerPtr planner, std::vector<ConstraintPtr> constraints, bool restart = true);

            bool shouldMerge_(
                std::vector< std::pair< std::pair<int, int>, int> > &conf_cntr, 
                const int agent1, const int agent2);

            // function that orders priority queue
            struct Compare
            {
                bool operator()(const KCBSNode n1, const KCBSNode n2) const
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

            // this is the Multi-agent motion planning problem definition
            const MultiRobotProblemDefinitionPtr mrmp_pdef_;
            
            std::vector<PlannerPtr> low_level_planners_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            // conflictNode *lastGoalNode_{nullptr};

            double computation_time_;

            double soc_;

            bool ready_;

            double mp_comp_time_{5};  // seconds

            double prop_step_size_{-1};

            int B_{100};

            bool bypass_{false};

            std::vector<std::pair<int, int>> merger_count_{};

            std::vector< std::pair< std::pair<int, int>, int>> conf_counter_;
        };
    }
}