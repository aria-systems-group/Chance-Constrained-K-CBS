#pragma once
#include "Constraints/Constraint.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <boost/concept_check.hpp>
#include <thread>
#include <memory>

namespace ob = ompl::base;
namespace oc = ompl::control;


OMPL_CLASS_FORWARD(ConstraintRespectingPlanner);
OMPL_CLASS_FORWARD(PlanValidityChecker);
// namespace ompl
// {
//     namespace control
//     {
//         class PlannerInputStates
//         {
//         public:
//             /** \brief Default constructor. No work is performed. */
//             PlannerInputStates(const ConstraintRespectingPlannerPtr &planner) : planner_(planner.get())
//             {
//                 tempState_ = nullptr;
//                 update();
//             }

//             /** \brief Default constructor. No work is performed. */
//             PlannerInputStates(const ConstraintRespectingPlanner *planner) : planner_(planner)
//             {
//                 tempState_ = nullptr;
//                 update();
//             }

//             /** \brief Default constructor. No work is performed. A
//                 call to use() needs to be made, before making any
//                 calls to nextStart() or nextGoal(). */
//             PlannerInputStates()
//             {
//                 tempState_ = nullptr;
//                 clear();
//             }

//             /** \brief Destructor. Clear allocated memory. */
//             ~PlannerInputStates()
//             {
//                 clear();
//             }

//             /** \brief Clear all stored information. */
//             void clear();

//             /** \brief Forget how many states were returned by
//                 nextStart() and nextGoal() and return all states
//                 again */
//             void restart();

//             /** \brief Set the space information and problem
//                 definition this class operates on, based on the
//                 available planner instance. Returns true if changes
//                 were found (different problem definition) and clear()
//                 was called. */
//             bool update();

//             /** \brief Set the problem definition this class operates on.
//                 If a planner is not set in the constructor argument, a call
//                 to this function is needed before any calls to nextStart()
//                 or nextGoal() are made. Returns true if changes were found
//                 (different problem definition) and clear() was called. */
//             bool use(const ob::ProblemDefinitionPtr &pdef);

//             /** \brief Check if the problem definition was set, start
//                 state are available and goal was set */
//             void checkValidity() const;

//             /** \brief Return the next valid start state or nullptr if no
//                 more valid start states are available. */
//             const ob::State *nextStart();

//             /** \brief Return the next valid goal state or nullptr if no
//                 more valid goal states are available.  Because
//                 sampling of goal states may also produce invalid
//                 goals, this function takes an argument that specifies
//                 whether a termination condition has been reached.  If
//                 the termination condition evaluates to true the
//                 function terminates even if no valid goal has been
//                 found. */
//             const ob::State *nextGoal(const ob::PlannerTerminationCondition &ptc);

//             /** \brief Same as above but only one attempt is made to find a valid goal. */
//             const ob::State *nextGoal();

//             /** \brief Check if there are more potential start states */
//             bool haveMoreStartStates() const;

//             /** \brief Check if there are more potential goal states */
//             bool haveMoreGoalStates() const;

//             /** \brief Get the number of start states from the problem
//                 definition that were already seen, including invalid
//                 ones. */
//             unsigned int getSeenStartStatesCount() const
//             {
//                 return addedStartStates_;
//             }

//             /** \brief Get the number of sampled goal states, including invalid ones */
//             unsigned int getSampledGoalsCount() const
//             {
//                 return sampledGoalsCount_;
//             }

//         private:
//             const ConstraintRespectingPlanner *planner_{nullptr};

//             unsigned int addedStartStates_;
//             unsigned int sampledGoalsCount_;
//             ob::State *tempState_;

//             ob::ProblemDefinitionPtr pdef_;
//             const ob::SpaceInformation *si_;
//         };
//     }
// }

/** \brief Base class for a planner */
class ConstraintRespectingPlanner: public ob::Planner
{
public:
    /** \brief Constructor */
    ConstraintRespectingPlanner(ob::SpaceInformationPtr si, std::string name);

    // // add my things required for planning w/ KCBS
    // virtual void setPlanValidator(PlanValidityCheckerPtr &validator) = 0;

    // virtual const PlanValidityCheckerPtr getPlanValidator() = 0;
    
    // virtual void updateConstraints(std::vector<ConstraintPtr> c) = 0;

    // add my things required for planning w/ KCBS
    void setPlanValidator(PlanValidityCheckerPtr &validator)
    {
        planValidator_ = validator;
    }

    const PlanValidityCheckerPtr getPlanValidator() 
    {
        return planValidator_;
    }

    void updateConstraints(std::vector<ConstraintPtr> c) 
    {
        /* clear old data */
        clear();
        /* clear old solutions */
        pdef_->clearSolutionPaths();
        /* update constraints */
        constraints_ = c;
    }

protected:
    // my additions for replanning w. KCBS
    PlanValidityCheckerPtr planValidator_;
    std::vector<ConstraintPtr> constraints_;
};

