#pragma once
#include "Planners/ConstraintRespectingPlanner.h"
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/config/SelfConfig.h>
/** \brief Select a default nearest neighbor datastructure for the given space
*
* The default depends on the planning algorithm and the space the planner operates in:
* - If the space is a metric space and the planner is single-threaded,
*   then the default is ompl::NearestNeighborsGNATNoThreadSafety.
* - If the space is a metric space and the planner is multi-threaded,
*   then the default is ompl::NearestNeighborsGNAT.
* - If the space is a not a metric space,
*   then the default is ompl::NearestNeighborsSqrtApprox.
*/
namespace ompl
{
    namespace tools
    {
        template <typename _T>
        static NearestNeighbors<_T> *getDefaultNearestNeighbors(const ConstraintRespectingPlanner *planner)
        {
            const base::StateSpacePtr &space = planner->getSpaceInformation()->getStateSpace();
            const base::PlannerSpecs &specs = planner->getSpecs();
            if (space->isMetricSpace())
            {
                if (specs.multithreaded)
                    return new NearestNeighborsGNAT<_T>();
                return new NearestNeighborsGNATNoThreadSafety<_T>();
            }
            return new NearestNeighborsSqrtApprox<_T>();
        }
    }
}