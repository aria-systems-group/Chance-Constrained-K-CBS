#pragma once
#include "Instance.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;


class MultiRobotStateSpaceSVC : public ob::StateValidityChecker
{
public:
    MultiRobotStateSpaceSVC(const oc::SpaceInformationPtr &si, InstancePtr mrmp_instance, const Robot *r, std::string dyn);
 
    virtual bool isValid(const ob::State *state) const;
private:
    const ob::SpaceInformation *si_;
    InstancePtr mrmp_instance_;
    const Robot *robot_;
    const std::string dyn_;
};
