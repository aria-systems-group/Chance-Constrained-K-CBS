/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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
/* Author: Ali-akbar Agha-mohammadi, Saurav Agarwal */

#ifndef UNICYCLE_STATE_PROPAGATOR_
#define UNICYCLE_STATE_PROPAGATOR_

// #include "SpaceInformation/SpaceInformation.h"
#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "Spaces/RealVectorBeliefSpace.h"
// #include "../Spaces/R2BeliefSpaceEuclidean.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;
// typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Mat44;
// typedef Eigen::Matrix<double, 2, 4, Eigen::DontAlign> Mat24;
// typedef Eigen::Matrix<double, 4, 2, Eigen::DontAlign> Mat42;

/** \brief State propagation for a Unicycle motion model. */
class DynUnicycleControlSpace : public oc::StatePropagator {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DynUnicycleControlSpace(const oc::SpaceInformationPtr &si);

        virtual ~DynUnicycleControlSpace(void)
        {

        }

        virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const;

        virtual bool canPropagateBackward(void) const;

    private:


        double duration_;
        std::vector<double> controller_parameters_, forward_acceleration_bounds_, turning_rate_bounds_, surge_bounds_, system_noise_;
        // Eigen::MatrixXd A_ol_, B_ol_, Pw_22_, K_, A_cl_, A_cl_d_;

        Eigen::Matrix4d A_ol_;
        Eigen::Matrix<double, 4, 2> B_ol_;
        // Eigen::Matrix2d  Pw_22_;
        Eigen::Matrix<double, 2, 4> K_;

        Eigen::Matrix4d A_cl_, A_cl_d_;
        
        // Eigen::Matrix2d A_cl_d_22_;


        Eigen::Matrix2d B_cl_d_;

        // mutable double x_pose, x_pose_reference, y_pose, y_pose_reference;

        // mutable const R2BeliefSpace::StateType *start_css;

        // mutable const ob::CompoundStateSpace::StateType *start_css;

        // mutable const oc::CompoundControlSpace::ControlType *control_css;

        // mutable oc::RealVectorControlSpace::ControlType *control_css;

        // const Eigen::Vector2d start_css_rvs_pose;
        // const Eigen::Matrix2d start_css_rvs_cov;

        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        // Eigen::Matrix2d I = Eigen::MatrixXd::Identity(2, 2);
        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        

        Eigen::Matrix4d Q, R, Ak;
        double myK_ = 0.3;

        mutable const RealVectorBeliefSpace::StateType *result_css;
        ob::RealVectorStateSpace::StateType *result_css_rvs_pose;
        Eigen::Matrix2d result_css_rvs_cov;

        mutable double u_bar_0, u_bar_1, surge_final;

    protected:

        int dimensions_ = 2;


    /**
    You can add a simulated environment here where the controls can get applied, useful for
    showing the graphics, very similar to the concept of ActuationSystem in PMPL.
    */
};

#endif