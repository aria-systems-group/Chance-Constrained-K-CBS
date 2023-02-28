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

#pragma once
#include "ompl/control/SpaceInformation.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
// #include "../Spaces/R2BeliefSpace.h"
#include "Spaces/RealVectorBeliefSpace.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Mat;
// typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Mat44;
// typedef Eigen::Matrix<double, 2, 4, Eigen::DontAlign> Mat24;
// typedef Eigen::Matrix<double, 4, 2, Eigen::DontAlign> Mat42;

/** \brief State propagation for a Unicycle motion model. */
class UncertainUnicycleStatePropagator : public oc::StatePropagator {
    public:
        UncertainUnicycleStatePropagator(const oc::SpaceInformationPtr &si);

        void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

        bool canPropagateBackward(void) const override;

    private:
        double duration_;
        // int dimensions_ = 2;
        std::vector<double> controller_parameters_{0.0316, 0.3054, 0.0, 0.0, 0.0, 0.0, 0.3054, 0.0316};
        std::vector<double> forward_acceleration_bounds_{-0.5, 0.5};
        std::vector<double> turning_rate_bounds_{-0.5, 0.5};
        std::vector<double> surge_bounds_{0.05, 5.0};
        std::vector<double> system_noise_{0.1, 0.0, 0.0, 0.1};

        
        Eigen::Matrix4d A_cl_;
        Eigen::Matrix4d B_cl_;
        Eigen::Matrix4d A_cl_d_;
        Eigen::Matrix4d B_cl_d_;
        Eigen::Matrix2d A_cl_d_22_;

        // Mat44 A_ol_;
        // Mat42 B_ol_;
        // Eigen::Matrix2d  Pw_22_;
        // Mat24 K_;

        // Eigen::Matrix4d A_cl_, A_cl_d_;
        
        // Eigen::Matrix2d A_cl_d_22_;


        // Eigen::Matrix2d B_cl_d_;

        // mutable double x_pose, x_pose_reference, y_pose, y_pose_reference;

        // mutable const R2BeliefSpace::StateType *start_css;

        // mutable const ob::CompoundStateSpace::StateType *start_css;

        // mutable const oc::CompoundControlSpace::ControlType *control_css;

        // mutable oc::RealVectorControlSpace::ControlType *control_css;

        // const Eigen::Vector2d start_css_rvs_pose;
        // const Eigen::Matrix2d start_css_rvs_cov;
        Eigen::Matrix2d I_ = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d H_ = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d F_ = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d Q_ = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d R_ = Eigen::Matrix2d::Identity();
        

        // Eigen::Matrix2d Q, R, Ak;
        double K_ = 0.3;

        // mutable const R2BeliefSpace::StateType *result_css;
        // ob::RealVectorStateSpace::StateType *result_css_rvs_pose;
        // Eigen::Matrix2d result_css_rvs_cov;

        // mutable double u_bar_0, u_bar_1, surge_final;

        
};
