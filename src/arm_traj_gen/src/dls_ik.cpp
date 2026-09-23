#include "arm_traj_gen/dls_ik.hpp"
#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <cmath>
#include <algorithm>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"


// initialize the DLSIK class with the model and end-effector frame index

DLSIK::DLSIK(const pinocchio::Model& model, pinocchio::FrameIndex ee_frame)
    : model_(model), ee_frame_(ee_frame), data_(model)
{
    // Set default parameters for the DLSIK algorithm
    k_ = 0.1;          
    lambda_max_ = 0.1; 
    epsilon_ = 1e-6;   
    max_iter_ = 100;  
    tol_ = 1e-4;       
}





Eigen::Vector3d DLSIK::logSO3(const Eigen::Matrix3d& R)
{
    double trace_val = (R.trace()-1.0)/2.0;
    
    double theta = std::acos(std::clamp(trace_val,-1.0,1.0));
    if (std::abs(theta)<1e-7)
    {
        Eigen::Matrix3d skew = 0.5*(R-R.transpose());
        Eigen::Vector3d e;
        e << skew(2,1),skew(0,2),skew(1,0);
        return e;
    }
    else if (std::abs(theta-M_PI) < 1e-4)
    {
        Eigen::Matrix3d A =
            (R+Eigen::Matrix3d::Identity())/2.0;

        Eigen::Vector3d n;
        n(0)=std::sqrt(std::max(0.0, A(0, 0)));
        n(1)=std::sqrt(std::max(0.0, A(1, 1)));
        n(2)=std::sqrt(std::max(0.0, A(2, 2)));

        if (A(0,1)<0.0)
            n(1) = -n(1);
        if (A(0,2)<0.0)
            n(2) = -n(2);
        if (A(1,2)<0.0)
            n(2) = -n(2);

        return M_PI * n;
    }
    else 
    {
        Eigen::Matrix3d skew = (theta / (2.0 * std::sin(theta)))*(R - R.transpose());

        Eigen::Vector3d e;

        e << skew(2, 1),skew(0, 2),skew(1, 0);
        return e;
    }
}

Eigen::MatrixXd DLSIK::computeJacobian(const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(6, model_.nv);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacement(model_, data_, ee_frame_);
    pinocchio::getFrameJacobian(model_, data_, ee_frame_, pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}