#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <pinocchio/multibody/model.hpp>

// Position limits per named joint, taken from a URDF. Validation rejects; it never clamps.
class JointLimits
{
    public:
        // Throws std::runtime_error if the model has no 1-DoF joints with limits.
        static JointLimits fromModel(const pinocchio::Model& model);
        // Throws if the URDF cannot be loaded or has no limits.
        static JointLimits fromUrdf(const std::string& urdf_path);

        // Returns true if every start and target position is within limits.
        // Otherwise returns false and describes the first violation in reason.
        bool check(const std::vector<std::string>& names,
                   const Eigen::VectorXd& q_start,
                   const Eigen::VectorXd& q_end,
                   std::string& reason) const;

        std::size_t size() const { return limits_.size(); }

    private:
        std::map<std::string, std::pair<double, double>> limits_;
};
