#include "arm_traj_gen/joint_limits.hpp"

#include <cstdio>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>

JointLimits JointLimits::fromModel(const pinocchio::Model& model)
{
    JointLimits limits;

    for (pinocchio::JointIndex i = 1;
         i < static_cast<pinocchio::JointIndex>(model.njoints); ++i)
    {
        const auto& joint = model.joints[i];
        if (joint.nq() != 1)
        {
            continue;
        }
        limits.limits_[model.names[i]] = {
            model.lowerPositionLimit[joint.idx_q()],
            model.upperPositionLimit[joint.idx_q()]};
    }

    if (limits.limits_.empty())
    {
        throw std::runtime_error("No joint limits found in model.");
    }
    return limits;
}

JointLimits JointLimits::fromUrdf(const std::string& urdf_path)
{
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    return fromModel(model);
}

bool JointLimits::check(const std::vector<std::string>& names,
                        const Eigen::VectorXd& q_start,
                        const Eigen::VectorXd& q_end,
                        std::string& reason) const
{
    if (static_cast<Eigen::Index>(names.size()) != q_start.size() ||
        q_start.size() != q_end.size())
    {
        reason = "Joint names and positions have different sizes.";
        return false;
    }

    char buf[160];
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        const auto limit = limits_.find(names[i]);
        if (limit == limits_.end())
        {
            reason = "No URDF limits for joint '" + names[i] + "'.";
            return false;
        }

        const double lower = limit->second.first;
        const double upper = limit->second.second;

        if (!(q_start[i] >= lower && q_start[i] <= upper))  // also rejects NaN
        {
            std::snprintf(buf, sizeof(buf), "%s start %.4f outside limits [%.4f, %.4f].",
                          names[i].c_str(), q_start[i], lower, upper);
            reason = buf;
            return false;
        }
        if (!(q_end[i] >= lower && q_end[i] <= upper))
        {
            std::snprintf(buf, sizeof(buf), "%s target %.4f outside limits [%.4f, %.4f].",
                          names[i].c_str(), q_end[i], lower, upper);
            reason = buf;
            return false;
        }
    }
    return true;
}
