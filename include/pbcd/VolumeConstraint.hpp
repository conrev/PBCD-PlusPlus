#ifndef VOLUME_CONSTRAINT_H
#define VOLUME_CONSTRAINT_H

#include <pbcd/Constraint.hpp>

namespace pbcd
{
    class VolumeConstraint : public Constraint
    {
    public:
        VolumeConstraint(const std::vector<int> &indices, const Eigen::MatrixXd &positions, double alpha) : Constraint(indices, alpha)
        {
            assert(indices.size() == 4);
            m_rest_volume = volume(positions);
        }

        double evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const override;

        virtual void project(Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const override;

        double volume(const Eigen::MatrixXd &positions) const
        {
            Eigen::RowVector3d const p0 = positions.row(indices()[0]);
            Eigen::RowVector3d const p1 = positions.row(indices()[1]);
            Eigen::RowVector3d const p2 = positions.row(indices()[2]);
            Eigen::RowVector3d const p3 = positions.row(indices()[3]);

            auto const vol = (1. / 6.) * (p1 - p0).cross(p2 - p0).dot(p3 - p0);
            return std::abs(vol);
        }

    private:
        double m_rest_volume;
    };

}

#endif