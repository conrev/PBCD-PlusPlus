#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include <pbcd/Constraint.hpp>

namespace pbcd
{
    class DistanceConstraint : public Constraint
    {
    public:
        DistanceConstraint(const std::vector<int> &indices, const Eigen::MatrixXd &positions, double alpha) : Constraint(indices, alpha), m_rest_distance(0.0)
        {
            assert(indices.size() == 2);
            const auto index1 = this->indices()[0];
            const auto index2 = this->indices()[1];

            m_rest_distance = (positions.row(index1) - positions.row(index2)).norm();
        }

        virtual double evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const override;

        virtual void project(Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const override;

    private:
        double m_rest_distance;
    };

}

#endif