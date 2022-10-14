#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Geometry>
#include <vector>

namespace pbcd
{
    class Constraint
    {
    public:
        Constraint() = default;
        Constraint(const std::vector<int> &indices, const double alpha) : m_indices(indices), m_alpha(alpha) {}

        virtual double evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const = 0;

        virtual void project(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const = 0;

        const std::vector<int> &indices() const { return m_indices; }

    private:
        std::vector<int> m_indices;
        double m_alpha;
    };

}

#endif