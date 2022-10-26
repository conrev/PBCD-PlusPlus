#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Geometry>
#include <vector>

namespace pbcd
{
    class Constraint
    {
    public:
        Constraint(const std::vector<int> &indices, const double alpha) : m_indices(indices), m_alpha(alpha) {}
        virtual ~Constraint() = default;
        virtual double evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const = 0;

        virtual void project(Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const = 0;

        const std::vector<int> &indices() const { return m_indices; }
        const double &compliance() const { return m_alpha; }

    private:
        std::vector<int> m_indices;
        double m_alpha;
    };

}

#endif