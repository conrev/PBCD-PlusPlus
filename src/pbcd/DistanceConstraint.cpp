#include <pbcd/DistanceConstraint.hpp>

namespace pbcd
{

    double DistanceConstraint::evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const
    {
        const int index1 = indices()[0];
        const int index2 = indices()[1];

        return (positions.row(index1) - positions.row(index2)).norm() - m_rest_distance;
    }

    void DistanceConstraint::project(Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const
    {
        auto const &indices = this->indices();
        auto const v0 = indices.at(0);
        auto const v1 = indices.at(1);
        auto const p0 = positions.row(v0);
        auto const p1 = positions.row(v1);
        auto const w0 = 1. / masses(v0);
        auto const w1 = 1. / masses(v1);
        auto const n = (p0 - p1).normalized();
        auto const C = evaluate(positions, masses);

        double const weighted_sum_of_gradients = w0 + w1;
        double const alpha_tilde = compliance() / (dt * dt);
        double const delta_lagrange =
            -(C + alpha_tilde * langrange_multiplier) / (weighted_sum_of_gradients + alpha_tilde);

        langrange_multiplier += delta_lagrange;
        positions.row(v0) += w0 * n * delta_lagrange;
        positions.row(v1) += w1 * -n * delta_lagrange;
    }
}