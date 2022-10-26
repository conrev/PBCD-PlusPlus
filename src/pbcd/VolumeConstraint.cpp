#include <pbcd/VolumeConstraint.hpp>
namespace pbcd
{
    double VolumeConstraint::evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const
    {
        return volume(positions) - m_rest_volume;
    }

    void VolumeConstraint::project(Eigen::MatrixXd &positions, const Eigen::VectorXd &masses, double &langrange_multiplier, const double dt) const
    {
        auto const v0 = indices()[0];
        auto const v1 = indices()[1];
        auto const v2 = indices()[2];
        auto const v3 = indices()[3];

        auto const w0 = 1. / masses(v0);
        auto const w1 = 1. / masses(v1);
        auto const w2 = 1. / masses(v2);
        auto const w3 = 1. / masses(v3);

        Eigen::RowVector3d const p0 = positions.row(v0);
        Eigen::RowVector3d const p1 = positions.row(v1);
        Eigen::RowVector3d const p2 = positions.row(v2);
        Eigen::RowVector3d const p3 = positions.row(v3);

        auto const C = evaluate(positions, masses);

        Eigen::RowVector3d const grad0 = (1. / 6.) * (p1 - p2).cross(p3 - p2);
        Eigen::RowVector3d const grad1 = (1. / 6.) * (p2 - p0).cross(p3 - p0);
        Eigen::RowVector3d const grad2 = (1. / 6.) * (p0 - p1).cross(p3 - p1);
        Eigen::RowVector3d const grad3 = (1. / 6.) * (p1 - p0).cross(p2 - p0);

        auto const weighted_sum_of_gradients = w0 * grad0.squaredNorm() + w1 * grad1.squaredNorm() +
                                               w2 * grad2.squaredNorm() + w3 * grad3.squaredNorm();

        if (weighted_sum_of_gradients < 1e-5)
            return;

        double const alpha_tilde = compliance() / (dt * dt);
        double const delta_lagrange =
            -(C + alpha_tilde * langrange_multiplier) / (weighted_sum_of_gradients + alpha_tilde);

        langrange_multiplier += delta_lagrange;
        positions.row(v0) += w0 * grad0 * delta_lagrange;
        positions.row(v1) += w1 * grad1 * delta_lagrange;
        positions.row(v2) += w2 * grad2 * delta_lagrange;
        positions.row(v3) += w3 * grad3 * delta_lagrange;
    }
}
