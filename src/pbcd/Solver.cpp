#include <pbcd/Solver.hpp>

void solve(pbcd::DeformableModel &model, Eigen::MatrixXd const &gravity, double timestep, uint32_t iterations, uint32_t substeps)
{
    const uint32_t num_iterations = iterations / substeps;
    double dt = timestep / static_cast<double>(substeps);

    for (uint32_t s = 0; s < substeps; ++s)
    {
        auto &v = model.velocities();
        auto &x = model.positions();

        const auto &m = model.masses();

        Eigen::MatrixX3d const a = gravity.array().colwise() / m.array();

        auto v_ex = v + dt * a;
        auto p = x + dt * v_ex;

        for (uint32_t i = 0; i < x.rows(); ++i)
        {
            // inertial velocity
            v.row(i) = (p.row(i) - x.row(i)) / dt;
            x.row(i) = p.row(i);
        }
    }
}