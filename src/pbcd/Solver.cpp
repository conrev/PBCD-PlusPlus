#include <pbcd/Solver.hpp>

void solve(pbcd::DeformableModel &model, Eigen::MatrixXd const &gravity, double timestep, uint32_t iterations, uint32_t substeps)
{
    const uint32_t num_iterations = iterations / substeps;
    double dt = timestep / static_cast<double>(substeps);
    auto const &constraints = model.constraints();
    auto const J = constraints.size();
    std::vector<double> lagrange_multipliers(J, 0.);

    for (uint32_t s = 0; s < substeps; ++s)
    {
        auto &v = model.velocities();
        auto &x = model.positions();

        const auto &m = model.masses();

        Eigen::MatrixX3d const a = gravity.array().colwise() / m.array();

        auto v_ex = v + dt * a;
        Eigen::MatrixXd p = x + dt * v_ex;

        // sequential gauss seidel type solve
        std::fill(lagrange_multipliers.begin(), lagrange_multipliers.end(), 0.0);
        for (auto n = 0u; n < num_iterations; ++n)
        {
            for (auto j = 0u; j < J; ++j)
            {
                auto const &constraint = constraints[j];
                constraint->project(p, m, lagrange_multipliers[j], dt);
            }
        }

        for (uint32_t i = 0; i < x.rows(); ++i)
        {
            // inertial velocity
            v.row(i) = (p.row(i) - x.row(i)) / dt;
            x.row(i) = p.row(i);
        }
    }
}