#ifndef SOLVER_H
#define SOLVER_H

#include <pbcd/DeformableModel.hpp>

void solve(pbcd::DeformableModel &model, Eigen::MatrixXd const &gravity, double timestep = 0.01, uint32_t iterations = 10, uint32_t substeps = 10);

#endif
