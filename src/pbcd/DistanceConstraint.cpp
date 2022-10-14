#include <pbcd/DistanceConstraint.h>

namespace pbcd
{

    double DistanceConstraint::evaluate(const Eigen::MatrixXd &positions, const Eigen::VectorXd &masses) const
    {
        const int index1 = indices()[0];
        const int index2 = indices()[1];

        return (positions.row(index1) - positions.row(index2)).norm() - m_rest_distance;
    }

}