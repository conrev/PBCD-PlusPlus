#include <pbcd/DeformableModel.hpp>
#include <igl/directed_edge_parents.h>
#include <iostream>

namespace pbcd
{

    DeformableModel::DeformableModel()
    {
    }

    DeformableModel::DeformableModel(
        const Eigen::MatrixXd &V,
        const Eigen::MatrixXi &F,
        const Eigen::MatrixXd &C,
        const Eigen::MatrixX2i &BE) : m_restVerts(V),
                                      m_Verts(V),
                                      m_Faces(F),
                                      m_BoneVerts(C),
                                      m_BoneEdges(BE)
    {
        igl::directed_edge_parents(DeformableModel::m_BoneEdges, DeformableModel::m_ParentEdges);
    }

    Eigen::MatrixXd &DeformableModel::positions() { return DeformableModel::m_Verts; }
    Eigen::MatrixXi &DeformableModel::faces() { return DeformableModel::m_Faces; }

}
