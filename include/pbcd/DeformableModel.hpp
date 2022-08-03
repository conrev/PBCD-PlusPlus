#ifndef DEFORMABLEMODEL_H
#define DEFORMABLEMODEL_H

#include <Eigen/Geometry>

namespace pbcd
{
    class DeformableModel
    {

    public:
        DeformableModel();
        ~DeformableModel();

        DeformableModel(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            const Eigen::MatrixXd &C,
            const Eigen::MatrixX2i &BE) : m_restVerts(V),
                                          m_Verts(V),
                                          m_Faces(F),
                                          m_BoneVerts(C),
                                          m_BoneEdges(BE)
        {
        }

        private:
        Eigen::MatrixXd m_Verts;
        Eigen::MatrixXi m_Faces;
        Eigen::MatrixXd m_BoneVerts;
        Eigen::MatrixXi m_BoneEdges;
        Eigen::MatrixXd m_restVerts;
    };
}

#endif