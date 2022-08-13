#ifndef DEFORMABLEMODEL_H
#define DEFORMABLEMODEL_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <pbcd/AnimationClip.hpp>

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
            const Eigen::MatrixX2i &BE,
            const Eigen::MatrixXd &W);

        Eigen::MatrixXd &positions();
        Eigen::MatrixXi &faces();
        Eigen::MatrixXd &bonePositions();
        Eigen::MatrixX2i &boneEdges();
        std::vector<RotationList> &poses();

        void resetState();
        void playAnimationClip(const AnimationClip &_clip, double _t);

    private:
        Eigen::MatrixXd m_verts;
        Eigen::MatrixXi m_faces;
        Eigen::MatrixXd m_boneVerts;
        Eigen::MatrixX2i m_boneEdges;
        Eigen::MatrixXd m_restVerts;
        Eigen::VectorXi m_parentEdges;
        Eigen::MatrixXd m_vertWeights;
        RotationList m_restPose;
        std::vector<RotationList> m_poses;
        Eigen::MatrixXd m_LBSMatrix;
    };
}

#endif