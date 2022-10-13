#include <pbcd/DeformableModel.hpp>

namespace pbcd
{
    void DeformableModel::resetState()
    {
        m_verts = m_restVerts;
    }

    void DeformableModel::playAnimationClip(const AnimationClip &_clip, double _t)
    {
        // For now, assume that each poses is one second apart from each other
        // select 2 poses that should be playing according to _t
        const int startPoseIndex = (int)floor(_t) % _clip.posesList.size();
        const int endPoseIndex = ((int)floor(_t) + 1) % _clip.posesList.size();
        const double completion = _t - (int)floor(_t);

        // Interpolate between 2 poses
        RotationList anim_pose(_clip.posesList[startPoseIndex].size());
        for (int e = 0; e < _clip.posesList[startPoseIndex].size(); e++)
        {
            anim_pose[e] = _clip.posesList[startPoseIndex][e].slerp(completion, _clip.posesList[endPoseIndex][e]);
        }
        // Propagate relative rotations via FK to retrieve absolute transformations
        RotationList vQ;
        std::vector<Eigen::Vector3d> vT;

        igl::forward_kinematics(m_boneVerts, m_boneEdges, m_parentEdges, anim_pose, vQ, vT);
        const int dim = m_boneVerts.cols();
        Eigen::MatrixXd T(m_boneEdges.rows() * (dim + 1), dim);
        for (int e = 0; e < m_boneEdges.rows(); e++)
        {
            Eigen::Affine3d a = Eigen::Affine3d::Identity();
            a.translate(vT[e]);
            a.rotate(vQ[e]);
            T.block(e * (dim + 1), 0, dim + 1, dim) =
                a.matrix().transpose().block(0, 0, dim + 1, dim);
        }
        // Compute deformation via LBS as matrix multiplication
        m_verts = m_LBSMatrix * T;

        // Also deform skeleton edges
        Eigen::MatrixXd CT;
        Eigen::MatrixXi BET;

        igl::deform_skeleton(m_boneVerts, m_boneEdges, T, CT, BET);
    }

    void DeformableModel::tetrahedralize()
    {
    }
}