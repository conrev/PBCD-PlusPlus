#ifndef DEFORMABLEMODEL_H
#define DEFORMABLEMODEL_H

#include <igl/deform_skeleton.h>
#include <igl/directed_edge_parents.h>
#include <igl/directed_edge_orientations.h>
#include <igl/forward_kinematics.h>
#include <igl/lbs_matrix.h>
#include <igl/edges.h>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <pbcd/AnimationClip.hpp>
#include <pbcd/Constraint.hpp>
#include <pbcd/DistanceConstraint.hpp>

namespace pbcd
{
    class DeformableModel
    {

    public:
        DeformableModel() = default;
        ~DeformableModel();
        DeformableModel(
            const Eigen::MatrixXd &V,
            const Eigen::MatrixXi &F,
            const Eigen::MatrixXd &C,
            const Eigen::MatrixX2i &BE,
            const Eigen::MatrixXd &W) : m_restVerts(V),
                                        m_verts(V),
                                        m_faces(F),
                                        m_boneVerts(C),
                                        m_boneEdges(BE),
                                        m_vertWeights(W),
                                        m_velocities(V.rows(), V.cols()),
                                        m_masses(V.rows()),
                                        m_constraints{}
        {
            igl::directed_edge_parents(m_boneEdges, m_parentEdges);
            igl::directed_edge_orientations(m_boneVerts, m_boneEdges, m_restPose);
            igl::lbs_matrix(m_verts, m_vertWeights, m_LBSMatrix);
            m_masses.setOnes();
        }

        Eigen::MatrixXd &positions()
        {
            return m_verts;
        }
        Eigen::MatrixXi &faces()
        {
            return m_faces;
        }
        Eigen::MatrixXd &bonePositions()
        {
            return m_boneVerts;
        }
        Eigen::MatrixX2i &boneEdges()
        {
            return m_boneEdges;
        }
        Eigen::MatrixXd &velocities()
        {
            return m_velocities;
        }
        Eigen::VectorXd &masses()
        {
            return m_masses;
        }
        std::vector<RotationList> &poses()
        {
            return m_poses;
        }
        Eigen::MatrixXd &restPositions()
        {
            return m_restVerts;
        }
        std::vector<std::unique_ptr<Constraint>> &constraints()
        {
            return m_constraints;
        }
        void resetState();
        void playAnimationClip(const AnimationClip &_clip, double _t);
        void tetrahedralize();
        void initDistanceConstraint(double compliance);
        void initVolumeConstraint(double compliance);

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
        Eigen::MatrixXd m_velocities;
        Eigen::VectorXd m_masses;
        std::vector<std::unique_ptr<Constraint>> m_constraints;
    };
}

#endif