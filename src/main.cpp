#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readOBJ.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/bbw.h>
#include <igl/Timer.h>
#include <igl/boundary_conditions.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include <pbcd/DeformableModel.hpp>
#include <pbcd/Solver.hpp>

const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);
pbcd::DeformableModel *body;
Eigen::MatrixXd gravity;
Eigen::MatrixXd V, W, C, U, M, TV;
Eigen::MatrixXi F, TF, TT, BE;
Eigen::VectorXi P;
pbcd::AnimationClip clip;
double anim_t = 0.0;
double anim_t_dir = 0.015;
bool recompute = true;
igl::Timer timer;

bool pre_draw(igl::opengl::glfw::Viewer &viewer)
{
  using namespace Eigen;
  using namespace std;
  if (recompute)
  {
    timer.start();

    // body->playAnimationClip(clip, anim_t);

    if (viewer.core().is_animating)
    {
      // cout << anim_t << endl;
      solve(*body, gravity, 0.01, 10, 10);

      viewer.data().set_vertices(body->positions());
      // viewer.data().set_edges(body->bonePositions(), body->boneEdges(), sea_green);
      viewer.data().compute_normals();
    }
    else
    {
      recompute = false;
    }
  }
  return false;
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
{
  recompute = true;
  switch (key)
  {
  case ' ':
    viewer.core().is_animating = !viewer.core().is_animating;
    return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;

  igl::readOBJ("../models/arm.obj", V, F);
  U = V;
  igl::readTGF("../models/arm.tgf", C, BE);
  // retrieve parents for forward kinematics
  igl::directed_edge_parents(BE, P);
  RotationList rest_pose;
  igl::directed_edge_orientations(C, BE, rest_pose);

  // string settings = "pq1.414Y";
  // igl::copyleft::tetgen::tetrahedralize(V, F, "pq1.414", TV, TT, TF);
  // // // List of boundary indices (aka fixed value indices into VV)
  // VectorXi b;
  // // // List of boundary conditions of each weight function
  // MatrixXd bc;
  // igl::boundary_conditions(TV, TT, C, VectorXi(), BE, MatrixXi(), b, bc);

  // // compute BBW weights matrix
  // igl::BBWData bbw_data;
  // // // only a few iterations for sake of demo
  // bbw_data.active_set_params.max_iter = 8;
  // bbw_data.verbosity = 2;
  // if (!igl::bbw(TV, TT, b, bc, bbw_data, W))
  // {
  //   return EXIT_FAILURE;
  // }

  // // Normalize weights to sum to one
  // igl::normalize_row_sums(W, W);
  // cout << W << endl;

  igl::readDMAT("../models/arm-weights.dmat", W);

  body = new pbcd::DeformableModel(V, F, C, BE, W);

  gravity.resizeLike(body->positions());
  gravity.setZero();
  gravity.col(1).array() -= 9.81;

  std::cout << "The matrix m is of size "
            << gravity.rows() << "x" << gravity.cols() << std::endl;

  std::vector<RotationList>
      poses;
  poses.resize(4, RotationList(4, Quaterniond::Identity()));
  // poses[1] // twist
  const Quaterniond twist(AngleAxisd(igl::PI, Vector3d(1, 0, 0)));
  poses[1][3] = rest_pose[3] * twist * rest_pose[3].conjugate();
  const Quaterniond bend(AngleAxisd(-igl::PI * 0.7, Vector3d(0, 0, 1)));
  poses[3][3] = rest_pose[3] * bend * rest_pose[3].conjugate();

  clip.posesList = poses;

  // Plot the mesh with pseudocolors
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(body->positions(), body->faces());
  viewer.data().set_edges(body->bonePositions(), body->boneEdges(), sea_green);
  viewer.data().compute_normals();
  viewer.data().show_lines = false;
  viewer.data().show_overlay_depth = false;
  viewer.data().line_width = 1;
  viewer.core().trackball_angle.normalize();
  viewer.callback_pre_draw = &pre_draw;
  viewer.callback_key_down = &key_down;
  viewer.core().is_animating = false;
  viewer.core().camera_zoom = 2.5;
  viewer.core().animation_max_fps = 60.;
  viewer.core().background_color.setOnes();
  viewer.launch();
}