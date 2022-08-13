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

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include "pbcd/DeformableModel.hpp"

const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);
pbcd::DeformableModel *body;
Eigen::MatrixXd V, W, C, U, M;
Eigen::MatrixXi F, BE;
Eigen::VectorXi P;
pbcd::AnimationClip clip;
double anim_t = 0.0;
double anim_t_dir = 0.015;
bool recompute = true;

bool pre_draw(igl::opengl::glfw::Viewer &viewer)
{
  using namespace Eigen;
  using namespace std;
  if (recompute)
  {

    body->playAnimationClip(clip, anim_t);

    viewer.data().set_vertices(body->positions());
    viewer.data().set_edges(body->bonePositions(), body->boneEdges(), sea_green);
    viewer.data().compute_normals();
    if (viewer.core().is_animating)
    {
      anim_t += anim_t_dir;
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

  igl::readDMAT("../models/arm-weights.dmat", W);
  igl::lbs_matrix(V, W, M);

  body = new pbcd::DeformableModel(V, F, C, BE, W);

  std::vector<RotationList> poses;
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
  cout << "Press [space] to toggle animation" << endl;
  viewer.launch();
}