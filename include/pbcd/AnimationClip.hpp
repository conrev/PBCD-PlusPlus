#ifndef ANIMATIONCLIP_H
#define ANIMATIONCLIP_H

#include <Eigen/Geometry>
#include <vector>

typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> RotationList;

namespace pbcd
{
    struct AnimationClip
    {
        std::vector<RotationList> posesList;
    };
}

#endif