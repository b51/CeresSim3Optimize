#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

struct Vertex {
  int index;
  Eigen::Matrix<double, 6, 1> pose;
};

struct Edge {
  int i;
  int j;
  Sophus::SE3d pose;
  Eigen::Matrix<double, 6, 6> information;
};

/*
struct Sim3Vertex
{
    int                         index;
    Eigen::Matrix<double, 7, 1> pose;
};
*/
typedef std::map<
    int, Eigen::Matrix<double, 7, 1>, std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix<double, 7, 1>>>>
    Sim3Vertex;

struct Sim3Edge {
  int i;
  int j;
  Sophus::Sim3d pose;
  Eigen::Matrix<double, 7, 7> information;
};

#endif // DATASTRUCT_H
