#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <Eigen/Dense>
#include <sophus/sim3.hpp>

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
