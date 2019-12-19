#ifndef SIM3_OPTIMIZE_H_
#define SIM3_OPTIMIZE_H_

#include <iostream>
#include <vector>
#include <sophus/sim3.hpp>

#include "DataStruct.h"

class Sim3Optimizer {
public:
  Sim3Optimizer();
  // void pushVertex(Sim3Vertex &vertex) {
  //     vertexes.push_back(vertex);
  // }

  void pushEdge(Sim3Edge &edge) { edges.push_back(edge); }

  const std::vector<Sim3Edge> &getEdges() { return edges; }

  Sim3Vertex &getVertexes() { return vertexes; }

  void setVertexes(Sim3Vertex &_vertexes) { vertexes = _vertexes; }

  virtual bool optimize(int iter = 50);

  virtual bool LocalBAOptimize(int iter = 50);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void ErrorAndJacobianCalculation(
      const Sophus::Sim3d& Sji, const Eigen::Matrix<double, 7, 7>& information,
      const Eigen::Matrix<double, 7, 1>& lie_i,
      const Eigen::Matrix<double, 7, 1>& lie_j,
      Eigen::Matrix<double, 7, 1>& residuals,
      Eigen::Matrix<double, 7, 7>& Jacobian_i,
      Eigen::Matrix<double, 7, 7>& Jacobian_j);

  double IterateOnce(Eigen::Matrix<double, Eigen::Dynamic, 1>& delta_sim);

  Sim3Vertex vertexes;
  std::vector<Sim3Edge> edges;

  std::map<int, int> vertexes_remapped;
  std::map<int, int> inversed_vertexes_remapped;
};

#endif // SPHERE_H
