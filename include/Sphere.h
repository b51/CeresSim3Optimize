#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <vector>

#include "DataStruct.h"

class Sphere {
public:
  Sphere();
  // void pushVertex(Sim3Vertex &vertex) {
  //     vertexes.push_back(vertex);
  // }

  void pushEdge(Sim3Edge &edge) { edges.push_back(edge); }

  const std::vector<Sim3Edge> &getEdges() { return edges; }

  Sim3Vertex &getVertexes() { return vertexes; }

  void setVertexes(Sim3Vertex &_vertexes) { vertexes = _vertexes; }

  virtual bool optimize(int iter = 50);

private:
  Sim3Vertex vertexes;
  std::vector<Sim3Edge> edges;
};

#endif // SPHERE_H
