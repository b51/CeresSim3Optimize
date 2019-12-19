#include "QtScene.h"
#include "Sim3Optimizer.h"

QtScene::QtScene(QWidget* parent, const QGLWidget* shareWidget,
                 Qt::WindowFlags flags)
    : QGLViewer(parent, shareWidget, flags) {
  isInit = false;
}

void QtScene::draw() {
  if (isInit == true) {
    setSceneRadius(350);
    Sim3Vertex& vertexes = optimizer_->getVertexes();
    const std::vector<Sim3Edge>& edges = optimizer_->getEdges();

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.5f, 1.0f);
    for (auto& it : vertexes) {
      Sophus::Sim3d pose = Sophus::Sim3d::exp(it.second).inverse();
      glVertex3d(pose.translation()[0], pose.translation()[1],
                 pose.translation()[2]);
    }
    glEnd();

    glBegin(GL_LINES);
    for (size_t i = 0; i < edges.size(); ++i) {
      const Sim3Edge& edge = edges[i];
      glColor3f(1, 0, 0);
      Sophus::Sim3d pose_i = Sophus::Sim3d::exp(vertexes[edge.i]).inverse();
      Sophus::Sim3d pose_j = Sophus::Sim3d::exp(vertexes[edge.j]).inverse();
      glVertex3d(pose_i.translation()[0], pose_i.translation()[1],
                 pose_i.translation()[2]);
      glColor3f(0, 1, 0);
      glVertex3d(pose_j.translation()[0], pose_j.translation()[1],
                 pose_j.translation()[2]);
    }
    glEnd();
  }
}

void QtScene::init() {
  restoreStateFromFile();
  glDisable(GL_LIGHTING);
  glPointSize(3.0);
  setGridIsDrawn();
}
