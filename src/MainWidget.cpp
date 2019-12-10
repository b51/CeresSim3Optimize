#include <fstream>
#include <iostream>
#include <string.h>

#include <QFileDialog>
#include <QGridLayout>
#include <QRegExpValidator>

#include "MainWidget.h"
#include "Sphere.h"
#include "sphereScene.h"

void getVertex(char *ptr, Sim3Vertex &vertex) {
  ptr += 17;
  int index;
  double x;
  double y;
  double z;
  double phi1;
  double phi2;
  double phi3;
  double sigma;

  sscanf(ptr, "%d %lf %lf %lf %lf %lf %lf %lf",
         // &index, &x, &y, &z, &phi1, &phi2, &phi3, &sigma);
         &index, &phi1, &phi2, &phi3, &x, &y, &z, &sigma);

  Eigen::Matrix<double, 7, 1> lie;
  lie << x, y, z, phi1, phi2, phi3, sigma;

  vertex[index] = lie;
}

void getEdge(char *ptr, Sim3Edge &edge) {
  int index1;
  int index2;
  double x;
  double y;
  double z;
  double phi1;
  double phi2;
  double phi3;
  double sigma;

  sscanf(ptr + 15, "%lf %lf %lf %lf %lf %lf %lf %d %d ",
         // &x, &y, &z, &phi1, &phi2, &phi3, &sigma, &index1, &index2);
         &phi1, &phi2, &phi3, &x, &y, &z, &sigma, &index1, &index2);

  Eigen::Matrix<double, 7, 1> lie;
  lie << x, y, z, phi1, phi2, phi3, sigma;

  edge.j = index1;
  edge.i = index2;
  edge.pose = Sophus::Sim3d::exp(lie);

  edge.information = Eigen::Matrix<double, 7, 7>::Identity();
}

MainWidget::MainWidget(QWidget *parent) : QWidget(parent) {
  QGridLayout *mainLayout = new QGridLayout;

  sphereScene = new SphereScene;
  IterSetEdit = new QLineEdit;
  InitButton = new QPushButton;
  optButton = new QPushButton;

  InitButton->setText("initial");
  IterSetEdit->setText("100");

  QRegExp regExp2("[0-9]{1,4}");
  QRegExpValidator *pRegExpValidator2 = new QRegExpValidator(regExp2, this);
  IterSetEdit->setValidator(pRegExpValidator2);

  optButton->setText("optimize");
  optButton->setDisabled(false);

  mainLayout->addWidget(sphereScene, 0, 0, 3, 3);
  mainLayout->addWidget(IterSetEdit, 3, 0, 1, 1);
  mainLayout->addWidget(InitButton, 3, 1, 1, 1);
  mainLayout->addWidget(optButton, 3, 2, 1, 1);

  this->setLayout(mainLayout);

  connect(optButton, SIGNAL(clicked()), this, SLOT(btnOptimize()));
  connect(InitButton, SIGNAL(clicked()), this, SLOT(btnInit()));

  isInit = false;
  sphere.reset(new Sphere);
  sphereScene->setSphere(sphere);
}

void MainWidget::btnInit() {
  QString dataFile = QFileDialog::getOpenFileName(this, "data file");

  std::fstream f(dataFile.toLatin1().data());

  char *buffer = (char *)malloc(512);

  memset(buffer, 0, 512);
  f.getline(buffer, 512);

  const char *vertexSign = "VERTEX_SIM3:QUAT";
  const char *edgeSign = "EDGE_SIM3:QUAT";

  Sim3Vertex vertexes;
  while (strlen(buffer) != 0) {
    char *ptr = nullptr;

    if ((ptr = strstr(buffer, vertexSign)) != nullptr) {
      getVertex(ptr, vertexes);
    }

    if ((ptr = strstr(buffer, edgeSign)) != NULL) {
      Sim3Edge edge;
      getEdge(ptr, edge);
      sphere->pushEdge(edge);
    }

    memset(buffer, 0, 512);
    f.getline(buffer, 512);
  }
  sphere->setVertexes(vertexes);

  free(buffer);

  isInit = true;
  sphereScene->isInit = true;
  optButton->setEnabled(true);
  update();
}

void MainWidget::btnOptimize() {
  isInit = false;
  int iter = 100;
  QString Striter = IterSetEdit->text();
  if (Striter.size())
    iter = atoi(Striter.toLatin1().data());
  sphere->optimize(iter);
  update();
  optButton->setDisabled(true);
}
