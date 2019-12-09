#include <string.h>

#include <iostream>
#include <fstream>

#include <QGridLayout>
#include <QFileDialog>
#include <QRegExpValidator>

#include "MainWidget.h"
#include "sphereScene.h"
#include "Sphere.h"

void getVertex(char *ptr, Vertex &vertex) {
    ptr += 16;
    int index;
    double x;
    double y;
    double z;
    double q1;
    double q2;
    double q3;
    double q4;

    sscanf(ptr, "%d %lf %lf %lf %lf %lf %lf %lf",
           &index, &x, &y, &z, &q2, &q3, &q4, &q1);

    Sophus::SE3d se3(Eigen::Quaternion<double>(q1, q2, q3, q4), Eigen::Vector3d(x, y, z));

    vertex.index = index;
    vertex.pose = se3.log();
}

void getEdge(char *ptr, Edge &edge) {
    int index1;
    int index2;
    double x;
    double y;
    double z;
    double q1;
    double q2;
    double q3;
    double q4;

    double h[21];

    sscanf(ptr + 14, "%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
           &index1, &index2, &x, &y, &z, &q2, &q3, &q4, &q1,
           h, h+1, h+2, h+3, h+4, h+5, h+6,
           h+7, h+8, h+9, h+10, h+11, h+12, h+13,
           h+14, h+15, h+16, h+17, h+18, h+19, h+20);

    Sophus::SE3d se3(Eigen::Quaternion<double>(q1, q2, q3, q4), Eigen::Vector3d(x, y, z));

    edge.i = index1;
    edge.j = index2;
    edge.pose = se3;

    int k = 0;

    for(int j = 0; j < 6; ++j) {
        for(int i = j; i < 6; ++i) {
            edge.infomation(i, j) = edge.infomation(j, i) = h[i + k];
        }

        k += 5 - j;
    }
}

MainWidget::MainWidget(QWidget *parent) :
    QWidget(parent)
{
    QGridLayout *mainLayout = new QGridLayout;

    sphereScene = new SphereScene;
    IterSetEdit = new QLineEdit;
    InitButton  = new QPushButton;
    optButton   = new QPushButton;

    InitButton->setText("initial");
    IterSetEdit->setText("100");

    QRegExp regExp2("[0-9]{1,4}");
    QRegExpValidator *pRegExpValidator2 = new QRegExpValidator(regExp2,this);
    IterSetEdit->setValidator(pRegExpValidator2);

    optButton->setText("optimize");
    optButton->setDisabled(false);

    mainLayout->addWidget(sphereScene, 0, 0, 3, 3);
    mainLayout->addWidget(IterSetEdit, 3, 0, 1, 1);
    mainLayout->addWidget(InitButton,3, 1, 1, 1);
    mainLayout->addWidget(optButton, 3, 2, 1, 1);

    this->setLayout(mainLayout);


    connect(optButton, SIGNAL(clicked()), this, SLOT(btnOptimize()));
    connect(InitButton, SIGNAL(clicked()), this, SLOT(btnInit()));


    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////

    isInit = false;
    sphere.reset(new Sphere);
    sphereScene->setSphere(sphere);
}

void MainWidget::btnInit()
{
    // QString dataFile = QFileDialog::getOpenFileName(this, "data file");

    // std::fstream f(dataFile.toLatin1().data());
    std::fstream f("sphere_data.g2o");

    char *buffer = (char*)malloc(512);

    memset(buffer, 0, 512);
    f.getline(buffer, 512);

    const char *vertexSign = "VERTEX_SE3:QUAT";
    const char *edgeSign = "EDGE_SE3:QUAT";

    while(strlen(buffer) != 0) {
        char *ptr = nullptr;

        if((ptr = strstr(buffer, vertexSign)) != nullptr) {
            Vertex vertex;
            getVertex(ptr, vertex);
            sphere->pushVertex(vertex);
        }

        if((ptr = strstr(buffer, edgeSign)) != NULL) {
            Edge edge;
            getEdge(ptr, edge);
            sphere->pushEdge(edge);
        }

        memset(buffer, 0, 512);
        f.getline(buffer, 512);
    }

    free(buffer);

    isInit = true;
    sphereScene->isInit = true;
    optButton->setEnabled(true);
    update();
}

void MainWidget::btnOptimize()
{
    isInit = false;
    int iter = 100;
    QString Striter = IterSetEdit->text();
    if(Striter.size())
        iter = atoi(Striter.toLatin1().data());
    sphere->optimize(iter);
    update();
    optButton->setDisabled(true);
}



