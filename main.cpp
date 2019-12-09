#include <QApplication>
#include <glog/logging.h>

#include "MainWidget.h"
#include <QGLWidget>


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    google::InitGoogleLogging(argv[0]);

    MainWidget w;
    w.show();

    app.exec();
    w.btnInit();
    w.btnOptimize();
    return 0;
}

