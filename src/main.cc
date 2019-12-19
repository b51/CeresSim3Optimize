#include <QApplication>
#include <QGLWidget>
#include <glog/logging.h>

#include "MainWidget.h"

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  MainWidget w;
  w.show();

  app.exec();
  return 0;
}
