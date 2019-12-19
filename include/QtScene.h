#ifndef QT_SCENCE_H_
#define QT_SCENCE_H_

#include <QGLViewer/qglviewer.h>
#include <memory>

#include "Sim3Optimizer.h"

class QtScene : public QGLViewer {
  Q_OBJECT
 public:
  explicit QtScene(QWidget* parent = 0, const QGLWidget* shareWidget = 0,
                   Qt::WindowFlags flags = 0);

 public:
  void setSphere(std::shared_ptr<Sim3Optimizer>& optimizer) {
    optimizer_ = optimizer;
  }

 public:
  bool isInit;

 signals:

 public slots:

 protected:
  virtual void draw();
  virtual void init();

 private:
  std::shared_ptr<Sim3Optimizer> optimizer_;
};

#endif
