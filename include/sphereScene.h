#ifndef SphereScene_H
#define SphereScene_H

#include <memory>

#include <QGLViewer/qglviewer.h>

class Sphere;

class SphereScene : public QGLViewer {
  Q_OBJECT
public:
  explicit SphereScene(QWidget *parent = 0, const QGLWidget *shareWidget = 0,
                       Qt::WindowFlags flags = 0);

public:
  void setSphere(std::shared_ptr<Sphere> &sp) { sphere = sp; }

public:
  bool isInit;

signals:

public slots:

protected:
  virtual void draw();
  virtual void init();

private:
  std::shared_ptr<Sphere> sphere;
};

#endif // SphereScene_H
