#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <memory>
#include <vector>

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

class Sphere;

class SphereScene;

class MainWidget : public QWidget {
  Q_OBJECT
public:
  explicit MainWidget(QWidget *parent = 0);

signals:

public slots:
  void btnInit();
  void btnOptimize();

private:
  SphereScene *sphereScene;
  QLineEdit *IterSetEdit;
  QPushButton *InitButton;
  QPushButton *optButton;

private:
  bool isInit;

private:
  std::shared_ptr<Sphere> sphere;
};

#endif // MAINWIDGET_H
