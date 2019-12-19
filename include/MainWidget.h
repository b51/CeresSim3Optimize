#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <memory>
#include <vector>

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include "QtScene.h"
#include "Sim3Optimizer.h"

class MainWidget : public QWidget {
  Q_OBJECT
 public:
  explicit MainWidget(QWidget* parent = 0);

 signals:

 public slots:
  void btnInit();
  void btnOptimize();

 private:
  QtScene* qt_scene_;
  QLineEdit* IterSetEdit;
  QPushButton* InitButton;
  QPushButton* optButton;

 private:
  bool isInit;

 private:
  std::shared_ptr<Sim3Optimizer> optimizer_;
};

#endif  // MAINWIDGET_H
