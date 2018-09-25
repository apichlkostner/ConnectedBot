#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), showRobotCamera() {
  ui->setupUi(this);
  ui->graphicsView->setScene(&scene);
  ui->statusBar->showMessage(QString::fromStdString(showRobotCamera.image_topic_name()));

  // connect event loop with signal from ROS node ROS node has NO Qt event loop
  // but a ROS event loop
  connect(&showRobotCamera, &ShowRobotCamera::newImageReady, this,
          &MainWindow::showImage);

  // ROS node will run as separate thread without Qt event loop but it emits
  // singals
  showRobotCamera.start();
}

MainWindow::~MainWindow() {
  delete ui;
  showRobotCamera.quit();
  showRobotCamera.wait();
}

void MainWindow::showImage(QPixmap pixmap) {
  scene.clear();
  scene.addPixmap(pixmap);
  ui->graphicsView->fitInView(scene.itemsBoundingRect(), Qt::KeepAspectRatio);
}

/************************************************
 * main function
 ************************************************/
int main(int argc, char *argv[]) {
  ShowRobotCamera::init(argc, argv);
  QApplication a(argc, argv);

  MainWindow w;
  w.show();

  return a.exec();
}