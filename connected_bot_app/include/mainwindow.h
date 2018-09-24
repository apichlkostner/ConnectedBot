#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPixmap>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QFileInfo>
#include <QMessageBox>
#include <QResizeEvent>
#include <QGraphicsScene>
#include "show_robot_camera_node.h"


namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void newImageReady(QPixmap pixmap);

private:
    Ui::MainWindow *ui;
    QGraphicsScene scene;
    ShowRobotCamera showRobotCamera;
};

#endif // MAINWINDOW_H
