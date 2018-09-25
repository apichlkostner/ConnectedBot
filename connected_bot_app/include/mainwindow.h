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
    void showImage(QPixmap pixmap); //! shows the image on the main scene

private:
    Ui::MainWindow *ui;     //! main window ui
    QGraphicsScene scene;   //! main scene to show processed camera image
    ShowRobotCamera showRobotCamera;    //! ROS node which emits processed camera images
};

#endif // MAINWINDOW_H
