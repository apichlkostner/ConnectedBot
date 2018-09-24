#include "mainwindow.h"
#include <QApplication>
#include "ui_mainwindow.h"
#include <QDebug>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    showRobotCamera("/make_photo", "/raspi_camera/image_raw/compressed")
{
    // todo: showRobotCamera parameter should be configurable as in previous version
    ui->setupUi(this);
    ui->graphicsView->setScene(&scene);

    connect(&showRobotCamera, 
            SIGNAL(newImageReady(QPixmap)),
            this, 
            SLOT(newImageReady(QPixmap)));
    showRobotCamera.start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::newImageReady(QPixmap pixmap)
{
    scene.clear();
    scene.addPixmap(pixmap);
    ui->graphicsView->fitInView(scene.itemsBoundingRect() ,Qt::KeepAspectRatio);
}


/************************************************
 * main function
 ************************************************/
int main(int argc, char *argv[])
{
    ShowRobotCamera::init(argc, argv);
    QApplication a(argc, argv);
    
    MainWindow w;
    w.show();

    return a.exec();
}