#pragma once

#include <iostream>

// Qt
#include <QMainWindow>
#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QPixmap>
#include <QImageReader>
#include <QDateTime>
#include <QTimer>
#include <QThread>
#include <QInputDialog>
#include <QString>
#include <QMessageBox>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

#include <vtkSmartPointer.h>
#include <vtkPNGReader.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 定义获取系统时间并更新到UI的线程
class TimeUpdater : public QThread
{
    Q_OBJECT

public:
    explicit TimeUpdater(QObject* parent = nullptr) : QThread(parent) {}

signals:
    void updateTimeSignal(const QString &currentTime);

protected:
    // 在 QThread 的 run() 函数中执行线程任务
    void run() override
    {
        while (!isInterruptionRequested())
        {
            // 获取系统时间
            QDateTime currentDateTime = QDateTime::currentDateTime();
            QString formattedDateTime = currentDateTime.toString("yyyy年MM月dd hh:mm:ss");

            // 发送信号通知主线程更新时间
            emit updateTimeSignal(formattedDateTime);

            // 等待一秒
            sleep(1);
        }
    }
};


// 定义拍照线程
class CameraConnector : public QThread
{
    Q_OBJECT

signals:
    void cameraConnectedSignal(bool connected);

protected:
    void run() override
    {
        // Your camera connection code goes here
        // Emit the signal with the result
        emit cameraConnectedSignal(true);  // Modify as per your camera connection logic
    }
};


//// 定义一个 WorkerThread 类，用于在单独的线程中获取系统时间
//class CameraConnector : public QThread
//{
//    Q_OBJECT

//signals:
//    void cameraConnectedSignal(bool connected);

//protected:
//    void run() override
//    {
//        // Your camera connection code goes here
//        // Emit the signal with the result
//        emit cameraConnectedSignal(true);  // Modify as per your camera connection logic
//    }
//};


namespace Ui
{
  class PCLViewer;
  class TimeUpdater;
}



class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = nullptr);
  ~PCLViewer ();

public Q_SLOTS:
  void runButtonPressed ();

  void DLImagViewer();
  void resultImagViewer();
  void pointCloundViewer();
  void workSpaceViewer();
  void exitViewer();
  void updateTime();
  void receiveUpdateTime(const QString &currentTime);
  void connectCameraButton();

protected:
  void refreshView();

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::PCLVisualizer::Ptr viewerPointClound;
  PointCloudT::Ptr cloud;

private:
  Ui::PCLViewer *ui;
  TimeUpdater *timeUpdater;
};




