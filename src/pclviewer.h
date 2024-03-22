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
#include <QDebug>
#include <QCoreApplication>
#include <QTcpSocket>

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

//// 更新CPU使用率
//class CPUMonitor : public QObject
//{
//    Q_OBJECT

//public:
//    CPUMonitor(QObject *parent = nullptr) : QObject(parent)
//    {
//        // 创建定时器，每秒更新一次CPU使用率
//        timer = new QTimer(this);
//        connect(timer, &QTimer::timeout, this, &CPUMonitor::updateCPUUsage);
//        timer->start(1000); // 每秒更新一次

//        // 获取初始CPU使用率
//        updateCPUUsage();
//    }

//public slots:
//    void updateCPUUsage()
//    {
//        // 获取当前应用程序的CPU使用率
//        double cpuUsage = QSystemInfo::currentCpuUsage();
//        qDebug() << "Current CPU Usage: " << cpuUsage << "%";
//    }

//private:
//    QTimer *timer;
//};


// 更新软件时间
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
//  class CPUMonitor;
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
  void connectRobotButton();
//  void closeSocket(int socket);
  void sendCommand(QTcpSocket& socket, const QString& command);
//  void receiveResponse(int socket);
  void TCP_connection(int port);

//private slots:
    void on_TiaoZhuan_clicked();
    void returnToMain(); // 添加返回主界面的槽函数

signals:
    void sendData(QString data);

protected:
  void refreshView();

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::PCLVisualizer::Ptr viewerPointClound;
  PointCloudT::Ptr cloud;

private:
  Ui::PCLViewer *ui;
  TimeUpdater *timeUpdater;
  // 创建CPUMonitor对象
//  CPUMonitor *cpuMonitor;
};




