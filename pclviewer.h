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

namespace Ui
{
  class PCLViewer;
  class WorkerThread;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = nullptr);
  ~PCLViewer ();

public Q_SLOTS:
  void randomButtonPressed ();

  void DLImagViewer();
  void resultImagViewer();
  void pointCloundViewer();
  void workSpaceViewer();
  void exitViewer();
  void updateTime();
  void receiveUpdateTime(const QString &currentTime);

protected:
  void refreshView();

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::PCLVisualizer::Ptr viewerPointClound;
  PointCloudT::Ptr cloud;

private:
  Ui::PCLViewer *ui;
};


// 定义一个 WorkerThread 类，用于在单独的线程中获取系统时间
class WorkerThread : public QThread
{
    Q_OBJECT

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
