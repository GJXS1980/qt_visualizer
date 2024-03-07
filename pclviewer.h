#pragma once

#include <iostream>

// Qt
#include <QMainWindow>

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
#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QPixmap>
#include <QImageReader>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public Q_SLOTS:
  void randomButtonPressed ();

  void DLImagViewer();
  void resultImagViewer();
  void pointCloundViewer();
  void workSpaceViewer();
  void exitViewer();
protected:
  void
  refreshView();

  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::PCLVisualizer::Ptr viewerPointClound;
  PointCloudT::Ptr cloud;

private:
  Ui::PCLViewer *ui;
};
