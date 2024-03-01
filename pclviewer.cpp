#include "pclviewer.h"
#include "ui_pclviewer.h"

#include <opencv2/opencv.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

mmind::eye::Camera mecheyecamera;

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("隆深智能装箱系统");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->resize (200);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

  // Fill the cloud with some points
  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);

    point.r = red;
    point.g = green;
    point.b = blue;
  }

  // Set up the QVTK window  
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
  ui->qvtkWidget->setRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
#else
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
#endif

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  
  refreshView();
}

void PCLViewer::randomButtonPressed ()
{
  // 搜索相机列表
  std::cout << "Discovering all available cameras..." << std::endl;
  std::vector<mmind::eye::CameraInfo> cameraInfoList = mmind::eye::Camera::discoverCameras();
  
try 
{
  //  搜索相机
  if (cameraInfoList.empty()) 
  {
    throw std::runtime_error("No cameras found.");
  }
  else
  {
    // 通过ip连接相机
    mmind::eye::ErrorStatus status = mecheyecamera.connect("192.168.23.10");


    // 获取彩色图像
    mmind::eye::Frame2D frame2D;
    cv::Mat image2D;

    mecheyecamera.capture2D(frame2D);

    // 保存图像
    const std::string imageColorFile = "2DColorImage.png";
    switch (frame2D.colorType()) 
    {
        case mmind::eye::ColorTypeOf2DCamera::Monochrome:
        {
            mmind::eye::GrayScale2DImage grayImage = frame2D.getGrayScaleImage();
            image2D = cv::Mat(grayImage.height(), grayImage.width(), CV_8UC1, grayImage.data());
        }
        case mmind::eye::ColorTypeOf2DCamera::Color:
        {
            mmind::eye::Color2DImage colorImage = frame2D.getColorImage();
            image2D = cv::Mat(colorImage.height(), colorImage.width(), CV_8UC3, colorImage.data());
        }
    }
    cv::imwrite(imageColorFile, image2D);

    // 获取深度图像和点云
    mmind::eye::Frame3D frame3D;
    mecheyecamera.capture3D(frame3D);
    mmind::eye::DepthMap depthMap = frame3D.getDepthMap();
    // 保存深度图
    cv::Mat depth32F = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    const std::string depthImgFile = "DepthMap.tiff";
    cv::imwrite(depthImgFile, depth32F);

    // 保存黑白点云
    const std::string pointCloudFile = "PointCloud.ply";
    frame3D.saveUntexturedPointCloud(mmind::eye::FileFormat::PLY, pointCloudFile);


    // 定义获取2D和3D数据
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    // 采集并获取用于生成2D图、深度图和含法向量的纹理点云的数据
    mecheyecamera.capture2DAnd3DWithNormal(frame2DAnd3D);

    // 保存彩色点云
    const std::string texturedPointCloudFile = "TexturedPointCloud.ply";
    frame2DAnd3D.saveTexturedPointCloud(mmind::eye::FileFormat::PLY, texturedPointCloudFile);

    // 获取带法向量的纹理点云
    mmind::eye::TexturedPointCloudWithNormals texturedpointcloudwithnormals = frame2DAnd3D.getTexturedPointCloudWithNormals();
    // 保存带法向量纹理点云
    const std::string texturedPointCloudWithNormalsFile = "TexturedPointCloudWithNormals.ply";
    frame2DAnd3D.saveTexturedPointCloudWithNormals(mmind::eye::FileFormat::PLY, texturedPointCloudWithNormalsFile);

  }
} catch (const std::exception& e) 
{
    std::cerr << "Exception caught: " << e.what() << std::endl;
    // 处理异常的代码
}

  printf ("Random button was pressed\n");

  // Set the new color
  for (auto& point: *cloud)
  {
    point.r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    point.g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    point.b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
  }

  viewer->updatePointCloud (cloud, "cloud");
  refreshView();
}

void PCLViewer::RGBsliderReleased ()
{
  // Set the new color
  for (auto& point: *cloud)
  {
    point.r = red;
    point.g = green;
    point.b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  refreshView();
}

void PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  refreshView();
}

void PCLViewer::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui->qvtkWidget->renderWindow()->Render();
#else
  ui->qvtkWidget->update();
#endif
}

void PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
