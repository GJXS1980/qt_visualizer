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


  // Set up the QVTK window  
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
  ui->qvtkWidget->setRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());

  auto renderer1 = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow1 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow1->AddRenderer(renderer1);

  viewerPointClound.reset(new pcl::visualization::PCLVisualizer(renderer1, renderWindow1, "PointCloundviewer", false));
  ui->qvtkPointWidget->setRenderWindow(viewerPointClound->getRenderWindow());
  viewerPointClound->setupInteractor(ui->qvtkPointWidget->interactor(), ui->qvtkPointWidget->renderWindow());


  // 使用 VTK 中的 PNG 读取器 qvtkImagWidget
  auto Imgrenderer = vtkSmartPointer<vtkRenderer>::New();
  auto ImgrenderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  ImgrenderWindow->AddRenderer(Imgrenderer);

  viewerColorImg.reset(new pcl::visualization::PCLVisualizer(Imgrenderer, ImgrenderWindow, "Imgviewer", false));
  ui->qvtkImagWidget->setRenderWindow(viewerColorImg->getRenderWindow());
  viewerColorImg->setupInteractor(ui->qvtkImagWidget->interactor(), ui->qvtkImagWidget->renderWindow());


#else
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

  viewerPointClound.reset(new pcl::visualization::PCLVisualizer("PointCloundviewer", false));
  ui->qvtkPointWidget->SetRenderWindow(viewerPointClound->getRenderWindow());
  viewerPointClound->setupInteractor(ui->qvtkPointWidget->GetInteractor(), ui->qvtkPointWidget->GetRenderWindow());

#endif

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

//  // Connect R,G,B sliders and their functions
//  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
//  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
//  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
//  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
//  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
//  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
//  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  viewerPointClound->addPointCloud (cloud, "cloud");
//  pSliderValueChanged (2);
   viewerPointClound->resetCamera ();
  viewer->resetCamera ();
  
  refreshView();

}


/**
 * @brief 连接相机并获取彩色图、深度图像和点云，并作可视化显示
 *
 * @param None
 * @return None
 */
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
    mmind::eye::ErrorStatus status = mecheyecamera.connect("192.168.23.15");


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

    // Load the point cloud from file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("TexturedPointCloud.ply", *cloud);

    // Load the point cloud from file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("TexturedPointCloudWithNormals.ply", *cloud1);

    // Update the PCL viewer with the new point cloud
    viewer->removePointCloud("cloud");
    viewerPointClound->removePointCloud("cloud");

    viewer->addPointCloud(cloud, "cloud");
    viewerPointClound->addPointCloud(cloud1, "cloud");

    viewer->resetCamera();
    viewerPointClound->resetCamera();


//    // 读取图片
    QImageReader Imgreader("2DColorImage.png");
    // 转换 QImage 到 QPixmap
    QImage image = Imgreader.read();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    convertImageToPointCloud(image, cloud2);
    viewerColorImg->addPointCloud(cloud, "image_cloud");
    viewerColorImg->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "image_cloud");
    viewerColorImg->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "image_cloud");


    refreshView();




  }
} catch (const std::exception& e) 
{
    std::cerr << "Exception caught: " << e.what() << std::endl;
    // 处理异常的代码
}
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


/**
 * @brief 更新点云可视化界面
 *
 * @param None
 * @return None
 */
void PCLViewer::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui->qvtkWidget->renderWindow()->Render();
  ui->qvtkPointWidget->renderWindow()->Render();
  ui->qvtkImagWidget->renderWindow()->Render();
#else
  ui->qvtkWidget->update();
  ui->qvtkPointWidget->update();
  ui->qvtkImagWidget->update();


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


void PCLViewer::ImagViewer ()
{



//void displayImage(const QString &imagePath, vtkSmartPointer<vtkRenderer> renderer, pcl::visualization::PCLVisualizer::Ptr pclViewer, QVTKWidget *qvtkWidget)
//{
//    // 读取图片
//    QImageReader reader(imagePath);
//    QImage image = reader.read();

//    // 检查是否成功读取图片
//    if (image.isNull()) {
//        qDebug() << "Failed to load image from" << imagePath;
//        return;
//    }

//    // 转换 QImage 到 QPixmap
//    QPixmap pixmap = QPixmap::fromImage(image);

//    // 在 QLabel 中显示 QPixmap
//    QLabel label;
//    label.setPixmap(pixmap);

//    // 设置窗口标题
//    label.setWindowTitle("Image Viewer");

//    // 如果有提供的 VTK 渲染器，将其用于渲染图片
//    if (renderer) {
//        vtkSmartPointer<vtkImageActor> imageActor = vtkSmartPointer<vtkImageActor>::New();
//        imageActor->SetInputData(imageToVtkImageData(image));
//        renderer->AddActor(imageActor);
//        renderer->ResetCamera();
//        qvtkWidget->GetRenderWindow()->Render();
//    }

//    // 如果有提供的 PCLVisualizer，将图片作为背景显示
//    if (pclViewer) {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//        convertImageToPointCloud(image, cloud);
//        pclViewer->addPointCloud(cloud, "image_cloud");
//        pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "image_cloud");
//        pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "image_cloud");
//    }

//    // 显示 Qt 窗口
//    label.show();
//}

}


void PCLViewer::convertImageToPointCloud(const QImage &image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    // 根据你的需要实现将 QImage 转换为 PCL 点云的逻辑
    // 这里只是一个简单的示例，仅用于演示

    // 创建一个点云
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 假设图片大小为 width x height
    int width = image.width();
    int height = image.height();

    // 遍历图片像素，并将每个像素转换为点云中的一个点
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            QRgb pixelColor = image.pixel(x, y);

            pcl::PointXYZRGB point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = 0.0;  // 你可能需要更复杂的深度计算逻辑

            // 从 QRgb 中提取颜色值
            point.r = qRed(pixelColor);
            point.g = qGreen(pixelColor);
            point.b = qBlue(pixelColor);

            // 添加点到点云
            cloud->push_back(point);
        }
    }
}
