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


  // 搜索相机列表
  std::cout << "Discovering all available cameras..." << std::endl;
  std::vector<mmind::eye::CameraInfo> cameraInfoList = mmind::eye::Camera::discoverCameras();

  int findCamera = 1;

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

#else
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

  viewerPointClound.reset(new pcl::visualization::PCLVisualizer("PointCloundviewer", false));
  ui->qvtkPointWidget->SetRenderWindow(viewerPointClound->getRenderWindow());
  viewerPointClound->setupInteractor(ui->qvtkPointWidget->GetInteractor(), ui->qvtkPointWidget->GetRenderWindow());

#endif

  // 点击运行按钮开始拍照识别
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  //  搜索相机
  while (findCamera)
 {
      if (cameraInfoList.empty())
      {
        throw std::runtime_error("请检测相机连接是否正常.");
      }
      else
      {
        // 通过ip连接相机
        mmind::eye::ErrorStatus status = mecheyecamera.connect("192.168.23.15");
        findCamera = 0;
      }


  }


}


/**
 * @brief 连接相机并获取彩色图、深度图像和点云，并作可视化显示
 *
 * @param None
 * @return None
 */
void PCLViewer::randomButtonPressed ()
{

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

    // 点云结果可视化界面
    pointCloundViewer();
    // 工位可视化界面
    workSpaceViewer();

    // 实例分割图像可视化
    DLImagViewer();
    // 结果图像可视化
    resultImagViewer();

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

#else
  ui->qvtkWidget->update();
  ui->qvtkPointWidget->update();
//  ui->qvtkImagWidget->update();


#endif
}


PCLViewer::~PCLViewer ()
{
  delete ui;
}


/**
 * @brief 深度学习实例分割可视化界面
 *
 * @param None
 * @return None
 */
void PCLViewer::DLImagViewer ()
{
    // 读取图片
    QImageReader Imgreader("2DColorImage.png");
    QImage image = Imgreader.read();

    // 转换 QImage 到 QPixmap
    QPixmap pixmap = QPixmap::fromImage(image);

    // 获取 dlImg显示框 的大小
    int labelWidth = ui->dlImg->width();
    int labelHeight = ui->dlImg->height();

    // 调整 QPixmap 的大小以适应dlImg显示框
    QPixmap scaledPixmap = pixmap.scaled(labelWidth, labelHeight, Qt::KeepAspectRatio);

    // 在 QLabel 中显示调整大小后的 QPixmap
    ui->dlImg->setPixmap(scaledPixmap);

}

/**
 * @brief 码垛结果可视化界面
 *
 * @param None
 * @return None
 */
void PCLViewer::resultImagViewer ()
{

    // 读取图片
    QImageReader Imgreader("2DColorImage.png");
    QImage image = Imgreader.read();

    // 转换 QImage 到 QPixmap
    QPixmap pixmap = QPixmap::fromImage(image);

    // 获取 resultImg 的大小
    int labelWidth = ui->resultImg->width();
    int labelHeight = ui->resultImg->height();

    // 调整 QPixmap 的大小以适应 resultImg
    QPixmap scaledPixmap = pixmap.scaled(labelWidth, labelHeight, Qt::KeepAspectRatio);

    // 在 QLabel 中显示调整大小后的 QPixmap
    ui->resultImg->setPixmap(scaledPixmap);

}

/**
 * @brief 点云结果可视化界面
 *
 * @param None
 * @return None
 */
void PCLViewer::pointCloundViewer ()
{

    // 加载点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("TexturedPointCloudWithNormals.ply", *cloud);

    viewerPointClound->removePointCloud("cloud");
    viewerPointClound->addPointCloud(cloud, "cloud");
    viewerPointClound->resetCamera();

}


/**
 * @brief 工位可视化界面
 *
 * @param None
 * @return None
 */
void PCLViewer::workSpaceViewer ()
{

    // 加载点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("TexturedPointCloud.ply", *cloud);

    // Update the PCL viewer with the new point cloud
    viewer->removePointCloud("cloud");
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();
}

