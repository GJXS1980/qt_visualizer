#include "pclviewer.h"
#include "ui_pclviewer.h"

#include <opencv2/opencv.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

mmind::eye::Camera mecheyecamera;
//bool findCamera = true;

PCLViewer::PCLViewer (QWidget *parent) : QMainWindow (parent), ui (new Ui::PCLViewer)
{
  ui->setupUi (this);

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->resize (200);

  // 更新时间线程
  timeUpdater = new TimeUpdater(this);
  connect(timeUpdater, &TimeUpdater::updateTimeSignal, this, &PCLViewer::receiveUpdateTime);
  timeUpdater->start();


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
  connect (ui->runButton,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // 点击退出按钮
  connect (ui->exitButton,  SIGNAL (clicked ()), this, SLOT (exitViewer ()));
  // 点击连接相机按钮
  connect (ui->connectCameraButton,  SIGNAL (clicked ()), this, SLOT (connectCameraButton ()));
}


/**
 * @brief 连接相机
 *
 * @param None
 * @return None
 */
void PCLViewer::connectCameraButton ()
{

    // 搜索相机列表
    std::cout << "Discovering all available cameras..." << std::endl;
    std::vector<mmind::eye::CameraInfo> cameraInfoList = mmind::eye::Camera::discoverCameras();


    if (cameraInfoList.empty())
    {
        // 相机连接异常，弹窗显示消息
        QMessageBox::critical(nullptr, "连接异常", "请检测相机连接是否正常.", QMessageBox::Ok);
        qApp->quit();
     }
     else
     {
        // 弹窗输入相机IP
        bool ok;
        QString cameraIP = QInputDialog::getText(nullptr, "输入相机IP", "请输入相机IP地址:", QLineEdit::Normal, "", &ok);

        if (!ok || cameraIP.isEmpty())
        {
            // 用户取消输入，执行退出操作
            qApp->quit();
        }
        else
        {
            // 通过IP连接相机
            mmind::eye::ErrorStatus status = mecheyecamera.connect(cameraIP.toStdString());

            if (status.isOK())
            {
                // 连接成功，退出循环
                std::cout << "连接成功" << std::endl;
                ui->runStatus->setText("相机连接成功");
                ui->cameraStatus->setText("相机连接成功");
            }
            else
            {
                // 连接错误，弹窗显示消息
                QMessageBox::critical(nullptr, "错误", "输入错误，请重新输入！", QMessageBox::Ok);
                ui->runStatus->setText("相机连接失败");
            }
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
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    showError(mecheyecamera.capture2DAnd3D(frame2DAnd3D));

    //  保存彩色图像
    const std::string imageColorFile = "2DColorImage.png";
    // Save the obtained data with the set filenames.
    mmind::eye::Color2DImage colorImage = frame2DAnd3D.frame2D().getColorImage();
    cv::Mat color8UC3 = cv::Mat(colorImage.height(), colorImage.width(), CV_8UC3, colorImage.data());
    cv::imwrite(imageColorFile, color8UC3);

    const std::string depthImgFile = "DepthMap.tiff";
    mmind::eye::DepthMap depthMap = frame2DAnd3D.frame3D().getDepthMap();
    cv::Mat depth32F = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv::imwrite(depthImgFile, depth32F);

   const std::string UntexturedPointCloudFile = "UntexturedPointCloud.ply";
    showError(frame2DAnd3D.frame3D().saveUntexturedPointCloud(mmind::eye::FileFormat::PLY, UntexturedPointCloudFile));

    const std::string texturedPointCloudFile = "TexturedPointCloud.ply";
    showError(frame2DAnd3D.saveTexturedPointCloud(mmind::eye::FileFormat::PLY, texturedPointCloudFile));
    //std::cout << "Capture and save the textured point cloud: " << texturedPointCloudFile << std::endl;

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
    pcl::io::loadPLYFile<pcl::PointXYZRGB>("TexturedPointCloud.ply", *cloud);

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


/**
 * @brief 界面退出
 *
 * @param None
 * @return None
 */
void PCLViewer::exitViewer ()
{
    // 断开相机连接
    mecheyecamera.disconnect();
    // 执行退出操作
    qApp->quit();
}



/**
 * @brief 更新时间的槽函数
 *
 * @param None
 * @return None
 */
void PCLViewer::updateTime()
{
    QDateTime currentDateTime = QDateTime::currentDateTime();
    // 显示时间
    QString formattedTime = currentDateTime.toString("yyyy年MM月dd hh:mm:ss");
    ui->timeLabel->setText(formattedTime);
}


/**
 * @brief 更新时间的槽函数
 *
 * @param None
 * @return None
 */
void PCLViewer::receiveUpdateTime(const QString &currentTime)
{
    // 更新时间显示
    ui->timeLabel->setText(currentTime);
}



