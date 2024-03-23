#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "container.h"

#include <opencv2/opencv.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

#include <iostream>
#include <cstring>
#include <chrono>
#include <future>
#include <thread>
#include <atomic>

#include <boost/asio.hpp>

#include <QTcpSocket>
#include <QTcpServer>
#include <QHostAddress>

#define SERVER_IP "192.168.11.55"
#define MOTION_PORT 31400
#define STATUS_PORT 31401


bool status_flag = false;

mmind::eye::Camera mecheyecamera;
//bool findCamera = true;
bool camera_ok; // 相机连接状态
QString cameraIP; // 相机ip

PCLViewer::PCLViewer (QWidget *parent) : QMainWindow (parent), ui (new Ui::PCLViewer)
{
  ui->setupUi (this);

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->resize (200);

  // 更新时间线程
  timeUpdater = new TimeUpdater(this);
  // 更新CPU使用率线程
//  cpuMonitor = new CPUMonitor(this);

  connect(timeUpdater, &TimeUpdater::updateTimeSignal, this, &PCLViewer::receiveUpdateTime);
//  connect(cpuMonitor, &CPUMonitor::CPUMonitor, this, &PCLViewer::receiveUpdateTime);

  timeUpdater->start();
//  cpuMonitor->start();

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
  connect (ui->runButton,  SIGNAL (clicked ()), this, SLOT (runButtonPressed ()));

  // 点击退出按钮
  connect (ui->exitButton,  SIGNAL (clicked ()), this, SLOT (exitViewer ()));
  // 点击连接相机按钮
  connect (ui->connectCameraButton,  SIGNAL (clicked ()), this, SLOT (connectCameraButton ()));
  // 点击连接机器人按钮
  connect (ui->connectRobotButton,  SIGNAL (clicked ()), this, SLOT (connectRobotButton ()));

  // 点击进入垛型编辑界面
  connect (ui->conterButton,  SIGNAL (clicked ()), this, SLOT (on_TiaoZhuan_clicked ()));
//  connect(this, SIGNAL(sendData(QString)), ui->conterButton, SLOT(getDataFromMainW(QString)));

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
//        qApp->quit();
     }
     else
     {
        // 弹窗输入相机IP
        cameraIP = QInputDialog::getText(nullptr, "输入相机IP", "请输入相机IP地址:", QLineEdit::Normal, "", &camera_ok);

        if (!camera_ok || cameraIP.isEmpty())
        {
            // 用户取消输入，执行退出操作
//            qApp->quit();
        }
        else
        {
            // 显示网口连接状态
            QString strTCP = "TCP Server\n";
            QString strTCPStatus = "设备已接入";
            QString strCameraTCP = strTCP + strTCPStatus;
            ui->TCPServer->setText(strCameraTCP);

            // 通过IP连接相机
            mmind::eye::ErrorStatus status = mecheyecamera.connect(cameraIP.toStdString());

            if (status.isOK())
            {
                // 连接成功，退出循环
                std::cout << "连接成功" << std::endl;
                ui->runStatus->setText("相机连接成功");

                // 显示相机连接状态
                QString strMAC = "主机序列号 \n";
                QString strIP = "IP:  ";
                QString strCameraStatus = " 已连接";
                QString strCamera = strMAC + strIP + cameraIP + strCameraStatus;
                ui->cameraStatus->setText(strCamera);
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
 * @brief 连接机器人
 *
 * @param None
 * @return None
 */
void PCLViewer::connectRobotButton ()
{

    // 创建 motionSocket
    QTcpSocket* motionSocket = new QTcpSocket(this);
    motionSocket->connectToHost(QHostAddress(SERVER_IP), MOTION_PORT);

    if (!motionSocket->waitForConnected(3000))
    {
        std::cerr << "Error connecting to motion server" << std::endl;
        delete motionSocket;
        return;
    }

    // 发送切换机器人工具控制命令
//    motionSocket.write("83,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "83,");
    TCP_connection(STATUS_PORT);
    // 显示相机连接状态



    // 设置TCP
//    motionSocket.write("6, 10, 10, 10, 0, 0, 0, 100, 100,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "6, 10, 10, 10, 0, 0, 0, 100, 100,");
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 延时5s

    std::cout << "工具TCP: " << std::endl;
    // 获取工具坐标系
//    motionSocket.write("9,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "9,");

    std::cout << "工具坐标: " << std::endl;
    // 获取工件坐标系
//    motionSocket.write("11,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "11,");


    std::cout << "运动位姿: 224.917, 662.782, 384.972, -57.200, 176.259, -150.652" << std::endl;
//    motionSocket.write("2, 224.917, 662.782, 384.972, -57.200, 176.259, -150.652, 50, 50, 50,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "2, 224.917, 662.782, 384.972, -57.200, 176.259, -150.652, 50, 50, 50,");

    std::cout << "工具坐标: " << std::endl;
    // 获取工件坐标系
//    motionSocket.write("11,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "11,");

    std::this_thread::sleep_for(std::chrono::seconds(1)); // 延时1s

    // 关闭连接
//    motionSocket.write("82,");
//    motionSocket.waitForBytesWritten();
    sendCommand(*motionSocket, "82,");

    // 断开连接
    motionSocket->disconnectFromHost();
    delete motionSocket;

    // 退出31401端口监听
    status_flag = true;

    QString strRobotStatus = " 机器人已断开";
    ui->robotLabel->setText(strRobotStatus);
    std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时1s
}



/**
 * @brief 连接相机并获取彩色图、深度图像和点云，并作可视化显示
 *
 * @param None
 * @return None
 */
void PCLViewer::runButtonPressed ()
{
    if (!camera_ok || cameraIP.isEmpty())
    {
        QMessageBox::critical(nullptr, "连接异常", "请检测相机连接是否正常.", QMessageBox::Ok);

        // 没有连接相机时，不操作
        //  qApp->quit();
    }
    else
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

/**
 * @brief TCP连接
 *
 * @param port 端口
 * @return None
 */
void PCLViewer::TCP_connection(int port)
{
    QString strRobotStatus = " 机器人已连接";
    ui->robotLabel->setText(strRobotStatus);

    QTcpSocket socket;
    try
    {
        socket.connectToHost(QHostAddress(SERVER_IP), port);
        if (!socket.waitForConnected(3000))
        {
            std::cerr << "Error connecting to server" << std::endl;
            return;
        }
        else
        {
            if (status_flag)
            {
                socket.disconnectFromHost();
                std::cout << "Discovering all available robot..." << std::endl;

            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}


/**
 * @brief 发送数据函数
 *
 * @param socket socket_id
 * @param command 命令
 * @return None
 */
void PCLViewer::sendCommand(QTcpSocket& socket, const QString& command)
{
    QByteArray data = command.toUtf8();
    socket.write(data);
    if (!socket.waitForBytesWritten(3000))
    {
        std::cerr << "Error: Failed to write data to socket" << std::endl;
    }
}

void Container::returnToConter()
{
    this->show(); // 显示主界面
}


void PCLViewer::on_TiaoZhuan_clicked()
{
    emit returnToConter();
    Container *aw = new Container(); // 创建界面A的实例
    connect(aw, &Container::returnToMain, this, &PCLViewer::show); // 连接界面A的返回主界面信号和主界面的显示槽函数
    aw->show(); // 显示界面A
    this->hide(); // 隐藏主界面
}


