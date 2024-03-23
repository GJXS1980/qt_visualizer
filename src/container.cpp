#include "container.h"
#include "ui_container.h"
#include "pclviewer.h"
#include <QMessageBox>

#ifndef QT_NO_OPENGL
#include "window.h"
#endif


Container::Container(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Container)
{
    ui->setupUi(this);
    connect (ui->backButton,  SIGNAL (clicked ()), this, SLOT (on_Return_clicked ()));
    connect (ui->pushButton,  SIGNAL (clicked ()), this, SLOT (show_cube ()));


}

Container::~Container()
{
    delete ui;
}


/**
 * @brief 显示集装箱函数
 *
 * @param None
 * @return None
 */
void Container::show_cube()
{
    // 创建一个OpenGL窗口并显示立方体
    Window *win = new Window(this);
    // 创建一个布局，并将窗口添加到布局中
    QVBoxLayout *layout = new QVBoxLayout(ui->demoShow);
    layout->addWidget(win);
    ui->demoShow->setLayout(layout);

    // 显示 OpenGL 窗口
    win->show();
}


/**
 * @brief 从主界面获取数据
 *
 * @param data 主界面传输的数据
 * @return None
 */
void Container::getDataFromMainW(QString data)
{
    QMessageBox mes(this);
    mes.setText(data);
    mes.exec();
}


/**
 * @brief 返回主界面函数
 *
 * @param None
 * @return None
 */
void Container::on_Return_clicked()
{
    emit returnToMain(); // 发送返回主界面的信号
    this->hide(); // 隐藏界面A
}


/**
 * @brief 显示主界面函数
 *
 * @param None
 * @return None
 */
void PCLViewer::returnToMain()
{
    this->show(); // 显示主界面
}


/**
 * @brief 返回主界面函数
 *
 * @param None
 * @return None
 */
//void Container::on_TiaoZhuan_clicked()
//{
//    emit returnToMain(); // 发送返回主界面的信号
//    this->hide(); // 隐藏界面A
//}




