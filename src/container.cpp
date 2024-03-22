#include "container.h"
#include "ui_container.h"
#include "pclviewer.h"
#include <QMessageBox>

Container::Container(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Container)
{
    ui->setupUi(this);
    connect (ui->backButton,  SIGNAL (clicked ()), this, SLOT (on_Return_clicked ()));

}

Container::~Container()
{
    delete ui;
}

void Container::getDataFromMainW(QString data)
{
    QMessageBox mes(this);
    mes.setText(data);
    mes.exec();
}

//void Container::on_Return_clicked()
//{
//    PCLViewer *mw = new PCLViewer();
//    mw->show();
//    this->hide();
//}



void Container::on_Return_clicked()
{
    emit returnToMain(); // 发送返回主界面的信号
    this->hide(); // 隐藏界面A
}


void PCLViewer::returnToMain()
{
    this->show(); // 显示主界面
}

void Container::on_TiaoZhuan_clicked()
{
    emit returnToMain(); // 发送返回主界面的信号
    this->hide(); // 隐藏界面A
}
