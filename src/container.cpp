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

void Container::on_Return_clicked()
{
    PCLViewer *mw = new PCLViewer();
    mw->show();
    this->close();
}
