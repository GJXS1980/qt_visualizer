#ifndef CONTAINER_H
#define CONTAINER_H

#include <QMainWindow>

namespace Ui {
class Container;
}

class Container : public QMainWindow
{
    Q_OBJECT

public:
    explicit Container(QWidget *parent = nullptr);
    ~Container();

public slots:
    void getDataFromMainW(QString data);

private slots:
    void on_Return_clicked();
    void on_TiaoZhuan_clicked(); // 添加新的槽函数声明

signals:
    void returnToMain(); // 声明返回主界面的信号

private:
    Ui::Container *ui;
};

#endif // CONTAINER_H
