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

private:
    Ui::Container *ui;
};

#endif // CONTAINER_H
