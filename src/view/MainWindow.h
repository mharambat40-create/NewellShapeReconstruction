#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "controller/ApplicationState.h"

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void importPointCloud();

private:
    Ui::MainWindow *ui;
    ApplicationState applicationState_;
};
#endif // MAINWINDOW_H
