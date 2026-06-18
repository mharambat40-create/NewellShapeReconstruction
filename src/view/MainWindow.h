#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "controller/ApplicationState.h"

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class PointCloudViewport;
class FloatingWorkflowMenu;
class PreprocessingSideMenu;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void importPointCloud();
    void showPlaceholderFeatureMessage(const QString &featureName);
    void handleWorkflowStepSelection(const QString &stepName);

private:
    void positionOverlayMenu();

    Ui::MainWindow *ui;
    ApplicationState applicationState_;
    PointCloudViewport *viewport_ = nullptr;
    FloatingWorkflowMenu *workflowMenu_ = nullptr;
    PreprocessingSideMenu *preprocessingSideMenu_ = nullptr;
};
#endif // MAINWINDOW_H
