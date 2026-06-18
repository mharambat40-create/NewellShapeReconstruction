#include "view/MainWindow.h"
#include "view/PointCloudViewport.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto *layout = new QVBoxLayout(ui->centralwidget);
    layout->setContentsMargins(0, 0, 0, 0);

    viewport_ = new PointCloudViewport(ui->centralwidget);
    layout->addWidget(viewport_);

    connect(ui->actionImportPointCloud, &QAction::triggered, this, &MainWindow::importPointCloud);
    statusBar()->showMessage("No geometry loaded");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::importPointCloud()
{
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        "Import ASCII PLY Point Cloud",
        QString(),
        "PLY files (*.ply)");

    if (filePath.isEmpty()) {
        return;
    }

    const LoadPointCloudResult result = applicationState_.loadPointCloudFromFile(
        std::filesystem::path(filePath.toUtf8().constData()));

    if (!result.success) {
        QMessageBox::critical(
            this,
            "Import failed",
            QString::fromStdString(result.errorMessage));
        statusBar()->showMessage("Import failed");
        return;
    }

    const QString successMessage = QString("Loaded %1 point(s)").arg(result.pointCount);
    viewport_->setPointCloud(applicationState_.currentPointCloud());
    QMessageBox::information(this, "Import successful", successMessage);
    statusBar()->showMessage(successMessage);
}
