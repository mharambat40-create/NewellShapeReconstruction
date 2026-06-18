#include "view/FloatingWorkflowMenu.h"
#include "view/MainWindow.h"
#include "view/PreprocessingSideMenu.h"
#include "view/PointCloudViewport.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QMessageBox>
#include <QResizeEvent>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <system_error>

namespace
{
std::optional<std::filesystem::path> findSampleGeometryFrom(
    const std::filesystem::path &startDirectory)
{
    if (startDirectory.empty()) {
        return std::nullopt;
    }

    std::error_code errorCode;
    std::filesystem::path current = std::filesystem::weakly_canonical(startDirectory, errorCode);
    if (errorCode) {
        current = startDirectory;
    }

    for (int depth = 0; depth < 8; ++depth) {
        const std::filesystem::path candidate = current / "assets" / "sample_geometry";
        if (std::filesystem::is_directory(candidate, errorCode)) {
            return candidate;
        }

        errorCode.clear();
        if (!current.has_parent_path() || current == current.parent_path()) {
            break;
        }

        current = current.parent_path();
    }

    return std::nullopt;
}

QString defaultImportDirectory()
{
    std::error_code errorCode;
    if (const auto sampleGeometry = findSampleGeometryFrom(std::filesystem::current_path(errorCode))) {
        return QString::fromStdString(sampleGeometry->string());
    }

    if (const auto sampleGeometry = findSampleGeometryFrom(
            std::filesystem::path(QCoreApplication::applicationDirPath().toStdString()))) {
        return QString::fromStdString(sampleGeometry->string());
    }

    const QString currentDirectory = QDir::currentPath();
    return currentDirectory.isEmpty() ? QDir::homePath() : currentDirectory;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    auto *layout = new QVBoxLayout(ui->centralwidget);
    layout->setContentsMargins(0, 0, 0, 0);

    viewport_ = new PointCloudViewport(ui->centralwidget);
    layout->addWidget(viewport_);

    workflowMenu_ = new FloatingWorkflowMenu(ui->centralwidget);
    workflowMenu_->raise();
    preprocessingSideMenu_ = new PreprocessingSideMenu(ui->centralwidget);
    preprocessingSideMenu_->hideMenu();

    connect(ui->actionImportPointCloud, &QAction::triggered, this, &MainWindow::importPointCloud);
    connect(workflowMenu_, &FloatingWorkflowMenu::importRequested, this, &MainWindow::importPointCloud);
    connect(
        workflowMenu_,
        &FloatingWorkflowMenu::workflowStepSelected,
        this,
        &MainWindow::handleWorkflowStepSelection);

    positionOverlayMenu();
    statusBar()->showMessage("No geometry loaded");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    positionOverlayMenu();
}

void MainWindow::importPointCloud()
{
    const QString filePath = QFileDialog::getOpenFileName(
        this,
        "Import ASCII PLY Point Cloud",
        defaultImportDirectory(),
        "PLY files (*.ply);;All files (*)");

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

void MainWindow::showPlaceholderFeatureMessage(const QString &featureName)
{
    QMessageBox::information(
        this,
        featureName,
        featureName + " is not implemented yet.");
}

void MainWindow::positionOverlayMenu()
{
    if (!workflowMenu_ || !ui->centralwidget) {
        return;
    }

    workflowMenu_->adjustSize();
    workflowMenu_->raise();

    const int x = (ui->centralwidget->width() - workflowMenu_->width()) / 2;
    const int y = 12;
    workflowMenu_->move(std::max(0, x), y);

    if (!preprocessingSideMenu_) {
        return;
    }

    preprocessingSideMenu_->adjustSize();
    preprocessingSideMenu_->raise();

    const int sideMenuX = ui->centralwidget->width() - preprocessingSideMenu_->width() - 16;
    const int sideMenuY = workflowMenu_->geometry().bottom() + 12;
    preprocessingSideMenu_->move(std::max(0, sideMenuX), sideMenuY);
}

void MainWindow::handleWorkflowStepSelection(const QString &stepName)
{
    if (stepName == "Pre-processing") {
        workflowMenu_->setActiveWorkflowStep(stepName);
        preprocessingSideMenu_->showOperationList();
        positionOverlayMenu();
        return;
    }

    workflowMenu_->setActiveWorkflowStep(QString());
    preprocessingSideMenu_->hideMenu();

    if (stepName == "Import") {
        return;
    }

    showPlaceholderFeatureMessage(stepName);
}
