#include "model/preprocessing/InvalidPointSelection.h"
#include "view/FloatingWorkflowMenu.h"
#include "view/MainWindow.h"
#include "view/PreprocessingSideMenu.h"
#include "view/PointCloudViewport.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QFutureWatcher>
#include <QMessageBox>
#include <QResizeEvent>
#include <QString>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <system_error>

namespace
{
constexpr std::size_t kSparseAutoSelectBruteForcePointLimit = 50000U;

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
    resize(880, 660);

    auto *layout = new QVBoxLayout(ui->centralwidget);
    layout->setContentsMargins(0, 0, 0, 0);

    viewport_ = new PointCloudViewport(ui->centralwidget);
    layout->addWidget(viewport_);

    workflowMenu_ = new FloatingWorkflowMenu(ui->centralwidget);
    workflowMenu_->raise();
    preprocessingSideMenu_ = new PreprocessingSideMenu(ui->centralwidget);
    preprocessingSideMenu_->hideMenu();
    connect(&autoSelectWatcher_, &QFutureWatcher<SparseSelectionResult>::finished, this, &MainWindow::handleAutoSelectFinished);

    connect(ui->actionImportPointCloud, &QAction::triggered, this, &MainWindow::importPointCloud);
    connect(workflowMenu_, &FloatingWorkflowMenu::importRequested, this, &MainWindow::importPointCloud);
    connect(
        workflowMenu_,
        &FloatingWorkflowMenu::workflowStepSelected,
        this,
        &MainWindow::handleWorkflowStepSelection);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::removeInvalidPointsMenuOpened,
        this,
        &MainWindow::handleRemoveInvalidPointsMenuOpened);
    connect(viewport_, &PointCloudViewport::pointPicked, this, &MainWindow::handlePickedPoint);
    connect(viewport_, &PointCloudViewport::areaPointsSelected, this, &MainWindow::handleAreaPointsSelected);
    connect(viewport_, &PointCloudViewport::areaPointsDeselected, this, &MainWindow::handleAreaPointsDeselected);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::placeholderOperationRequested,
        this,
        &MainWindow::showPlaceholderFeatureMessage);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::manualSelectionToggled,
        this,
        &MainWindow::handleManualSelectionToggled);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::autoSelectSparsePointsRequested,
        this,
        &MainWindow::handleAutoSelectSparsePoints);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::applyRemoveInvalidPointsRequested,
        this,
        &MainWindow::applyRemoveInvalidPoints);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::confirmRemoveInvalidPointsRequested,
        this,
        &MainWindow::confirmRemoveInvalidPoints);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::cancelRemoveInvalidPointsRequested,
        this,
        &MainWindow::cancelRemoveInvalidPoints);

    positionOverlayMenu();
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
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const QString filePath = QFileDialog::getOpenFileName(
        this,
        "Import ASCII PLY Point Cloud",
        defaultImportDirectory(),
        "PLY files (*.ply);;All files (*)");

    if (filePath.isEmpty()) {
        return;
    }

    clearSelectedPoints();
    const LoadPointCloudResult result = applicationState_.loadPointCloudFromFile(
        std::filesystem::path(filePath.toUtf8().constData()));

    if (!result.success) {
        QMessageBox::critical(
            this,
            "Import failed",
            QString::fromStdString(result.errorMessage));
        return;
    }

    const QString successMessage = QString("Loaded %1 point(s)").arg(result.pointCount);
    endRemoveInvalidPointsSession();
    refreshViewportPointCloud(true);
    QMessageBox::information(this, "Import successful", successMessage);
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
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    workflowMenu_->setActiveWorkflowStep(stepName);

    if (stepName == "Pre-processing") {
        preprocessingSideMenu_->showOperationList();
        positionOverlayMenu();
        return;
    }

    preprocessingSideMenu_->hideMenu();
    clearSelectedPoints();

    if (stepName == "Import") {
        return;
    }

    showPlaceholderFeatureMessage(stepName);
}

void MainWindow::handleRemoveInvalidPointsMenuOpened()
{
    beginRemoveInvalidPointsSession();
}

void MainWindow::handlePickedPoint(std::size_t pointIndex)
{
    if (!manualPointSelectionEnabled_) {
        return;
    }

    toggleSelectedPoint(pointIndex);
}

void MainWindow::handleAreaPointsSelected(const std::vector<std::size_t> &pointIndices)
{
    if (!manualPointSelectionEnabled_) {
        return;
    }

    addSelectedPointIndices(pointIndices);
}

void MainWindow::handleAreaPointsDeselected(const std::vector<std::size_t> &pointIndices)
{
    if (!manualPointSelectionEnabled_) {
        return;
    }

    removeSelectedPointIndices(pointIndices);
}

void MainWindow::handleManualSelectionToggled(bool enabled)
{
    setManualPointSelectionActive(enabled);
}

void MainWindow::handleAutoSelectSparsePoints(double radiusMax, int minimumNeighbourCount)
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const PointCloud *currentCloud = applicationState_.currentPointCloud();
    if (!currentCloud || currentCloud->empty()) {
        QMessageBox::warning(this, "Auto-select failed", "Load a point cloud before auto-selecting.");
        return;
    }

    if (radiusMax <= 0.0) {
        QMessageBox::warning(this, "Auto-select failed", "Radius max must be greater than zero.");
        return;
    }

    if (minimumNeighbourCount < 0) {
        QMessageBox::warning(this, "Auto-select failed", "Number of neighbours cannot be negative.");
        return;
    }

    if (currentCloud->pointCount() > kSparseAutoSelectBruteForcePointLimit) {
        QMessageBox::warning(
            this,
            "Auto-select limited",
            QString(
                "Sparse-region auto-selection is currently limited to %1 points. "
                "Please downsample the point cloud first.")
                .arg(kSparseAutoSelectBruteForcePointLimit));
        return;
    }

    pendingAutoSelectRevision_ = applicationState_.geometryRevision();
    const PointCloud pointCloudSnapshot = *currentCloud;

    setManualPointSelectionActive(false);
    setProcessingState(ProcessingState::AutoSelecting);

    autoSelectWatcher_.setFuture(QtConcurrent::run(
        [pointCloudSnapshot, radiusMax, minimumNeighbourCount]() -> SparseSelectionResult {
            return SparseSelectionResult{
                true,
                {},
                selectSparseRegionPoints(pointCloudSnapshot, radiusMax, minimumNeighbourCount),
            };
        }));
}

void MainWindow::handleAutoSelectFinished()
{
    const SparseSelectionResult result = autoSelectWatcher_.result();
    setProcessingState(ProcessingState::Idle);

    if (pendingAutoSelectRevision_ != applicationState_.geometryRevision()) {
        QMessageBox::warning(
            this,
            "Auto-select discarded",
            "The point cloud changed while auto-selection was running. The result was discarded.");
        return;
    }

    if (!result.success) {
        QMessageBox::warning(this, "Auto-select failed", QString::fromStdString(result.errorMessage));
        return;
    }

    addSelectedPointIndices(result.selectedIndices);
}

void MainWindow::applyRemoveInvalidPoints()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const RemovePointResult result = applicationState_.removeCurrentPointIndices(selectedPointIndices_);
    if (!result.success) {
        QMessageBox::warning(this, "Remove invalid points", QString::fromStdString(result.errorMessage));
        return;
    }

    clearSelectedPoints();
    refreshViewportPointCloud(false);

    const QString message = QString("Removed %1 selected points. Remaining points: %2.")
                                .arg(result.removedCount)
                                .arg(result.remainingPointCount);
    QMessageBox::information(this, "Remove invalid points", message);
}

void MainWindow::confirmRemoveInvalidPoints()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    endRemoveInvalidPointsSession();
    clearSelectedPoints();
    preprocessingSideMenu_->showOperationList();
}

void MainWindow::cancelRemoveInvalidPoints()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    if (removeInvalidPointsSessionActive_ && removeInvalidPointsSnapshot_) {
        applicationState_.restoreCurrentPointCloud(*removeInvalidPointsSnapshot_);
        refreshViewportPointCloud(false);
    }

    endRemoveInvalidPointsSession();
    clearSelectedPoints();
    preprocessingSideMenu_->showOperationList();
}

void MainWindow::clearSelectedPoints()
{
    selectedPointIndices_.clear();
    setManualPointSelectionActive(false);
    viewport_->setSelectedPointIndices(selectedPointIndices_);
}

void MainWindow::refreshViewportPointCloud(bool fitView)
{
    if (const PointCloud *pointCloud = applicationState_.currentPointCloud()) {
        viewport_->setPointCloud(pointCloud, fitView);
    } else {
        viewport_->clearPointCloud();
    }

    viewport_->setSelectedPointIndices(selectedPointIndices_);
}

void MainWindow::toggleSelectedPoint(std::size_t pointIndex)
{
    const auto existing = std::lower_bound(
        selectedPointIndices_.begin(),
        selectedPointIndices_.end(),
        pointIndex);

    if (existing != selectedPointIndices_.end() && *existing == pointIndex) {
        selectedPointIndices_.erase(existing);
    } else {
        selectedPointIndices_.insert(existing, pointIndex);
    }

    viewport_->setSelectedPointIndices(selectedPointIndices_);
}

void MainWindow::addSelectedPointIndices(const std::vector<std::size_t> &pointIndices)
{
    selectedPointIndices_.insert(
        selectedPointIndices_.end(),
        pointIndices.begin(),
        pointIndices.end());
    std::sort(selectedPointIndices_.begin(), selectedPointIndices_.end());
    selectedPointIndices_.erase(
        std::unique(selectedPointIndices_.begin(), selectedPointIndices_.end()),
        selectedPointIndices_.end());
    viewport_->setSelectedPointIndices(selectedPointIndices_);
}

void MainWindow::removeSelectedPointIndices(const std::vector<std::size_t> &pointIndices)
{
    if (selectedPointIndices_.empty() || pointIndices.empty()) {
        return;
    }

    std::vector<std::size_t> sortedPointIndices = pointIndices;
    std::sort(sortedPointIndices.begin(), sortedPointIndices.end());
    sortedPointIndices.erase(
        std::unique(sortedPointIndices.begin(), sortedPointIndices.end()),
        sortedPointIndices.end());

    selectedPointIndices_.erase(
        std::remove_if(
            selectedPointIndices_.begin(),
            selectedPointIndices_.end(),
            [&sortedPointIndices](std::size_t pointIndex) {
                return std::binary_search(
                    sortedPointIndices.begin(),
                    sortedPointIndices.end(),
                    pointIndex);
            }),
        selectedPointIndices_.end());
    viewport_->setSelectedPointIndices(selectedPointIndices_);
}

void MainWindow::beginRemoveInvalidPointsSession()
{
    if (removeInvalidPointsSessionActive_) {
        return;
    }

    removeInvalidPointsSessionActive_ = true;
    removeInvalidPointsSnapshotSelection_ = selectedPointIndices_;

    if (const PointCloud *pointCloud = applicationState_.currentPointCloud()) {
        removeInvalidPointsSnapshot_ = *pointCloud;
    } else {
        removeInvalidPointsSnapshot_.reset();
    }
}

void MainWindow::endRemoveInvalidPointsSession()
{
    removeInvalidPointsSessionActive_ = false;
    removeInvalidPointsSnapshot_.reset();
    removeInvalidPointsSnapshotSelection_.clear();
}

void MainWindow::setManualPointSelectionActive(bool enabled)
{
    const bool selectionEnabled =
        enabled && processingState_ == ProcessingState::Idle;
    manualPointSelectionEnabled_ = selectionEnabled;
    viewport_->setSelectionModeEnabled(selectionEnabled);
    viewport_->setSelectionInteractionEnabled(selectionEnabled);

    if (preprocessingSideMenu_) {
        preprocessingSideMenu_->setManualSelectionEnabled(selectionEnabled);
    }
}

void MainWindow::setProcessingState(ProcessingState state)
{
    processingState_ = state;
    const bool idle = processingState_ == ProcessingState::Idle;

    ui->actionImportPointCloud->setEnabled(idle);

    if (workflowMenu_) {
        workflowMenu_->setInteractionEnabled(idle);
    }

    if (preprocessingSideMenu_) {
        preprocessingSideMenu_->setAutoSelectInProgress(!idle);
    }

    if (!idle) {
        setManualPointSelectionActive(false);
    }

    positionOverlayMenu();
}
