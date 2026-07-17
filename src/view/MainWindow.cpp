#include "controller/PreprocessingController.h"
#include "controller/SurfaceReconstructionController.h"
#include "model/processing/normals/NormalMethodRegistry.h"
#include "model/processing/reconstruction/common/ProjectionPlaneUtilities.h"
#include "model/processing/reconstruction/SurfaceReconstructorRegistry.h"
#include "view/FloatingWorkflowMenu.h"
#include "view/MainWindow.h"
#include "view/PreprocessingSideMenu.h"
#include "view/PointCloudViewport.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileDialog>
#include <QFutureWatcher>
#include <QMessageBox>
#include <QMetaObject>
#include <QResizeEvent>
#include <QString>
#include <QtConcurrent/QtConcurrentRun>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <optional>
#include <system_error>
#include <utility>

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

std::optional<std::filesystem::path> defaultSamplePointCloudPath()
{
    std::error_code errorCode;
    const auto findDefaultFile = [&errorCode](const std::filesystem::path &startDirectory)
        -> std::optional<std::filesystem::path> {
        const auto sampleGeometry = findSampleGeometryFrom(startDirectory);
        if (!sampleGeometry) {
            return std::nullopt;
        }

        const std::filesystem::path corruptedCube = *sampleGeometry / "Cube_corrupted.ply";
        return std::filesystem::is_regular_file(corruptedCube, errorCode)
            ? std::optional<std::filesystem::path>(corruptedCube)
            : std::nullopt;
    };

    if (const auto corruptedCube = findDefaultFile(std::filesystem::current_path(errorCode))) {
        return corruptedCube;
    }

    return findDefaultFile(std::filesystem::path(QCoreApplication::applicationDirPath().toStdString()));
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
    connect(
        &autoSelectWatcher_,
        &QFutureWatcher<PreprocessingPipelineResult>::finished,
        this,
        &MainWindow::handleAutoSelectFinished);
    connect(
        &reconstructionWatcher_,
        &QFutureWatcher<ReconstructionPipelineResult>::finished,
        this,
        &MainWindow::handleSurfaceReconstructionFinished);

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
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::removeDuplicatesMenuOpened,
        this,
        &MainWindow::handleRemoveDuplicatesMenuOpened);
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
        &PreprocessingSideMenu::placeholderMessageRequested,
        this,
        [this](const QString &title, const QString &message) {
            QMessageBox::information(this, title, message);
        });
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
        &PreprocessingSideMenu::autoSelectPerfectDuplicatesRequested,
        this,
        &MainWindow::handleAutoSelectPerfectDuplicates);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::autoSelectNearDuplicatesRequested,
        this,
        &MainWindow::handleAutoSelectNearDuplicates);
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
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::surfaceReconstructionRequested,
        this,
        &MainWindow::handleSurfaceReconstructionRequested);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::applySurfaceReconstructionRequested,
        this,
        &MainWindow::applySurfaceReconstruction);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::closeSurfaceReconstructionRequested,
        this,
        &MainWindow::closeSurfaceReconstruction);
    connect(
        preprocessingSideMenu_,
        &PreprocessingSideMenu::cancelSurfaceReconstructionRequested,
        this,
        &MainWindow::cancelSurfaceReconstruction);

    loadDefaultPointCloud();
    positionOverlayMenu();
}

MainWindow::~MainWindow()
{
    preprocessingCancellationSource_.requestCancellation();
    reconstructionCancellationSource_.requestCancellation();
    autoSelectWatcher_.waitForFinished();
    reconstructionWatcher_.waitForFinished();
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

    const LoadPointCloudResult result = loadPointCloudFile(
        std::filesystem::path(filePath.toUtf8().constData()));

    if (!result.success) {
        QMessageBox::critical(
            this,
            "Import failed",
            QString::fromStdString(result.errorMessage));
        return;
    }

    const QString successMessage = QString("Loaded %1 point(s)").arg(result.pointCount);
    QMessageBox::information(this, "Import successful", successMessage);
}

void MainWindow::loadDefaultPointCloud()
{
    const auto corruptedCube = defaultSamplePointCloudPath();
    if (!corruptedCube) {
        return;
    }

    const LoadPointCloudResult result = loadPointCloudFile(*corruptedCube);
    if (!result.success) {
        qWarning().noquote() << "Could not load the default sample point cloud:"
                             << QString::fromStdString(result.errorMessage);
    }
}

LoadPointCloudResult MainWindow::loadPointCloudFile(const std::filesystem::path &filePath)
{
    clearSelectedPoints();
    const LoadPointCloudResult result = applicationState_.loadPointCloudFromFile(filePath);
    if (!result.success) {
        return result;
    }

    endSelectionSession();
    refreshViewportPointCloud(true);
    return result;
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
    beginSelectionSession(SelectionSessionMode::RemoveInvalidPoints);
}

void MainWindow::handleRemoveDuplicatesMenuOpened()
{
    beginSelectionSession(SelectionSessionMode::RemoveDuplicates);
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

    PreprocessingPipelineParameters parameters;
    parameters.operation = PreprocessingOperation::SelectSparsePoints;
    parameters.radius = radiusMax;
    parameters.minimumNeighbourCount = minimumNeighbourCount;
    startPreprocessing(parameters);
}

void MainWindow::handleAutoSelectPerfectDuplicates()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const PointCloud *currentCloud = applicationState_.currentPointCloud();
    if (!currentCloud || currentCloud->empty()) {
        QMessageBox::warning(this, "Auto-select failed", "Load a point cloud before auto-selecting.");
        return;
    }

    PreprocessingPipelineParameters parameters;
    parameters.operation = PreprocessingOperation::SelectPerfectDuplicates;
    startPreprocessing(parameters);
}

void MainWindow::handleAutoSelectNearDuplicates(double distanceThreshold)
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const PointCloud *currentCloud = applicationState_.currentPointCloud();
    if (!currentCloud || currentCloud->empty()) {
        QMessageBox::warning(this, "Auto-select failed", "Load a point cloud before auto-selecting.");
        return;
    }

    if (distanceThreshold <= 0.0) {
        QMessageBox::warning(this, "Auto-select failed", "Distance threshold must be greater than zero.");
        return;
    }

    PreprocessingPipelineParameters parameters;
    parameters.operation = PreprocessingOperation::SelectNearDuplicates;
    parameters.distanceThreshold = distanceThreshold;
    startPreprocessing(parameters);
}

void MainWindow::handleAutoSelectFinished()
{
    const PreprocessingPipelineResult result = autoSelectWatcher_.result();
    setProcessingState(ProcessingState::Idle);

    if (pendingAutoSelectRevision_ != applicationState_.geometryRevision()) {
        preprocessingSideMenu_->clearProcessingProgress();
        QMessageBox::warning(
            this,
            "Auto-select discarded",
            "The point cloud changed while auto-selection was running. The result was discarded.");
        return;
    }

    if (!result.succeeded) {
        preprocessingSideMenu_->clearProcessingProgress();
        if (result.cancelled) {
            return;
        }
        QMessageBox::warning(this, "Auto-select failed", QString::fromStdString(result.errorMessage));
        return;
    }

    addSelectedPointIndices(result.selectedIndices);
    preprocessingSideMenu_->completeProcessingProgress();
}

void MainWindow::handleSurfaceReconstructionRequested(
    const SurfaceConversionSettings &settings)
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const PointCloud *currentCloud = applicationState_.currentPointCloud();
    if (!currentCloud || currentCloud->empty()) {
        QMessageBox::warning(this, "Convert to surface", "Load a point cloud before reconstruction.");
        return;
    }
    const std::optional<ReconstructionMethod> method =
        SurfaceReconstructorRegistry::methodFromDisplayName(settings.surfaceMethod.toStdString());
    if (!method) {
        QMessageBox::warning(this, "Convert to surface", "The selected method is not registered.");
        return;
    }
    const ReconstructionAvailability availability = SurfaceReconstructorRegistry().availability(*method);
    if (!availability.available) {
        QMessageBox::information(
            this,
            "Convert to surface",
            QString::fromStdString(availability.reason));
        return;
    }

    ReconstructionPipelineParameters parameters;
    parameters.method = *method;
    const ReconstructionRequirements reconstructionRequirements =
        SurfaceReconstructorRegistry().requirements(*method);
    if (reconstructionRequirements.normals != NormalRequirement::NotUsed) {
        const std::optional<NormalMethod> normalMethod =
            NormalMethodRegistry::methodFromDisplayName(settings.normalMethod.toStdString());
        if (!normalMethod ||
            NormalMethodRegistry().requirements(*normalMethod).category !=
                NormalMethodCategory::Estimation) {
            QMessageBox::warning(
                this,
                "Convert to surface",
                "Select a compatible normal-estimation method first.");
            return;
        }
        parameters.normalProcessing.estimationMethod = *normalMethod;
        switch (*normalMethod) {
        case NormalMethod::PcaKNearest: {
            PcaKnnParameters normalParameters;
            normalParameters.neighbourCount =
                static_cast<std::size_t>(std::max(settings.neighbourCount, 3));
            normalParameters.maximumSearchRadius = settings.searchRadius;
            parameters.normalEstimation = normalParameters;
            parameters.normalProcessing.estimationParameters = normalParameters;
            break;
        }
        case NormalMethod::PcaFixedRadius: {
            PcaRadiusParameters normalParameters;
            normalParameters.searchRadius = settings.searchRadius;
            normalParameters.minimumNeighbours = 6U;
            normalParameters.maximumNeighbours =
                static_cast<std::size_t>(std::max(settings.neighbourCount, 6));
            parameters.normalProcessing.estimationParameters = normalParameters;
            break;
        }
        case NormalMethod::PcaMultiScale:
            parameters.normalProcessing.estimationParameters = MultiScalePcaParameters{};
            break;
        case NormalMethod::QuadraticSurfaceFit: {
            QuadraticFitParameters normalParameters;
            normalParameters.neighbourCount =
                static_cast<std::size_t>(std::max(settings.neighbourCount, 6));
            parameters.normalProcessing.estimationParameters = normalParameters;
            break;
        }
        default:
            QMessageBox::warning(
                this,
                "Convert to surface",
                "The selected normal method cannot estimate normals from this point cloud.");
            return;
        }
    }

    if (*method == ReconstructionMethod::BallPivoting) {
        if (settings.searchRadius <= 0.0 || settings.neighbourCount < 3 ||
            settings.scaleParameter <= 0.0) {
            QMessageBox::warning(
                this,
                "Convert to surface",
                "Ball Pivoting requires positive radii, at least three neighbours and a compatible normal estimator.");
            return;
        }
        parameters.reconstruction.ballRadius = settings.scaleParameter;
        parameters.methodParameters = parameters.reconstruction;
    } else if (*method == ReconstructionMethod::GreedyProjection) {
        if (settings.searchRadius <= 0.0 || settings.neighbourCount < 3) {
            QMessageBox::warning(
                this,
                "Convert to surface",
                "Greedy Projection requires a positive radius, at least three neighbours and a compatible normal estimator.");
            return;
        }
        GreedyProjectionParameters greedyParameters;
        greedyParameters.searchRadius = settings.searchRadius;
        greedyParameters.maximumNeighbours = static_cast<std::size_t>(settings.neighbourCount);
        parameters.methodParameters = greedyParameters;
    } else if (*method == ReconstructionMethod::Voxel) {
        if (settings.scaleParameter <= 0.0) {
            QMessageBox::warning(this, "Convert to surface", "Voxel size must be positive.");
            return;
        }
        VoxelReconstructionParameters voxelParameters;
        voxelParameters.voxelSize = settings.scaleParameter;
        parameters.methodParameters = voxelParameters;
    } else if (*method == ReconstructionMethod::Delaunay25D) {
        Delaunay25DParameters delaunayParameters;
        delaunayParameters.mode = settings.preservePlanarHeight
            ? PlanarReconstructionMode::HeightField25D
            : PlanarReconstructionMode::Planar2D;
        delaunayParameters.planarityTolerance = settings.planarityTolerance;
        parameters.methodParameters = delaunayParameters;
    } else if (*method == ReconstructionMethod::AlphaShapes) {
        AlphaShapeParameters alphaParameters;
        alphaParameters.mode = settings.preservePlanarHeight
            ? PlanarReconstructionMode::HeightField25D
            : PlanarReconstructionMode::Planar2D;
        alphaParameters.planarityTolerance = settings.planarityTolerance;
        alphaParameters.alpha = settings.alphaValue;
        alphaParameters.automaticAlpha = settings.automaticAlpha;
        alphaParameters.automaticAlphaFactor = settings.automaticAlphaFactor;
        alphaParameters.keepLargestComponentOnly = settings.keepLargestComponentOnly;
        alphaParameters.minimumComponentArea = settings.minimumComponentArea;
        parameters.methodParameters = alphaParameters;
    } else if (*method == ReconstructionMethod::Rbf) {
        parameters.methodParameters = RbfParameters{};
    } else {
        QMessageBox::information(
            this,
            "Convert to surface",
            "This backend is not available from the point-cloud conversion workflow.");
        return;
    }

    pendingReconstructionRevision_ = applicationState_.geometryRevision();
    PointCloud pointCloudSnapshot = *currentCloud;
    applicationState_.discardTemporaryReconstructedMesh();
    viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
    reconstructionCancellationSource_ = CancellationSource();
    const CancellationToken cancellationToken = reconstructionCancellationSource_.token();
    const std::size_t generation = ++processingGeneration_;
    setProcessingState(ProcessingState::Reconstructing);
    preprocessingSideMenu_->beginProcessingProgress();
    reconstructionWatcher_.setFuture(QtConcurrent::run(
        [this,
         pointCloudSnapshot = std::move(pointCloudSnapshot),
         parameters,
         cancellationToken,
         generation]() {
            const SurfaceReconstructionController controller;
            const ProgressCallback progressCallback = [this, generation](const ProcessingProgress &progress) {
                queueProcessingProgress(generation, progress);
            };
            return controller.reconstruct(
                pointCloudSnapshot,
                parameters,
                progressCallback,
                cancellationToken);
        }));
}

void MainWindow::handleSurfaceReconstructionFinished()
{
    ReconstructionPipelineResult result = reconstructionWatcher_.result();
    setProcessingState(ProcessingState::Idle);

    if (pendingReconstructionRevision_ != applicationState_.geometryRevision()) {
        preprocessingSideMenu_->clearProcessingProgress();
        QMessageBox::warning(
            this,
            "Convert to surface",
            "The point cloud changed during reconstruction. The mesh was discarded.");
        return;
    }
    if (!result.succeeded) {
        preprocessingSideMenu_->clearProcessingProgress();
        if (result.cancelled) {
            return;
        }
        QMessageBox::warning(
            this,
            "Surface reconstruction failed",
            QString::fromStdString(result.errorMessage));
        return;
    }

    const std::size_t triangleCount = result.mesh.triangleCount();
    const auto resultTransferStart = std::chrono::steady_clock::now();
    applicationState_.setTemporaryReconstructedMesh(std::move(result.mesh));
    viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
    result.timings.resultTransferMilliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - resultTransferStart).count();
    preprocessingSideMenu_->completeProcessingProgress();
    qDebug().nospace()
        << "Surface conversion timings (ms): preparation=" << result.timings.preparationMilliseconds
        << ", spatial index=" << result.timings.spatialIndexMilliseconds
        << ", normals=" << result.timings.normalEstimationMilliseconds
        << ", normal orientation=" << result.timings.normalOrientationMilliseconds
        << ", BPA seed search=" << result.timings.seedSearchMilliseconds
        << ", BPA propagation=" << result.timings.frontPropagationMilliseconds
        << ", mesh validation=" << result.timings.meshValidationMilliseconds
        << ", result construction=" << result.timings.resultConstructionMilliseconds
        << ", view transfer=" << result.timings.resultTransferMilliseconds;
    QMessageBox::information(
        this,
        "Surface reconstruction",
        QString("Created a temporary mesh with %1 triangle(s). %2 point(s) had invalid normals.")
            .arg(triangleCount)
            .arg(result.invalidNormalCount));
}

void MainWindow::applySurfaceReconstruction()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }
    if (!applicationState_.commitTemporaryReconstructedMesh()) {
        QMessageBox::information(this, "Convert to surface", "No temporary mesh is available to apply.");
        return;
    }

    viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
    preprocessingSideMenu_->clearProcessingProgress();
}

void MainWindow::closeSurfaceReconstruction()
{
    if (processingState_ == ProcessingState::Reconstructing) {
        reconstructionCancellationSource_.requestCancellation();
        return;
    }
    if (applicationState_.hasTemporaryReconstructedMesh()) {
        applicationState_.discardTemporaryReconstructedMesh();
        viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
    }
    preprocessingSideMenu_->clearProcessingProgress();
}

void MainWindow::cancelSurfaceReconstruction()
{
    if (processingState_ == ProcessingState::Reconstructing) {
        reconstructionCancellationSource_.requestCancellation();
        return;
    }
    applicationState_.discardTemporaryReconstructedMesh();
    viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
    preprocessingSideMenu_->clearProcessingProgress();
}

void MainWindow::applyRemoveInvalidPoints()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    const RemovePointResult result = applicationState_.removeCurrentPointIndices(selectedPointIndices_);
    if (!result.success) {
        QMessageBox::warning(this, currentSelectionSessionTitle(), QString::fromStdString(result.errorMessage));
        return;
    }

    clearSelectedPoints();
    refreshViewportPointCloud(false);
    preprocessingSideMenu_->clearProcessingProgress();

    const QString message = QString("Removed %1 selected points. Remaining points: %2.")
                                .arg(result.removedCount)
                                .arg(result.remainingPointCount);
    QMessageBox::information(this, currentSelectionSessionTitle(), message);
}

void MainWindow::confirmRemoveInvalidPoints()
{
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    endSelectionSession();
    clearSelectedPoints();
    preprocessingSideMenu_->clearProcessingProgress();
    preprocessingSideMenu_->showOperationList();
}

void MainWindow::cancelRemoveInvalidPoints()
{
    if (processingState_ == ProcessingState::AutoSelecting) {
        preprocessingCancellationSource_.requestCancellation();
        return;
    }
    if (processingState_ != ProcessingState::Idle) {
        return;
    }

    if (selectionSessionActive_ && selectionSessionSnapshot_) {
        applicationState_.restoreCurrentPointCloud(*selectionSessionSnapshot_);
        refreshViewportPointCloud(false);
    }

    endSelectionSession();
    clearSelectedPoints();
    preprocessingSideMenu_->clearProcessingProgress();
    preprocessingSideMenu_->showOperationList();
}

void MainWindow::clearSelectedPoints()
{
    selectedPointIndices_.clear();
    setManualPointSelectionActive(false);
    viewport_->setSelectedPointIndices(selectedPointIndices_);
    preprocessingSideMenu_->setPointSelectionResultAvailable(false);
}

void MainWindow::refreshViewportPointCloud(bool fitView)
{
    const PointCloud *pointCloud = applicationState_.currentPointCloud();
    if (pointCloud) {
        viewport_->setPointCloud(pointCloud, fitView);
    } else {
        viewport_->clearPointCloud();
    }

    viewport_->setSelectedPointIndices(selectedPointIndices_);
    preprocessingSideMenu_->setPointSelectionResultAvailable(!selectedPointIndices_.empty());
    std::optional<double> planarityIndicator;
    QString planarityError;
    const bool validSurfaceInput = pointCloud && pointCloud->pointCount() >= 3U;
    if (validSurfaceInput) {
        const PlanarProjectionResult projection = computePlanarProjection(*pointCloud, 0.999999);
        if (projection.succeeded) {
            planarityIndicator = projection.projection.planarityIndicator;
        } else {
            planarityError = QString::fromStdString(projection.errorMessage);
        }
    }
    preprocessingSideMenu_->setSurfaceInputState(
        validSurfaceInput,
        planarityIndicator,
        planarityError);
    viewport_->setTriangleMesh(applicationState_.displayedReconstructedMesh());
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
    preprocessingSideMenu_->setPointSelectionResultAvailable(!selectedPointIndices_.empty());
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
    preprocessingSideMenu_->setPointSelectionResultAvailable(!selectedPointIndices_.empty());
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
    preprocessingSideMenu_->setPointSelectionResultAvailable(!selectedPointIndices_.empty());
}

void MainWindow::beginSelectionSession(SelectionSessionMode mode)
{
    selectionSessionActive_ = true;
    selectionSessionMode_ = mode;

    if (const PointCloud *pointCloud = applicationState_.currentPointCloud()) {
        selectionSessionSnapshot_ = *pointCloud;
    } else {
        selectionSessionSnapshot_.reset();
    }
}

void MainWindow::endSelectionSession()
{
    selectionSessionActive_ = false;
    selectionSessionMode_ = SelectionSessionMode::None;
    selectionSessionSnapshot_.reset();
}

QString MainWindow::currentSelectionSessionTitle() const
{
    switch (selectionSessionMode_) {
    case SelectionSessionMode::RemoveDuplicates:
        return "Downsampling";
    case SelectionSessionMode::RemoveInvalidPoints:
    case SelectionSessionMode::None:
        return "Remove invalid points";
    }

    return "Pre-processing";
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

void MainWindow::startPreprocessing(const PreprocessingPipelineParameters &parameters)
{
    const PointCloud *currentCloud = applicationState_.currentPointCloud();
    if (!currentCloud || currentCloud->empty()) {
        return;
    }

    pendingAutoSelectRevision_ = applicationState_.geometryRevision();
    PointCloud pointCloudSnapshot = *currentCloud;
    preprocessingCancellationSource_ = CancellationSource();
    const CancellationToken cancellationToken = preprocessingCancellationSource_.token();
    const std::size_t generation = ++processingGeneration_;

    setManualPointSelectionActive(false);
    setProcessingState(ProcessingState::AutoSelecting);
    preprocessingSideMenu_->beginProcessingProgress();
    autoSelectWatcher_.setFuture(QtConcurrent::run(
        [this,
         pointCloudSnapshot = std::move(pointCloudSnapshot),
         parameters,
         cancellationToken,
         generation]() {
            const PreprocessingController controller;
            const ProgressCallback progressCallback = [this, generation](const ProcessingProgress &progress) {
                queueProcessingProgress(generation, progress);
            };
            return controller.process(
                pointCloudSnapshot,
                parameters,
                progressCallback,
                cancellationToken);
        }));
}

void MainWindow::queueProcessingProgress(
    std::size_t generation,
    const ProcessingProgress &progress)
{
    QMetaObject::invokeMethod(
        this,
        [this, generation, progress] {
            if (generation != processingGeneration_ || processingState_ == ProcessingState::Idle) {
                return;
            }
            preprocessingSideMenu_->updateProcessingProgress(progress);
        },
        Qt::QueuedConnection);
}
