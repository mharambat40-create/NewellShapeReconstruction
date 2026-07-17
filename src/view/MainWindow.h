#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "controller/ApplicationState.h"
#include "model/pipeline/PreprocessingPipeline.h"
#include "model/pipeline/ReconstructionPipeline.h"
#include "model/processing/common/CancellationToken.h"

#include <QFutureWatcher>
#include <QMainWindow>

#include <cstddef>
#include <optional>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class PointCloudViewport;
class FloatingWorkflowMenu;
class PreprocessingSideMenu;
struct SurfaceConversionSettings;

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
    void handleRemoveInvalidPointsMenuOpened();
    void handleRemoveDuplicatesMenuOpened();
    void handlePickedPoint(std::size_t pointIndex);
    void handleAreaPointsSelected(const std::vector<std::size_t> &pointIndices);
    void handleAreaPointsDeselected(const std::vector<std::size_t> &pointIndices);
    void handleManualSelectionToggled(bool enabled);
    void handleAutoSelectSparsePoints(double radiusMax, int minimumNeighbourCount);
    void handleAutoSelectPerfectDuplicates();
    void handleAutoSelectNearDuplicates(double distanceThreshold);
    void handleAutoSelectFinished();
    void handleSurfaceReconstructionRequested(const SurfaceConversionSettings &settings);
    void handleSurfaceReconstructionFinished();
    void applySurfaceReconstruction();
    void closeSurfaceReconstruction();
    void cancelSurfaceReconstruction();
    void applyRemoveInvalidPoints();
    void confirmRemoveInvalidPoints();
    void cancelRemoveInvalidPoints();

private:
    enum class ProcessingState
    {
        Idle,
        AutoSelecting,
        Reconstructing,
    };

    enum class SelectionSessionMode
    {
        None,
        RemoveInvalidPoints,
        RemoveDuplicates,
    };

    void positionOverlayMenu();
    void loadDefaultPointCloud();
    [[nodiscard]] LoadPointCloudResult loadPointCloudFile(const std::filesystem::path &filePath);
    void clearSelectedPoints();
    void refreshViewportPointCloud(bool fitView = false);
    void toggleSelectedPoint(std::size_t pointIndex);
    void addSelectedPointIndices(const std::vector<std::size_t> &pointIndices);
    void removeSelectedPointIndices(const std::vector<std::size_t> &pointIndices);
    void beginSelectionSession(SelectionSessionMode mode);
    void endSelectionSession();
    [[nodiscard]] QString currentSelectionSessionTitle() const;
    void setManualPointSelectionActive(bool enabled);
    void setProcessingState(ProcessingState state);
    void startPreprocessing(const PreprocessingPipelineParameters &parameters);
    void queueProcessingProgress(std::size_t generation, const ProcessingProgress &progress);

    Ui::MainWindow *ui;
    ApplicationState applicationState_;
    PointCloudViewport *viewport_ = nullptr;
    FloatingWorkflowMenu *workflowMenu_ = nullptr;
    PreprocessingSideMenu *preprocessingSideMenu_ = nullptr;
    std::vector<std::size_t> selectedPointIndices_;
    bool manualPointSelectionEnabled_ = false;
    ProcessingState processingState_ = ProcessingState::Idle;
    std::size_t pendingAutoSelectRevision_ = 0U;
    QFutureWatcher<PreprocessingPipelineResult> autoSelectWatcher_;
    QFutureWatcher<ReconstructionPipelineResult> reconstructionWatcher_;
    std::size_t pendingReconstructionRevision_ = 0U;
    std::size_t processingGeneration_ = 0U;
    CancellationSource preprocessingCancellationSource_;
    CancellationSource reconstructionCancellationSource_;
    SelectionSessionMode selectionSessionMode_ = SelectionSessionMode::None;
    bool selectionSessionActive_ = false;
    std::optional<PointCloud> selectionSessionSnapshot_;
};
#endif // MAINWINDOW_H
