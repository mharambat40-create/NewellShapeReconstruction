#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "controller/ApplicationState.h"

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
    void handlePickedPoint(std::size_t pointIndex);
    void handleAreaPointsSelected(const std::vector<std::size_t> &pointIndices);
    void handleAreaPointsDeselected(const std::vector<std::size_t> &pointIndices);
    void handleManualSelectionToggled(bool enabled);
    void handleAutoSelectSparsePoints(double radiusMax, int minimumNeighbourCount);
    void handleAutoSelectFinished();
    void applyRemoveInvalidPoints();
    void confirmRemoveInvalidPoints();
    void cancelRemoveInvalidPoints();

private:
    enum class ProcessingState
    {
        Idle,
        AutoSelecting,
    };

    void positionOverlayMenu();
    void clearSelectedPoints();
    void refreshViewportPointCloud(bool fitView = false);
    void toggleSelectedPoint(std::size_t pointIndex);
    void addSelectedPointIndices(const std::vector<std::size_t> &pointIndices);
    void removeSelectedPointIndices(const std::vector<std::size_t> &pointIndices);
    void beginRemoveInvalidPointsSession();
    void endRemoveInvalidPointsSession();
    void setManualPointSelectionActive(bool enabled);
    void setProcessingState(ProcessingState state);

    Ui::MainWindow *ui;
    ApplicationState applicationState_;
    PointCloudViewport *viewport_ = nullptr;
    FloatingWorkflowMenu *workflowMenu_ = nullptr;
    PreprocessingSideMenu *preprocessingSideMenu_ = nullptr;
    std::vector<std::size_t> selectedPointIndices_;
    bool manualPointSelectionEnabled_ = false;
    ProcessingState processingState_ = ProcessingState::Idle;
    std::size_t pendingAutoSelectRevision_ = 0U;
    QFutureWatcher<SparseSelectionResult> autoSelectWatcher_;
    bool removeInvalidPointsSessionActive_ = false;
    std::optional<PointCloud> removeInvalidPointsSnapshot_;
    std::vector<std::size_t> removeInvalidPointsSnapshotSelection_;
};
#endif // MAINWINDOW_H
