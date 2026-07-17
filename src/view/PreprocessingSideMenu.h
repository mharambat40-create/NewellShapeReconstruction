#ifndef NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
#define NEWELL_VIEW_PREPROCESSINGSIDEMENU_H

#include "model/processing/common/ProcessingProgress.h"

#include <QWidget>

#include <optional>

class QEvent;
class QCheckBox;
class QComboBox;
class QFrame;
class QLabel;
class QStackedLayout;
class QTimer;
class QPushButton;
class QSpinBox;
#include <QPoint>

class FormattedDoubleSpinBox;
class ProcessingProgressWidget;

struct SurfaceConversionSettings
{
    double searchRadius = 0.0;
    int neighbourCount = 0;
    double scaleParameter = 0.0;
    QString surfaceMethod;
    QString normalMethod;
    bool preservePlanarHeight = true;
    double planarityTolerance = 0.02;
    double alphaValue = 1.0;
    bool automaticAlpha = true;
    double automaticAlphaFactor = 4.0;
    bool keepLargestComponentOnly = false;
    double minimumComponentArea = 0.0;
};

class PreprocessingSideMenu : public QWidget
{
    Q_OBJECT

public:
    explicit PreprocessingSideMenu(QWidget *parent = nullptr);
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

    void showOperationList();
    void hideMenu();
    void showRemoveInvalidPointsMenu();
    void showRemoveDuplicatesMenu();
    void showConvertToSurfaceMenu();
    void setManualSelectionEnabled(bool enabled);
    void setPointSelectionResultAvailable(bool available);
    void setAutoSelectInProgress(bool inProgress);
    [[nodiscard]] bool isAutoSelectInProgress() const;
    void beginProcessingProgress();
    void updateProcessingProgress(const ProcessingProgress &progress);
    void completeProcessingProgress();
    void clearProcessingProgress();
    void setSurfaceInputState(
        bool available,
        std::optional<double> planarityIndicator = std::nullopt,
        const QString &planarityError = {});

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
    void leaveEvent(QEvent *event) override;

signals:
    void removeInvalidPointsMenuOpened();
    void removeDuplicatesMenuOpened();
    void placeholderOperationRequested(const QString &operationName);
    void placeholderMessageRequested(const QString &title, const QString &message);
    void manualSelectionToggled(bool enabled);
    void autoSelectSparsePointsRequested(double radiusMax, int minimumNeighbourCount);
    void autoSelectPerfectDuplicatesRequested();
    void autoSelectNearDuplicatesRequested(double distanceThreshold);
    void surfaceReconstructionRequested(const SurfaceConversionSettings &settings);
    void applySurfaceReconstructionRequested();
    void closeSurfaceReconstructionRequested();
    void cancelSurfaceReconstructionRequested();
    void applyRemoveInvalidPointsRequested();
    void confirmRemoveInvalidPointsRequested();
    void cancelRemoveInvalidPointsRequested();

private:
    void showParameterPlaceholder(const QString &operationName);
    void openSurfaceMethodSelectionDialog();
    void openNormalMethodSelectionDialog();
    void updateNormalMethodCompatibility();
    void updateSurfaceMethodControls();
    void updateSurfaceConversionEligibility();
    void resetSurfaceConversionControls();
    QPushButton *createMenuButton(const QString &label, QWidget *parent) const;
    QFrame *createSeparator(QWidget *parent) const;
    void registerHelpTrigger(QWidget *trigger, QWidget *anchor, const QString &text);
    void scheduleContextHelpPopup(QWidget *anchor, const QString &text, const QPoint &globalPosition);
    void showContextHelpPopup(QWidget *anchor, const QString &text);
    void hideContextHelpPopup();
    [[nodiscard]] bool cursorInsideRegisteredHelpTrigger() const;
    [[nodiscard]] ProcessingProgressWidget *currentProgressWidget() const;

    QStackedLayout *stackedLayout_ = nullptr;
    QWidget *operationListPage_ = nullptr;
    QWidget *parameterPlaceholderPage_ = nullptr;
    QWidget *removeInvalidPointsPage_ = nullptr;
    QWidget *removeDuplicatesPage_ = nullptr;
    QWidget *convertToSurfacePage_ = nullptr;
    QWidget *surfaceConversionPanel_ = nullptr;
    QWidget *manualSelectionRowWidget_ = nullptr;
    QWidget *radiusRowWidget_ = nullptr;
    QWidget *neighboursRowWidget_ = nullptr;
    QWidget *duplicateManualSelectionRowWidget_ = nullptr;
    QWidget *perfectDuplicatesRowWidget_ = nullptr;
    QWidget *distanceThresholdRowWidget_ = nullptr;
    QWidget *normalEstimationRowWidget_ = nullptr;
    QWidget *surfaceSearchRadiusRowWidget_ = nullptr;
    QWidget *surfaceNeighboursRowWidget_ = nullptr;
    QWidget *ballRadiusRowWidget_ = nullptr;
    QWidget *surfaceMethodRowWidget_ = nullptr;
    QWidget *planarModeRowWidget_ = nullptr;
    QWidget *planarityToleranceRowWidget_ = nullptr;
    QWidget *alphaValueRowWidget_ = nullptr;
    QWidget *automaticAlphaRowWidget_ = nullptr;
    QWidget *automaticAlphaFactorRowWidget_ = nullptr;
    QWidget *largestComponentRowWidget_ = nullptr;
    QWidget *minimumComponentAreaRowWidget_ = nullptr;
    QWidget *surfaceConversionRowWidget_ = nullptr;
    QLabel *manualSelectionLabel_ = nullptr;
    QLabel *radiusLabel_ = nullptr;
    QLabel *neighboursLabel_ = nullptr;
    QLabel *duplicateManualSelectionLabel_ = nullptr;
    QLabel *perfectDuplicatesLabel_ = nullptr;
    QLabel *distanceThresholdLabel_ = nullptr;
    QLabel *normalEstimationLabel_ = nullptr;
    QLabel *surfaceSearchRadiusLabel_ = nullptr;
    QLabel *surfaceNeighboursLabel_ = nullptr;
    QLabel *ballRadiusLabel_ = nullptr;
    QLabel *surfaceMethodLabel_ = nullptr;
    QLabel *planarityStatusLabel_ = nullptr;
    QPushButton *manualSelectionButton_ = nullptr;
    QPushButton *duplicateManualSelectionButton_ = nullptr;
    FormattedDoubleSpinBox *radiusMaxSpinBox_ = nullptr;
    QSpinBox *minimumNeighboursSpinBox_ = nullptr;
    QPushButton *autoSelectButton_ = nullptr;
    QPushButton *perfectDuplicatesAutoSelectButton_ = nullptr;
    FormattedDoubleSpinBox *distanceThresholdSpinBox_ = nullptr;
    QPushButton *nearDuplicatesAutoSelectButton_ = nullptr;
    FormattedDoubleSpinBox *surfaceSearchRadiusSpinBox_ = nullptr;
    QSpinBox *surfaceNeighboursSpinBox_ = nullptr;
    FormattedDoubleSpinBox *ballRadiusSpinBox_ = nullptr;
    QPushButton *surfaceMethodSelectorButton_ = nullptr;
    QPushButton *normalMethodSelectorButton_ = nullptr;
    QComboBox *planarModeComboBox_ = nullptr;
    FormattedDoubleSpinBox *planarityToleranceSpinBox_ = nullptr;
    FormattedDoubleSpinBox *alphaValueSpinBox_ = nullptr;
    QCheckBox *automaticAlphaCheckBox_ = nullptr;
    FormattedDoubleSpinBox *automaticAlphaFactorSpinBox_ = nullptr;
    QCheckBox *largestComponentCheckBox_ = nullptr;
    FormattedDoubleSpinBox *minimumComponentAreaSpinBox_ = nullptr;
    QPushButton *surfaceConversionButton_ = nullptr;
    QPushButton *surfaceApplyButton_ = nullptr;
    QPushButton *surfaceOkButton_ = nullptr;
    QPushButton *surfaceCancelButton_ = nullptr;
    QPushButton *applyButton_ = nullptr;
    QPushButton *duplicateApplyButton_ = nullptr;
    QPushButton *okButton_ = nullptr;
    QPushButton *duplicateOkButton_ = nullptr;
    QPushButton *cancelButton_ = nullptr;
    QPushButton *duplicateCancelButton_ = nullptr;
    ProcessingProgressWidget *removeInvalidProgress_ = nullptr;
    ProcessingProgressWidget *downsamplingProgress_ = nullptr;
    ProcessingProgressWidget *surfaceProgress_ = nullptr;
    QLabel *contextHelpPopup_ = nullptr;
    QTimer *contextHelpTimer_ = nullptr;
    QWidget *pendingContextHelpAnchor_ = nullptr;
    QString selectedOperation_;
    QString selectedSurfaceMethod_;
    QString selectedNormalMethod_;
    QString pendingContextHelpText_;
    QPoint pendingContextHelpGlobalPosition_;
    bool autoSelectInProgress_ = false;
    bool pointSelectionResultAvailable_ = false;
    bool surfaceResultAvailable_ = false;
    bool surfaceInputAvailable_ = false;
    std::optional<double> surfacePlanarityIndicator_;
    QString surfacePlanarityError_;
};

#endif // NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
