#ifndef NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
#define NEWELL_VIEW_PREPROCESSINGSIDEMENU_H

#include <QWidget>

class QDoubleSpinBox;
class QEvent;
class QFrame;
class QLabel;
class QProgressBar;
class QStackedLayout;
class QTimer;
class QPushButton;
class QSpinBox;
#include <QPoint>

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
    void setManualSelectionEnabled(bool enabled);
    void setAutoSelectInProgress(bool inProgress);
    [[nodiscard]] bool isAutoSelectInProgress() const;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
    void leaveEvent(QEvent *event) override;

signals:
    void removeInvalidPointsMenuOpened();
    void placeholderOperationRequested(const QString &operationName);
    void manualSelectionToggled(bool enabled);
    void autoSelectSparsePointsRequested(double radiusMax, int minimumNeighbourCount);
    void applyRemoveInvalidPointsRequested();
    void confirmRemoveInvalidPointsRequested();
    void cancelRemoveInvalidPointsRequested();

private:
    void showParameterPlaceholder(const QString &operationName);
    QPushButton *createMenuButton(const QString &label, QWidget *parent) const;
    QFrame *createSeparator(QWidget *parent) const;
    void registerHelpTrigger(QWidget *trigger, QWidget *anchor, const QString &text);
    void scheduleContextHelpPopup(QWidget *anchor, const QString &text, const QPoint &globalPosition);
    void showContextHelpPopup(QWidget *anchor, const QString &text);
    void hideContextHelpPopup();
    [[nodiscard]] bool cursorInsideRegisteredHelpTrigger() const;

    QStackedLayout *stackedLayout_ = nullptr;
    QWidget *operationListPage_ = nullptr;
    QWidget *parameterPlaceholderPage_ = nullptr;
    QWidget *removeInvalidPointsPage_ = nullptr;
    QWidget *manualSelectionRowWidget_ = nullptr;
    QWidget *radiusRowWidget_ = nullptr;
    QWidget *neighboursRowWidget_ = nullptr;
    QLabel *manualSelectionLabel_ = nullptr;
    QLabel *radiusLabel_ = nullptr;
    QLabel *neighboursLabel_ = nullptr;
    QPushButton *manualSelectionButton_ = nullptr;
    QDoubleSpinBox *radiusMaxSpinBox_ = nullptr;
    QSpinBox *minimumNeighboursSpinBox_ = nullptr;
    QPushButton *autoSelectButton_ = nullptr;
    QPushButton *applyButton_ = nullptr;
    QPushButton *okButton_ = nullptr;
    QPushButton *cancelButton_ = nullptr;
    QLabel *loadingLabel_ = nullptr;
    QProgressBar *loadingProgressBar_ = nullptr;
    QLabel *contextHelpPopup_ = nullptr;
    QTimer *contextHelpTimer_ = nullptr;
    QWidget *pendingContextHelpAnchor_ = nullptr;
    QString selectedOperation_;
    QString pendingContextHelpText_;
    QPoint pendingContextHelpGlobalPosition_;
    bool autoSelectInProgress_ = false;
};

#endif // NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
