#include "view/PreprocessingSideMenu.h"

#include <QAbstractSpinBox>
#include <QCursor>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPoint>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedLayout>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
constexpr int kAlignedControlWidth = 84;
constexpr int kMenuWidth = 253;
constexpr int kContextHelpPopupWidth = 240;
constexpr int kContextHelpPopupOffset = 12;
constexpr int kContextHelpDelayMs = 1000;
constexpr int kContextHelpStillThresholdPixels = 4;
constexpr auto kManualSelectionHelpText =
    "Left-drag to select points with a box. Command-left-drag to deselect points with a box. "
    "Click a point to toggle it.";
constexpr auto kRadiusHelpText =
    "Maximum search radius used to count neighbouring points.";
constexpr auto kNeighbourHelpText =
    "Minimum number of neighbours required inside the radius. Points below this count are selected.";

constexpr auto kContainerStyle = R"(
    PreprocessingSideMenu {
        background-color: #F8F8F6;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 16px;
    }
)";

constexpr auto kButtonStyle = R"(
    QPushButton {
        color: #3C4146;
        background-color: transparent;
        border: none;
        border-radius: 8px;
        padding: 8px;
        font-weight: 600;
        text-align: left;
    }

    QPushButton[dialogAction="true"] {
        text-align: center;
    }

    QPushButton[actionButton="true"] {
        font-weight: 400;
    }

    QPushButton[fieldLike="true"] {
        background-color: white;
        border: 1px solid rgba(60, 65, 70, 22);
        text-align: center;
        font-weight: 400;
    }

    QPushButton[selectionToggle="true"]:checked {
        background-color: #E7FFFF;
        border: 1px solid rgba(60, 65, 70, 22);
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
    }

    QPushButton[selectionToggle="true"]:checked:hover {
        background-color: #E7FFFF;
    }

    QPushButton:pressed {
        background-color: rgba(60, 65, 70, 26);
    }

    QLabel {
        color: #3C4146;
        font-weight: 600;
    }

    QFrame[menuSeparator="true"] {
        background-color: rgba(60, 65, 70, 22);
        min-height: 1px;
        max-height: 1px;
        border: none;
    }

    QDoubleSpinBox,
    QSpinBox {
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 8px;
        background-color: white;
        color: #3C4146;
        padding: 4px 8px;
    }
)";

constexpr auto kContextHelpPopupStyle = R"(
    QLabel {
        background-color: #F8F8F6;
        color: #3C4146;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 12px;
        padding: 8px 10px;
        font-size: 11px;
        font-weight: 400;
    }
)";

}

PreprocessingSideMenu::PreprocessingSideMenu(QWidget *parent)
    : QWidget(parent)
{
    setObjectName("PreprocessingSideMenu");
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(QString::fromUtf8(kContainerStyle) + QString::fromUtf8(kButtonStyle));
    setFixedWidth(kMenuWidth);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);

    stackedLayout_ = new QStackedLayout(this);
    stackedLayout_->setContentsMargins(0, 0, 0, 0);

    operationListPage_ = new QWidget(this);
    auto *operationLayout = new QVBoxLayout(operationListPage_);
    operationLayout->setContentsMargins(10, 10, 10, 10);
    operationLayout->setSpacing(4);

    const QString removeInvalidPointsLabel("Remove invalid points");
    QPushButton *removeInvalidPointsButton =
        createMenuButton(removeInvalidPointsLabel, operationListPage_);
    operationLayout->addWidget(removeInvalidPointsButton);
    connect(removeInvalidPointsButton, &QPushButton::clicked, this, [this] {
        showRemoveInvalidPointsMenu();
    });

    for (const QString &label :
         {QString("Remove duplicates"), QString("Downsampling"), QString("Outlier removal")}) {
        QPushButton *button = createMenuButton(label, operationListPage_);
        operationLayout->addWidget(button);

        connect(button, &QPushButton::clicked, this, [this, label] {
            showParameterPlaceholder(label);
        });
    }

    removeInvalidPointsPage_ = new QWidget(this);
    auto *removeInvalidLayout = new QVBoxLayout(removeInvalidPointsPage_);
    removeInvalidLayout->setContentsMargins(10, 10, 10, 10);
    removeInvalidLayout->setSpacing(4);

    manualSelectionRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *manualSelectionRow = new QHBoxLayout(manualSelectionRowWidget_);
    manualSelectionRow->setContentsMargins(0, 0, 0, 0);
    manualSelectionRow->setSpacing(8);
    manualSelectionLabel_ = new QLabel("Manual selection", manualSelectionRowWidget_);
    manualSelectionRow->addWidget(manualSelectionLabel_);
    manualSelectionRow->addStretch(1);

    manualSelectionButton_ = createMenuButton("Select", manualSelectionRowWidget_);
    manualSelectionButton_->setCheckable(true);
    manualSelectionButton_->setFixedWidth(kAlignedControlWidth);
    manualSelectionButton_->setProperty("selectionToggle", true);
    manualSelectionButton_->setProperty("dialogAction", true);
    manualSelectionButton_->setProperty("fieldLike", true);
    manualSelectionRow->addWidget(manualSelectionButton_);
    removeInvalidLayout->addWidget(manualSelectionRowWidget_);
    connect(manualSelectionButton_, &QPushButton::toggled, this, [this](bool enabled) {
        emit manualSelectionToggled(enabled);
    });
    registerHelpTrigger(manualSelectionRowWidget_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(manualSelectionLabel_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(manualSelectionButton_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    radiusRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *radiusRow = new QHBoxLayout(radiusRowWidget_);
    radiusRow->setContentsMargins(0, 0, 0, 0);
    radiusRow->setSpacing(8);
    radiusLabel_ = new QLabel("Radius max", radiusRowWidget_);
    radiusRow->addWidget(radiusLabel_);
    radiusRow->addStretch(1);

    radiusMaxSpinBox_ = new QDoubleSpinBox(radiusRowWidget_);
    radiusMaxSpinBox_->setDecimals(3);
    radiusMaxSpinBox_->setRange(0.0, 1.0e9);
    radiusMaxSpinBox_->setValue(1.0);
    radiusMaxSpinBox_->setSingleStep(0.1);
    radiusMaxSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    radiusMaxSpinBox_->setAlignment(Qt::AlignRight);
    radiusMaxSpinBox_->setFixedWidth(kAlignedControlWidth);
    radiusRow->addWidget(radiusMaxSpinBox_);
    removeInvalidLayout->addWidget(radiusRowWidget_);
    registerHelpTrigger(radiusRowWidget_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
    registerHelpTrigger(radiusLabel_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
    registerHelpTrigger(radiusMaxSpinBox_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));

    neighboursRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *neighboursRow = new QHBoxLayout(neighboursRowWidget_);
    neighboursRow->setContentsMargins(0, 0, 0, 0);
    neighboursRow->setSpacing(8);
    neighboursLabel_ = new QLabel("Number of neighbours", neighboursRowWidget_);
    neighboursRow->addWidget(neighboursLabel_);
    neighboursRow->addStretch(1);

    minimumNeighboursSpinBox_ = new QSpinBox(neighboursRowWidget_);
    minimumNeighboursSpinBox_->setRange(0, 1000000);
    minimumNeighboursSpinBox_->setValue(4);
    minimumNeighboursSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    minimumNeighboursSpinBox_->setAlignment(Qt::AlignRight);
    minimumNeighboursSpinBox_->setFixedWidth(kAlignedControlWidth);
    neighboursRow->addWidget(minimumNeighboursSpinBox_);
    removeInvalidLayout->addWidget(neighboursRowWidget_);
    registerHelpTrigger(neighboursRowWidget_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
    registerHelpTrigger(neighboursLabel_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
    registerHelpTrigger(minimumNeighboursSpinBox_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));

    autoSelectButton_ = createMenuButton("Auto select", removeInvalidPointsPage_);
    autoSelectButton_->setProperty("dialogAction", true);
    autoSelectButton_->setProperty("actionButton", true);
    removeInvalidLayout->addWidget(autoSelectButton_);
    connect(autoSelectButton_, &QPushButton::clicked, this, [this] {
        emit autoSelectSparsePointsRequested(
            radiusMaxSpinBox_->value(),
            minimumNeighboursSpinBox_->value());
    });

    loadingLabel_ = new QLabel("Auto selecting points...", removeInvalidPointsPage_);
    loadingLabel_->setVisible(false);
    removeInvalidLayout->addWidget(loadingLabel_);

    loadingProgressBar_ = new QProgressBar(removeInvalidPointsPage_);
    loadingProgressBar_->setRange(0, 0);
    loadingProgressBar_->setTextVisible(false);
    loadingProgressBar_->setVisible(false);
    removeInvalidLayout->addWidget(loadingProgressBar_);

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    applyButton_ = createMenuButton("Apply Removal", removeInvalidPointsPage_);
    applyButton_->setProperty("dialogAction", true);
    applyButton_->setProperty("actionButton", true);
    removeInvalidLayout->addWidget(applyButton_);
    connect(applyButton_, &QPushButton::clicked, this, [this] {
        emit applyRemoveInvalidPointsRequested();
    });

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    auto *removeActionLayout = new QHBoxLayout();
    removeActionLayout->setContentsMargins(0, 0, 0, 0);
    removeActionLayout->setSpacing(6);

    okButton_ = createMenuButton("OK", removeInvalidPointsPage_);
    cancelButton_ = createMenuButton("Cancel", removeInvalidPointsPage_);
    okButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    cancelButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    okButton_->setProperty("dialogAction", true);
    cancelButton_->setProperty("dialogAction", true);
    okButton_->setProperty("actionButton", true);
    cancelButton_->setProperty("actionButton", true);
    removeActionLayout->addWidget(okButton_);
    removeActionLayout->addWidget(cancelButton_);
    removeInvalidLayout->addLayout(removeActionLayout);

    connect(okButton_, &QPushButton::clicked, this, [this] {
        emit confirmRemoveInvalidPointsRequested();
    });
    connect(cancelButton_, &QPushButton::clicked, this, [this] {
        emit cancelRemoveInvalidPointsRequested();
    });

    parameterPlaceholderPage_ = new QWidget(this);
    auto *parameterLayout = new QVBoxLayout(parameterPlaceholderPage_);
    parameterLayout->setContentsMargins(10, 10, 10, 10);
    parameterLayout->setSpacing(4);

    for (const QString &label : {QString("test1"), QString("test2"), QString("test3")}) {
        parameterLayout->addWidget(createMenuButton(label, parameterPlaceholderPage_));
    }

    auto *actionLayout = new QHBoxLayout();
    actionLayout->setContentsMargins(0, 2, 0, 0);
    actionLayout->setSpacing(6);

    QPushButton *placeholderOkButton = createMenuButton("OK", parameterPlaceholderPage_);
    QPushButton *placeholderCancelButton = createMenuButton("Cancel", parameterPlaceholderPage_);
    placeholderOkButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    placeholderCancelButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    placeholderOkButton->setProperty("dialogAction", true);
    placeholderCancelButton->setProperty("dialogAction", true);

    actionLayout->addWidget(placeholderOkButton);
    actionLayout->addWidget(placeholderCancelButton);
    parameterLayout->addLayout(actionLayout);

    connect(
        placeholderOkButton,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::showOperationList);
    connect(
        placeholderCancelButton,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::showOperationList);

    stackedLayout_->addWidget(operationListPage_);
    stackedLayout_->addWidget(removeInvalidPointsPage_);
    stackedLayout_->addWidget(parameterPlaceholderPage_);
    stackedLayout_->setCurrentWidget(operationListPage_);

    contextHelpPopup_ = new QLabel(parentWidget() ? parentWidget() : this);
    contextHelpPopup_->setObjectName("ContextHelpPopup");
    contextHelpPopup_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    contextHelpPopup_->setWordWrap(true);
    contextHelpPopup_->setFixedWidth(kContextHelpPopupWidth);
    contextHelpPopup_->setStyleSheet(QString::fromUtf8(kContextHelpPopupStyle));
    contextHelpPopup_->hide();

    contextHelpTimer_ = new QTimer(this);
    contextHelpTimer_->setSingleShot(true);
    connect(contextHelpTimer_, &QTimer::timeout, this, [this] {
        if (!pendingContextHelpAnchor_) {
            return;
        }

        showContextHelpPopup(pendingContextHelpAnchor_, pendingContextHelpText_);
    });

    hide();
}

QSize PreprocessingSideMenu::sizeHint() const
{
    if (!stackedLayout_ || stackedLayout_->count() == 0) {
        return QWidget::sizeHint();
    }

    if (QWidget *currentPage = stackedLayout_->currentWidget()) {
        return QSize(kMenuWidth, currentPage->sizeHint().height());
    }

    return QWidget::sizeHint();
}

QSize PreprocessingSideMenu::minimumSizeHint() const
{
    return sizeHint();
}

void PreprocessingSideMenu::showOperationList()
{
    if (autoSelectInProgress_) {
        return;
    }

    setManualSelectionEnabled(false);
    selectedOperation_.clear();
    stackedLayout_->setCurrentWidget(operationListPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
}

void PreprocessingSideMenu::hideMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    setManualSelectionEnabled(false);
    selectedOperation_.clear();
    stackedLayout_->setCurrentWidget(operationListPage_);
    updateGeometry();
    adjustSize();
    hide();
}

void PreprocessingSideMenu::showRemoveInvalidPointsMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = "Remove invalid points";
    setManualSelectionEnabled(false);
    stackedLayout_->setCurrentWidget(removeInvalidPointsPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
    emit removeInvalidPointsMenuOpened();
}

void PreprocessingSideMenu::setManualSelectionEnabled(bool enabled)
{
    if (!manualSelectionButton_) {
        return;
    }

    if (autoSelectInProgress_) {
        enabled = false;
    }

    manualSelectionButton_->blockSignals(true);
    manualSelectionButton_->setChecked(enabled);
    manualSelectionButton_->blockSignals(false);
    manualSelectionButton_->style()->unpolish(manualSelectionButton_);
    manualSelectionButton_->style()->polish(manualSelectionButton_);
}

void PreprocessingSideMenu::setAutoSelectInProgress(bool inProgress)
{
    autoSelectInProgress_ = inProgress;

    manualSelectionButton_->setEnabled(!inProgress);
    radiusMaxSpinBox_->setEnabled(!inProgress);
    minimumNeighboursSpinBox_->setEnabled(!inProgress);
    autoSelectButton_->setEnabled(!inProgress);
    applyButton_->setEnabled(!inProgress);
    okButton_->setEnabled(!inProgress);
    cancelButton_->setEnabled(!inProgress);
    loadingLabel_->setVisible(inProgress);
    loadingProgressBar_->setVisible(inProgress);

    if (inProgress) {
        setManualSelectionEnabled(false);
    }

    updateGeometry();
    adjustSize();
}

bool PreprocessingSideMenu::isAutoSelectInProgress() const
{
    return autoSelectInProgress_;
}

bool PreprocessingSideMenu::eventFilter(QObject *watched, QEvent *event)
{
    auto scheduleHelp = [this](QWidget *anchor, const QString &text) {
        scheduleContextHelpPopup(anchor, text, QCursor::pos());
    };

    auto maybeResetHelpTimer = [this](QWidget *anchor, const QString &text, QEvent *currentEvent) {
        if (!contextHelpTimer_ || !contextHelpTimer_->isActive() ||
            currentEvent->type() != QEvent::MouseMove) {
            return;
        }

        const auto *mouseEvent = static_cast<QMouseEvent *>(currentEvent);
        const QPoint currentGlobalPosition = mouseEvent->globalPosition().toPoint();
        if ((currentGlobalPosition - pendingContextHelpGlobalPosition_).manhattanLength() >
            kContextHelpStillThresholdPixels) {
            scheduleContextHelpPopup(anchor, text, currentGlobalPosition);
        }
    };

    if (watched == manualSelectionRowWidget_ || watched == manualSelectionLabel_ || watched == manualSelectionButton_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (watched == radiusRowWidget_ || watched == radiusLabel_ || watched == radiusMaxSpinBox_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (
        watched == neighboursRowWidget_ || watched == neighboursLabel_ || watched == minimumNeighboursSpinBox_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    }

    return QWidget::eventFilter(watched, event);
}

void PreprocessingSideMenu::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);

    if (!cursorInsideRegisteredHelpTrigger()) {
        hideContextHelpPopup();
    }
}

void PreprocessingSideMenu::showParameterPlaceholder(const QString &operationName)
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = operationName;
    stackedLayout_->setCurrentWidget(parameterPlaceholderPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
    emit placeholderOperationRequested(operationName);
}

QPushButton *PreprocessingSideMenu::createMenuButton(const QString &label, QWidget *parent) const
{
    auto *button = new QPushButton(label, parent);
    button->setFlat(true);
    button->setCursor(Qt::PointingHandCursor);
    return button;
}

QFrame *PreprocessingSideMenu::createSeparator(QWidget *parent) const
{
    auto *separator = new QFrame(parent);
    separator->setProperty("menuSeparator", true);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Plain);
    return separator;
}

void PreprocessingSideMenu::registerHelpTrigger(QWidget *trigger, QWidget *anchor, const QString &text)
{
    if (!trigger || !anchor) {
        return;
    }

    trigger->setToolTip(QString());
    trigger->setMouseTracking(true);
    trigger->installEventFilter(this);
}

void PreprocessingSideMenu::scheduleContextHelpPopup(
    QWidget *anchor,
    const QString &text,
    const QPoint &globalPosition)
{
    if (!contextHelpTimer_ || !anchor) {
        return;
    }

    pendingContextHelpAnchor_ = anchor;
    pendingContextHelpText_ = text;
    pendingContextHelpGlobalPosition_ = globalPosition;
    contextHelpTimer_->start(kContextHelpDelayMs);
}

void PreprocessingSideMenu::showContextHelpPopup(QWidget *anchor, const QString &text)
{
    if (!contextHelpPopup_ || !anchor) {
        return;
    }

    QWidget *popupParent = contextHelpPopup_->parentWidget();
    if (!popupParent) {
        popupParent = this;
    }

    contextHelpPopup_->setText(text);
    contextHelpPopup_->adjustSize();

    const QPoint anchorTopLeft = anchor->mapTo(popupParent, QPoint(0, 0));
    const int popupX = x() - contextHelpPopup_->width() - kContextHelpPopupOffset;
    const int popupY =
        anchorTopLeft.y() + (anchor->height() - contextHelpPopup_->height()) / 2;

    contextHelpPopup_->move(std::max(0, popupX), std::max(0, popupY));
    contextHelpPopup_->raise();
    contextHelpPopup_->show();
}

void PreprocessingSideMenu::hideContextHelpPopup()
{
    if (contextHelpTimer_) {
        contextHelpTimer_->stop();
    }

    pendingContextHelpAnchor_ = nullptr;
    pendingContextHelpText_.clear();

    if (contextHelpPopup_) {
        contextHelpPopup_->hide();
    }
}

bool PreprocessingSideMenu::cursorInsideRegisteredHelpTrigger() const
{
    const QPoint globalCursorPosition = QCursor::pos();
    const QWidget *widgets[] = {
        manualSelectionRowWidget_,
        manualSelectionLabel_,
        manualSelectionButton_,
        radiusRowWidget_,
        radiusLabel_,
        radiusMaxSpinBox_,
        neighboursRowWidget_,
        neighboursLabel_,
        minimumNeighboursSpinBox_,
    };

    for (const QWidget *widget : widgets) {
        if (!widget || !widget->isVisible()) {
            continue;
        }

        const QRect globalGeometry(
            widget->mapToGlobal(QPoint(0, 0)),
            widget->size());
        if (globalGeometry.contains(globalCursorPosition)) {
            return true;
        }
    }

    return false;
}
