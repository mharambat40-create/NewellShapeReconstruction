#include "view/FloatingWorkflowMenu.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QStyle>

namespace
{
constexpr auto kActiveInteractionColor = "#E7FFFF";

constexpr auto kContainerStyle = R"(
    FloatingWorkflowMenu {
        background-color: #F8F8F6;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 16px;
    }
)";

constexpr auto kButtonStyleTemplate = R"(
    QPushButton {
        color: #3C4146;
        background-color: transparent;
        border: none;
        border-radius: 8px;
        padding: 5px 12px;
        font-weight: 600;
        font-size: 13px;
    }

    QPushButton[active="true"] {
        background-color: %1;
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
    }

    QPushButton[active="true"]:hover {
        background-color: %1;
    }

    QPushButton:pressed {
        background-color: rgba(60, 65, 70, 26);
    }
)";
}

FloatingWorkflowMenu::FloatingWorkflowMenu(QWidget *parent)
    : QWidget(parent)
{
    setObjectName("FloatingWorkflowMenu");
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(
        QString::fromUtf8(kContainerStyle)
        + QString::fromUtf8(kButtonStyleTemplate).arg(QString::fromUtf8(kActiveInteractionColor)));

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 6, 12, 6);
    layout->setSpacing(6);

    importButton_ = new QPushButton("Import", this);
    preProcessingButton_ = new QPushButton("Pre-processing", this);
    surfaceSmoothingButton_ = new QPushButton("Surface Smoothing", this);
    parametricFittingButton_ = new QPushButton("Parametric Fitting", this);
    exportButton_ = new QPushButton("Export", this);

    for (QPushButton *button :
         {importButton_,
          preProcessingButton_,
          surfaceSmoothingButton_,
          parametricFittingButton_,
          exportButton_}) {
        button->setFlat(true);
        button->setCursor(Qt::PointingHandCursor);
    }

    layout->addWidget(importButton_);
    layout->addWidget(preProcessingButton_);
    layout->addWidget(surfaceSmoothingButton_);
    layout->addWidget(parametricFittingButton_);
    layout->addWidget(exportButton_);

    connect(importButton_, &QPushButton::clicked, this, [this] {
        emit workflowStepSelected("Import");
        emit importRequested();
    });
    connect(preProcessingButton_, &QPushButton::clicked, this, [this] {
        emit workflowStepSelected("Pre-processing");
    });
    connect(surfaceSmoothingButton_, &QPushButton::clicked, this, [this] {
        emit workflowStepSelected("Surface Smoothing");
    });
    connect(parametricFittingButton_, &QPushButton::clicked, this, [this] {
        emit workflowStepSelected("Parametric Fitting");
    });
    connect(exportButton_, &QPushButton::clicked, this, [this] {
        emit workflowStepSelected("Export");
    });

    updateButtonStates();
    adjustSize();
}

void FloatingWorkflowMenu::setActiveWorkflowStep(const QString &stepName)
{
    activeStepName_ = stepName;
    updateButtonStates();
}

void FloatingWorkflowMenu::setInteractionEnabled(bool enabled)
{
    for (QPushButton *button :
         {importButton_,
          preProcessingButton_,
          surfaceSmoothingButton_,
          parametricFittingButton_,
          exportButton_}) {
        button->setEnabled(enabled);
    }
}

void FloatingWorkflowMenu::updateButtonStates()
{
    for (QPushButton *button :
         {importButton_,
          preProcessingButton_,
          surfaceSmoothingButton_,
          parametricFittingButton_,
          exportButton_}) {
        button->setProperty("active", false);
        button->style()->unpolish(button);
        button->style()->polish(button);
    }

    if (QPushButton *activeButton = buttonForStep(activeStepName_)) {
        activeButton->setProperty("active", true);
        activeButton->style()->unpolish(activeButton);
        activeButton->style()->polish(activeButton);
    }
}

QPushButton *FloatingWorkflowMenu::buttonForStep(const QString &stepName) const
{
    if (stepName == "Import") {
        return importButton_;
    }
    if (stepName == "Pre-processing") {
        return preProcessingButton_;
    }
    if (stepName == "Surface Smoothing") {
        return surfaceSmoothingButton_;
    }
    if (stepName == "Parametric Fitting") {
        return parametricFittingButton_;
    }
    if (stepName == "Export") {
        return exportButton_;
    }

    return nullptr;
}
