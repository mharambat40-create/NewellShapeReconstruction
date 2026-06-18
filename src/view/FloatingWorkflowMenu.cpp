#include "view/FloatingWorkflowMenu.h"

#include <QHBoxLayout>
#include <QPushButton>

namespace
{
constexpr auto kContainerStyle = R"(
    FloatingWorkflowMenu {
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
        padding: 5px 12px;
        font-weight: 600;
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
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
    setStyleSheet(QString::fromUtf8(kContainerStyle) + QString::fromUtf8(kButtonStyle));

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

    connect(importButton_, &QPushButton::clicked, this, &FloatingWorkflowMenu::importRequested);
    connect(preProcessingButton_, &QPushButton::clicked, this, [this] {
        emit placeholderRequested("Pre-processing");
    });
    connect(surfaceSmoothingButton_, &QPushButton::clicked, this, [this] {
        emit placeholderRequested("Surface Smoothing");
    });
    connect(parametricFittingButton_, &QPushButton::clicked, this, [this] {
        emit placeholderRequested("Parametric Fitting");
    });
    connect(exportButton_, &QPushButton::clicked, this, [this] {
        emit placeholderRequested("Export");
    });

    adjustSize();
}
