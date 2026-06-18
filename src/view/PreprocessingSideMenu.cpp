#include "view/PreprocessingSideMenu.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QStackedLayout>
#include <QVBoxLayout>

namespace
{
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
        padding: 5px 12px;
        font-weight: 600;
        text-align: left;
    }

    QPushButton[dialogAction="true"] {
        text-align: center;
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
    }

    QPushButton:pressed {
        background-color: rgba(60, 65, 70, 26);
    }
)";

constexpr int kMenuWidth = 220;
}

PreprocessingSideMenu::PreprocessingSideMenu(QWidget *parent)
    : QWidget(parent)
{
    setObjectName("PreprocessingSideMenu");
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(QString::fromUtf8(kContainerStyle) + QString::fromUtf8(kButtonStyle));
    setFixedWidth(kMenuWidth);

    stackedLayout_ = new QStackedLayout(this);
    stackedLayout_->setContentsMargins(0, 0, 0, 0);

    operationListPage_ = new QWidget(this);
    auto *operationLayout = new QVBoxLayout(operationListPage_);
    operationLayout->setContentsMargins(12, 10, 12, 10);
    operationLayout->setSpacing(6);

    for (const QString &label :
         {QString("Remove invalid points"),
          QString("Remove duplicates"),
          QString("Downsampling"),
          QString("Outlier removal")}) {
        QPushButton *button = createMenuButton(label, operationListPage_);
        operationLayout->addWidget(button);

        connect(button, &QPushButton::clicked, this, [this, label] {
            showParameterPlaceholder(label);
        });
    }

    parameterPlaceholderPage_ = new QWidget(this);
    auto *parameterLayout = new QVBoxLayout(parameterPlaceholderPage_);
    parameterLayout->setContentsMargins(12, 10, 12, 10);
    parameterLayout->setSpacing(6);

    for (const QString &label : {QString("test1"), QString("test2"), QString("test3")}) {
        parameterLayout->addWidget(createMenuButton(label, parameterPlaceholderPage_));
    }

    auto *actionLayout = new QHBoxLayout();
    actionLayout->setContentsMargins(0, 2, 0, 0);
    actionLayout->setSpacing(6);

    QPushButton *okButton = createMenuButton("OK", parameterPlaceholderPage_);
    QPushButton *cancelButton = createMenuButton("Cancel", parameterPlaceholderPage_);
    okButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    cancelButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    okButton->setProperty("dialogAction", true);
    cancelButton->setProperty("dialogAction", true);

    actionLayout->addWidget(okButton);
    actionLayout->addWidget(cancelButton);
    parameterLayout->addLayout(actionLayout);

    connect(okButton, &QPushButton::clicked, this, &PreprocessingSideMenu::showOperationList);
    connect(cancelButton, &QPushButton::clicked, this, &PreprocessingSideMenu::showOperationList);

    stackedLayout_->addWidget(operationListPage_);
    stackedLayout_->addWidget(parameterPlaceholderPage_);
    stackedLayout_->setCurrentWidget(operationListPage_);

    hide();
}

void PreprocessingSideMenu::showOperationList()
{
    stackedLayout_->setCurrentWidget(operationListPage_);
    show();
    raise();
}

void PreprocessingSideMenu::hideMenu()
{
    selectedOperation_.clear();
    stackedLayout_->setCurrentWidget(operationListPage_);
    hide();
}

void PreprocessingSideMenu::showParameterPlaceholder(const QString &operationName)
{
    selectedOperation_ = operationName;
    stackedLayout_->setCurrentWidget(parameterPlaceholderPage_);
    show();
    raise();
}

QPushButton *PreprocessingSideMenu::createMenuButton(const QString &label, QWidget *parent) const
{
    auto *button = new QPushButton(label, parent);
    button->setFlat(true);
    button->setCursor(Qt::PointingHandCursor);
    return button;
}
