#include "view/MethodSelectionDialog.h"

#include <QAbstractItemView>
#include <QColor>
#include <QFontMetrics>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QShowEvent>
#include <QTextBrowser>
#include <QVBoxLayout>

#include <algorithm>
#include <utility>

namespace
{
constexpr int kOptionHorizontalPadding = 8;
constexpr int kOptionRowHeight = 32;
constexpr int kExplanationWidth = 330;
constexpr int kDialogHeight = 480;

constexpr auto kDialogStyle = R"(
    QFrame#MethodSelectionContainer {
        background-color: #F8F8F6;
        border: 1px solid rgba(60, 65, 70, 32);
        border-radius: 16px;
    }

    QLabel#MethodSelectionTitle {
        color: #3C4146;
        font-size: 14px;
        font-weight: 600;
    }

    QListWidget {
        color: #202326;
        background-color: white;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 8px;
        outline: none;
        padding: 0;
    }

    QListWidget::item {
        color: #202326;
        background-color: transparent;
        border: none;
        border-radius: 6px;
        padding: 6px 8px;
        font-weight: 400;
    }

    QListWidget::item:hover {
        background-color: rgba(60, 65, 70, 12);
    }

    QListWidget::item:selected {
        color: #202326;
        background-color: #E7FFFF;
    }

    QListWidget::item:disabled {
        color: #9A9EA2;
        background-color: transparent;
    }

    QTextBrowser {
        color: #3C4146;
        background-color: white;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 8px;
        padding: 10px;
    }

    QFrame[dialogSeparator="true"] {
        background-color: rgba(60, 65, 70, 22);
        border: none;
        min-height: 1px;
        max-height: 1px;
    }

    QPushButton {
        color: #3C4146;
        background-color: transparent;
        border: none;
        border-radius: 8px;
        padding: 8px 16px;
        font-weight: 400;
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
    }

    QPushButton:pressed {
        background-color: rgba(60, 65, 70, 26);
    }

    QPushButton:disabled {
        color: #9A9EA2;
        background-color: transparent;
    }
)";

QString htmlList(const QStringList &items)
{
    QString html = "<ul>";
    for (const QString &item : items) {
        html += "<li>" + item.toHtmlEscaped() + "</li>";
    }
    return html + "</ul>";
}
}

MethodSelectionDialog::MethodSelectionDialog(
    const QString &title,
    QVector<MethodSelectionOption> options,
    const QString &currentValue,
    QWidget *centerTarget)
    : QDialog(nullptr)
    , options_(std::move(options))
    , centerTarget_(centerTarget)
{
    setWindowTitle(title);
    setWindowModality(Qt::ApplicationModal);
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground, true);

    auto *outerLayout = new QVBoxLayout(this);
    outerLayout->setContentsMargins(0, 0, 0, 0);

    auto *container = new QFrame(this);
    container->setObjectName("MethodSelectionContainer");
    container->setAttribute(Qt::WA_StyledBackground, true);
    container->setStyleSheet(QString::fromUtf8(kDialogStyle));
    outerLayout->addWidget(container);

    auto *containerLayout = new QVBoxLayout(container);
    containerLayout->setContentsMargins(14, 14, 14, 12);
    containerLayout->setSpacing(10);

    auto *titleLabel = new QLabel(title, container);
    titleLabel->setObjectName("MethodSelectionTitle");
    containerLayout->addWidget(titleLabel);

    auto *contentLayout = new QHBoxLayout();
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(10);

    optionList_ = new QListWidget(container);
    optionList_->setSelectionMode(QAbstractItemView::SingleSelection);
    optionList_->setFocusPolicy(Qt::NoFocus);
    optionList_->setMouseTracking(true);
    optionList_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    optionList_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    int longestTextWidth = 0;
    const QFontMetrics fontMetrics(optionList_->font());
    for (int index = 0; index < options_.size(); ++index) {
        const MethodSelectionOption &option = options_.at(index);
        auto *item = new QListWidgetItem(option.value, optionList_);
        item->setData(Qt::UserRole, index);
        item->setSizeHint(QSize(0, kOptionRowHeight));

        if (!option.enabled) {
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
            item->setForeground(QColor("#9A9EA2"));
            item->setToolTip(option.unavailableReason);
        }

        longestTextWidth = std::max(longestTextWidth, fontMetrics.horizontalAdvance(option.value));
        if (option.enabled && option.value == currentValue) {
            optionList_->setCurrentItem(item);
        }
    }

    const int optionListWidth = longestTextWidth + (2 * kOptionHorizontalPadding) + 4;
    optionList_->setFixedWidth(optionListWidth);
    temporarySelection_ = optionList_->currentItem();
    contentLayout->addWidget(optionList_);

    explanationPanel_ = new QTextBrowser(container);
    explanationPanel_->setFocusPolicy(Qt::NoFocus);
    explanationPanel_->setOpenExternalLinks(false);
    explanationPanel_->setMinimumWidth(kExplanationWidth);
    contentLayout->addWidget(explanationPanel_, 1);
    containerLayout->addLayout(contentLayout, 1);

    auto *separator = new QFrame(container);
    separator->setProperty("dialogSeparator", true);
    containerLayout->addWidget(separator);

    auto *actionLayout = new QHBoxLayout();
    actionLayout->setContentsMargins(0, 0, 0, 0);
    actionLayout->setSpacing(6);
    actionLayout->addStretch(1);

    okButton_ = new QPushButton("OK", container);
    auto *cancelButton = new QPushButton("Cancel", container);
    okButton_->setDefault(true);
    okButton_->setEnabled(temporarySelection_ != nullptr);
    actionLayout->addWidget(okButton_);
    actionLayout->addWidget(cancelButton);
    containerLayout->addLayout(actionLayout);

    connect(optionList_, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        if (!optionForItem(item)) {
            return;
        }

        temporarySelection_ = item;
        updateExplanation(item);
        okButton_->setEnabled(true);
    });
    connect(optionList_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem *item) {
        if (item && (item->flags() & Qt::ItemIsEnabled)) {
            optionList_->setCurrentItem(item);
            temporarySelection_ = item;
            updateExplanation(item);
            accept();
        }
    });
    connect(okButton_, &QPushButton::clicked, this, &MethodSelectionDialog::accept);
    connect(cancelButton, &QPushButton::clicked, this, &MethodSelectionDialog::reject);

    updateExplanation(temporarySelection_);
    resize(optionListWidth + kExplanationWidth + 62, kDialogHeight);
}

QString MethodSelectionDialog::selectedValue() const
{
    return selectedValue_;
}

void MethodSelectionDialog::accept()
{
    const MethodSelectionOption *option = optionForItem(temporarySelection_);
    if (!option) {
        return;
    }

    selectedValue_ = option->value;
    QDialog::accept();
}

void MethodSelectionDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);

    if (!centerTarget_) {
        return;
    }

    const QRect targetGeometry(centerTarget_->mapToGlobal(QPoint(0, 0)), centerTarget_->size());
    move(targetGeometry.center() - rect().center());
}

void MethodSelectionDialog::updateExplanation(const QListWidgetItem *item)
{
    const MethodSelectionOption *option = optionForItem(item);
    if (!option) {
        explanationPanel_->setHtml(
            "<p style='color:#6F7478;'>Select a method to view its technical summary.</p>");
        return;
    }

    const QString html = QString(
        "<style>"
        "body { color:#3C4146; font-size:12px; }"
        "h3 { color:#3C4146; font-size:12px; margin:10px 0 2px 0; }"
        "p { margin:0; }"
        "ul { margin:2px 0 0 18px; padding:0; }"
        "li { margin-bottom:2px; }"
        "</style>"
        "<h3>Description</h3><p>%1</p>"
        "<h3>Advantages</h3>%2"
        "<h3>Disadvantages</h3>%3"
        "<h3>Typical use cases</h3>%4")
                             .arg(
                                 option->description.toHtmlEscaped(),
                                 htmlList(option->advantages),
                                 htmlList(option->disadvantages),
                                 htmlList(option->useCases));
    explanationPanel_->setHtml(html);
}

const MethodSelectionOption *MethodSelectionDialog::optionForItem(const QListWidgetItem *item) const
{
    if (!item || !(item->flags() & Qt::ItemIsEnabled)) {
        return nullptr;
    }

    const int index = item->data(Qt::UserRole).toInt();
    if (index < 0 || index >= options_.size()) {
        return nullptr;
    }

    return &options_.at(index);
}
