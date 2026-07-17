#include "view/ProcessingProgressWidget.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>

#include <algorithm>
#include <cmath>

namespace
{
constexpr auto kProgressStyle = R"(
QProgressBar {
    min-height: 6px;
    max-height: 6px;
    border: 1px solid #D6D4CE;
    border-radius: 3px;
    background: #EEEDE8;
}
QProgressBar::chunk {
    border-radius: 2px;
    background: #245CE6;
}
)";
}

ProcessingProgressWidget::ProcessingProgressWidget(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 2, 0, 2);
    layout->setSpacing(8);

    progressBar_ = new QProgressBar(this);
    progressBar_->setRange(0, 100);
    progressBar_->setTextVisible(false);
    progressBar_->setStyleSheet(QString::fromUtf8(kProgressStyle));
    layout->addWidget(progressBar_, 1);

    percentageLabel_ = new QLabel("0%", this);
    percentageLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    percentageLabel_->setFixedWidth(36);
    percentageLabel_->setStyleSheet("color: #4B4B47; font-size: 11px; font-weight: normal;");
    layout->addWidget(percentageLabel_);

    setVisible(false);
}

void ProcessingProgressWidget::begin()
{
    setPercentage(0);
    setVisible(true);
}

void ProcessingProgressWidget::setProgress(const ProcessingProgress &progress)
{
    const double fraction = std::isfinite(progress.fraction)
        ? std::clamp(progress.fraction, 0.0, 1.0)
        : 0.0;
    setPercentage(static_cast<int>(std::floor(fraction * 100.0 + 1.0e-9)));
    setVisible(true);
}

void ProcessingProgressWidget::complete()
{
    setPercentage(100);
    setVisible(true);
}

void ProcessingProgressWidget::clear()
{
    setPercentage(0);
    setVisible(false);
}

void ProcessingProgressWidget::setPercentage(int percentage)
{
    const int boundedPercentage = std::clamp(percentage, 0, 100);
    progressBar_->setValue(boundedPercentage);
    percentageLabel_->setText(QString::number(boundedPercentage) + "%");
}
