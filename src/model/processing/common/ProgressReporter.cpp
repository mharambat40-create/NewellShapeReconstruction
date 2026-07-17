#include "model/processing/common/ProgressReporter.h"

#include <algorithm>
#include <cmath>
#include <utility>

ProgressReporter::ProgressReporter(ProgressCallback callback)
    : callback_(std::move(callback))
{
}

void ProgressReporter::report(ProcessingStage stage, double fraction, std::string message)
{
    emit(stage, fraction, std::move(message), false);
}

void ProgressReporter::complete(std::string message)
{
    emit(ProcessingStage::Completed, 1.0, std::move(message), true);
}

void ProgressReporter::cancelled(std::string message)
{
    emit(ProcessingStage::Cancelled, lastFraction_, std::move(message), true);
}

void ProgressReporter::failed(std::string message)
{
    emit(ProcessingStage::Failed, lastFraction_, std::move(message), true);
}

double ProgressReporter::lastFraction() const
{
    return lastFraction_;
}

void ProgressReporter::emit(
    ProcessingStage stage,
    double fraction,
    std::string message,
    bool terminal)
{
    if (lastFraction_ >= 1.0 && stage != ProcessingStage::Completed) {
        return;
    }
    double safeFraction = std::isfinite(fraction) ? fraction : lastFraction_;
    safeFraction = std::clamp(safeFraction, lastFraction_, terminal ? 1.0 : 0.99);
    if (stage != ProcessingStage::Completed) {
        safeFraction = std::min(safeFraction, 0.99);
    }

    const int percentage = static_cast<int>(std::floor(safeFraction * 100.0 + 1.0e-9));
    const auto now = std::chrono::steady_clock::now();
    const bool throttleElapsed = lastEmissionTime_ == std::chrono::steady_clock::time_point{} ||
        now - lastEmissionTime_ >= std::chrono::milliseconds(50);
    const bool shouldEmit = terminal || (percentage != lastPercentage_ && throttleElapsed);
    lastFraction_ = safeFraction;
    if (!callback_ || !shouldEmit) {
        return;
    }

    lastPercentage_ = percentage;
    lastEmissionTime_ = now;
    callback_(ProcessingProgress{stage, safeFraction, std::move(message)});
}
