#ifndef NEWELL_MODEL_PROCESSING_COMMON_PROGRESSREPORTER_H
#define NEWELL_MODEL_PROCESSING_COMMON_PROGRESSREPORTER_H

#include "model/processing/common/ProcessingProgress.h"

#include <chrono>

class ProgressReporter
{
public:
    explicit ProgressReporter(ProgressCallback callback = {});

    void report(ProcessingStage stage, double fraction, std::string message = {});
    void complete(std::string message = {});
    void cancelled(std::string message = {});
    void failed(std::string message = {});

    [[nodiscard]] double lastFraction() const;

private:
    void emit(ProcessingStage stage, double fraction, std::string message, bool terminal);

    ProgressCallback callback_;
    double lastFraction_ = 0.0;
    int lastPercentage_ = -1;
    std::chrono::steady_clock::time_point lastEmissionTime_{};
};

#endif // NEWELL_MODEL_PROCESSING_COMMON_PROGRESSREPORTER_H
