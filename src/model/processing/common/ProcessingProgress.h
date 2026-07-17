#ifndef NEWELL_MODEL_PROCESSING_COMMON_PROCESSINGPROGRESS_H
#define NEWELL_MODEL_PROCESSING_COMMON_PROCESSINGPROGRESS_H

#include <functional>
#include <string>

enum class ProcessingStage
{
    Preparing,
    BuildingSpatialIndex,
    SelectingInvalidPoints,
    SelectingDuplicates,
    EstimatingNormals,
    BuildingNormalGraph,
    OrientingNormals,
    EvaluatingFieldGradients,
    FindingSeed,
    PropagatingSurface,
    Finalizing,
    Completed,
    Cancelled,
    Failed,
};

struct ProcessingProgress
{
    ProcessingStage stage = ProcessingStage::Preparing;
    double fraction = 0.0;
    std::string message;
};

using ProgressCallback = std::function<void(const ProcessingProgress &)>;

[[nodiscard]] double mapPhaseProgress(double phaseStart, double phaseEnd, double localFraction);

#endif // NEWELL_MODEL_PROCESSING_COMMON_PROCESSINGPROGRESS_H
