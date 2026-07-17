#ifndef NEWELL_MODEL_PIPELINE_RECONSTRUCTIONPIPELINE_H
#define NEWELL_MODEL_PIPELINE_RECONSTRUCTIONPIPELINE_H

#include "model/geometry/NormalField.h"
#include "model/geometry/TriangleMesh.h"
#include "model/processing/normals/INormalEstimator.h"
#include "model/processing/reconstruction/ISurfaceReconstructor.h"
#include "model/processing/reconstruction/ReconstructionTypes.h"

#include <cstddef>
#include <string>

class PointCloud;

struct ReconstructionProgressWeights
{
    static constexpr double preparation = 0.02;
    static constexpr double spatialIndex = 0.05;
    static constexpr double normalEstimation = 0.18;
    static constexpr double normalFinalization = 0.05;
    static constexpr double ballPivoting = 0.68;
    static constexpr double resultConstruction = 0.01;
    static constexpr double publication = 0.01;
};

struct ReconstructionPhaseTimings
{
    double preparationMilliseconds = 0.0;
    double spatialIndexMilliseconds = 0.0;
    double normalEstimationMilliseconds = 0.0;
    double normalOrientationMilliseconds = 0.0;
    double seedSearchMilliseconds = 0.0;
    double frontPropagationMilliseconds = 0.0;
    double meshValidationMilliseconds = 0.0;
    double resultConstructionMilliseconds = 0.0;
    double resultTransferMilliseconds = 0.0;
};

struct ReconstructionPipelineParameters
{
    ReconstructionMethod method = ReconstructionMethod::BallPivoting;
    NormalEstimationParameters normalEstimation;
    NormalProcessingConfiguration normalProcessing;
    ReconstructionMethodParameters methodParameters = BallPivotingParameters{};
    // Retained for source compatibility with the original BPA-only slice.
    ReconstructionParameters reconstruction;
};

struct ReconstructionPipelineResult
{
    bool succeeded = false;
    bool cancelled = false;
    NormalField normalField;
    TriangleMesh mesh;
    MeshDiagnostics diagnostics;
    NormalDiagnostics normalDiagnostics;
    std::size_t invalidNormalCount = 0U;
    ReconstructionPhaseTimings timings;
    std::string errorMessage;
};

class ReconstructionPipeline
{
public:
    [[nodiscard]] ReconstructionPipelineResult execute(
        const PointCloud &pointCloud,
        const ReconstructionPipelineParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

#endif // NEWELL_MODEL_PIPELINE_RECONSTRUCTIONPIPELINE_H
