#include "model/processing/normals/ImplicitFieldGradientNormalGenerator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

NormalMethod ImplicitFieldGradientNormalGenerator::method() const
{
    return NormalMethod::ImplicitFieldGradient;
}

NormalEstimationResult ImplicitFieldGradientNormalGenerator::generate(
    const FieldNormalInput &input,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<ImplicitGradientParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Implicit gradients received parameters for another normal method.";
        return result;
    }
    if (!input.queryPoints || !input.implicitField || input.queryPoints->empty()) {
        result.errorMessage = "Implicit-field gradients require query points and an implicit scalar field.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(*input.queryPoints)) {
        result.errorMessage = "Implicit-field gradients require finite query coordinates.";
        return result;
    }
    if (!std::isfinite(methodParameters->finiteDifferenceStep) ||
        methodParameters->finiteDifferenceStep <= 0.0 ||
        !std::isfinite(methodParameters->minimumGradientMagnitude) ||
        methodParameters->minimumGradientMagnitude <= 0.0) {
        result.errorMessage = "Implicit-gradient parameters must be positive and finite.";
        return result;
    }

    result.normalField.reserve(input.queryPoints->pointCount());
    result.invalidPointIndices.reserve(input.queryPoints->pointCount() / 20U);
    reporter.report(ProcessingStage::EvaluatingFieldGradients, 0.0, "Evaluating implicit-field gradients");
    try {
        for (std::size_t index = 0; index < input.queryPoints->pointCount(); ++index) {
            if (cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            const Eigen::Vector3d &position = input.queryPoints->points()[index].vector();
            std::optional<Eigen::Vector3d> gradient = input.implicitField->gradient(position);
            if (!gradient) {
                const double step = methodParameters->finiteDifferenceStep;
                Eigen::Vector3d finiteDifference;
                for (int axis = 0; axis < 3; ++axis) {
                    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
                    offset[axis] = step;
                    const double positive = input.implicitField->value(position + offset);
                    const double negative = input.implicitField->value(position - offset);
                    finiteDifference[axis] = (positive - negative) / (2.0 * step);
                }
                gradient = finiteDifference;
            }
            const double magnitude = gradient->norm();
            if (!gradient->allFinite() || !std::isfinite(magnitude) ||
                magnitude < methodParameters->minimumGradientMagnitude) {
                result.normalField.addInvalidNormal();
                result.invalidPointIndices.push_back(index);
                ++result.diagnostics.nearZeroGradientCount;
            } else {
                Eigen::Vector3d direction = *gradient / magnitude;
                if (methodParameters->invertDirection) {
                    direction = -direction;
                }
                const double confidence = std::clamp(
                    magnitude / (magnitude + methodParameters->minimumGradientMagnitude),
                    0.0,
                    1.0);
                result.normalField.addNormal(direction, confidence);
            }
            if ((index & 127U) == 0U) {
                reporter.report(
                    ProcessingStage::EvaluatingFieldGradients,
                    static_cast<double>(index) /
                        static_cast<double>(input.queryPoints->pointCount()),
                    "Evaluating implicit-field gradients");
            }
        }
    } catch (const std::exception &exception) {
        result.errorMessage = std::string("Implicit-field evaluation failed: ") + exception.what();
        return result;
    } catch (...) {
        result.errorMessage = "Implicit-field evaluation failed with an unknown exception.";
        return result;
    }

    result.normalField.setConsistentlyOriented(true);
    finalizeNormalDiagnostics(
        result.normalField,
        input.queryPoints->pointCount(),
        result.diagnostics);
    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "The implicit field produced no usable gradients.";
        return result;
    }
    result.succeeded = true;
    reporter.report(ProcessingStage::EvaluatingFieldGradients, 0.99, "Implicit-field normals ready");
    return result;
}
