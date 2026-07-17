#include "model/processing/common/ProcessingProgress.h"

#include <algorithm>
#include <cmath>

double mapPhaseProgress(double phaseStart, double phaseEnd, double localFraction)
{
    const double safeLocal = std::isfinite(localFraction)
        ? std::clamp(localFraction, 0.0, 1.0)
        : 0.0;
    const double safeStart = std::isfinite(phaseStart)
        ? std::clamp(phaseStart, 0.0, 1.0)
        : 0.0;
    const double safeEnd = std::isfinite(phaseEnd)
        ? std::clamp(phaseEnd, safeStart, 1.0)
        : safeStart;
    return safeStart + safeLocal * (safeEnd - safeStart);
}
