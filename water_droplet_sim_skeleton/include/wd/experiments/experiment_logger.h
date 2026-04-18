#pragma once
#include "wd/core/scene.h"
#include "wd/core/types.h"

namespace wd {

struct MetricSample {
    double timeSec = 0.0;
    double simMs = 0.0;
    double renderMs = 0.0;
    double meanVolumeError = 0.0;
    double maxVolumeError = 0.0;
    int dropletCount = 0;
    int totalVertexCount = 0;
};

class ExperimentLogger {
public:
    void beginRun(const std::string& name);
    void record(double timeSec, const Scene& scene, const SimulationStats& simStats, const RenderStats& renderStats);
    void saveCsv(const std::string& path) const;
    void clear();

private:
    std::string runName_;
    std::vector<MetricSample> samples_;
};

} // namespace wd
