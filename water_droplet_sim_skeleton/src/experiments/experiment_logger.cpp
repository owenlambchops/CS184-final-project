#include "wd/experiments/experiment_logger.h"
#include <fstream>

namespace wd {

void ExperimentLogger::beginRun(const std::string& name) {
    runName_ = name;
    samples_.clear();
}

void ExperimentLogger::record(double timeSec, const Scene& scene,
                              const SimulationStats& simStats, const RenderStats& renderStats) {
    MetricSample s;
    s.timeSec = timeSec;
    s.simMs = simStats.simMs;
    s.renderMs = renderStats.renderMs;
    s.meanVolumeError = simStats.meanVolumeError;
    s.maxVolumeError = simStats.maxVolumeError;
    s.dropletCount = static_cast<int>(scene.droplets().size());

    for (const auto& d : scene.droplets()) {
        s.totalVertexCount += static_cast<int>(d->positions().rows());
    }

    samples_.push_back(s);
}

void ExperimentLogger::saveCsv(const std::string& path) const {
    std::ofstream out(path);
    out << "run,time_sec,sim_ms,render_ms,mean_volume_error,max_volume_error,droplet_count,total_vertex_count\n";
    for (const auto& s : samples_) {
        out << runName_ << ','
            << s.timeSec << ','
            << s.simMs << ','
            << s.renderMs << ','
            << s.meanVolumeError << ','
            << s.maxVolumeError << ','
            << s.dropletCount << ','
            << s.totalVertexCount << '\n';
    }
}

void ExperimentLogger::clear() {
    samples_.clear();
}

} // namespace wd
