#include "wd/render/droplet_gpu_cache.h"
#include <algorithm>
#include <unordered_set>

namespace wd {

namespace {

Eigen::VectorXd buildDebugValues(const Droplet& d, DebugColorMode mode) {
    const int n = d.positions().rows();
    Eigen::VectorXd out = Eigen::VectorXd::Zero(n);

    if (mode == DebugColorMode::None || n == 0) return out;

    if (mode == DebugColorMode::EdgeLength && d.derived().vertexMeanEdgeLength.size() == n) {
        const double lo = std::max(d.derived().minEdgeLength, 0.0);
        const double hi = std::max(d.derived().maxEdgeLength, lo + 1e-8);
        for (int i = 0; i < n; ++i) out(i) = std::clamp((d.derived().vertexMeanEdgeLength(i) - lo) / (hi - lo), 0.0, 1.0);
        return out;
    }

    if (mode == DebugColorMode::Speed && d.derived().vertexSpeed.size() == n) {
        const double hi = std::max(d.derived().vertexSpeed.maxCoeff(), 1e-8);
        for (int i = 0; i < n; ++i) out(i) = std::clamp(d.derived().vertexSpeed(i) / hi, 0.0, 1.0);
        return out;
    }

    if (mode == DebugColorMode::Curvature && d.derived().vertexCurvature.size() == n) {
        const double hi = std::max(d.derived().vertexCurvature.maxCoeff(), 1e-8);
        for (int i = 0; i < n; ++i) out(i) = std::clamp(d.derived().vertexCurvature(i) / hi, 0.0, 1.0);
        return out;
    }

    return out;
}

} // namespace

void DropletGpuCache::sync(const std::vector<std::unique_ptr<Droplet>>& droplets) {
    std::unordered_set<int> alive;

    for (const auto& d : droplets) {
        alive.insert(d->id());
        auto [it, inserted] = buffers_.try_emplace(d->id());
        if (inserted) it->second.initialize(d->faces());
        const Eigen::VectorXd dbg = buildDebugValues(*d, colorMode_);
        it->second.updateVertices(d->positions(), d->derived().vertexNormals, dbg);
    }

    for (auto it = buffers_.begin(); it != buffers_.end();) {
        if (!alive.count(it->first)) it = buffers_.erase(it);
        else ++it;
    }
}

void DropletGpuCache::drawDroplet(int dropletId) const {
    auto it = buffers_.find(dropletId);
    if (it != buffers_.end()) it->second.draw();
}

} // namespace wd
