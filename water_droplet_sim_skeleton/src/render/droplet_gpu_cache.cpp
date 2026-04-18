#include "wd/render/droplet_gpu_cache.h"
#include <unordered_set>

namespace wd {

void DropletGpuCache::sync(const std::vector<std::unique_ptr<Droplet>>& droplets) {
    std::unordered_set<int> alive;

    for (const auto& d : droplets) {
        alive.insert(d->id());
        auto [it, inserted] = buffers_.try_emplace(d->id());
        if (inserted) it->second.initialize(d->faces());
        it->second.updateVertices(d->positions(), d->derived().vertexNormals);
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
