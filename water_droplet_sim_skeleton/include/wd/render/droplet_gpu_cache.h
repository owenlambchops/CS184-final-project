#pragma once
#include "wd/render/gpu_mesh_buffer.h"
#include "wd/sim/droplet.h"
#include <unordered_map>

namespace wd {

class DropletGpuCache {
public:
    void sync(const std::vector<std::unique_ptr<Droplet>>& droplets);
    void drawDroplet(int dropletId) const;

private:
    std::unordered_map<int, GpuMeshBuffer> buffers_;
};

} // namespace wd
