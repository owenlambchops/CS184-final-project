#pragma once
#include "wd/render/gpu_mesh_buffer.h"
#include "wd/sim/droplet.h"
#include <unordered_map>

namespace wd {

enum class DebugColorMode {
    None = 0,
    EdgeLength = 1,
    Speed = 2,
    Curvature = 3
};

class DropletGpuCache {
public:
    void setDebugColorMode(DebugColorMode mode) { colorMode_ = mode; }
    void sync(const std::vector<std::unique_ptr<Droplet>>& droplets);
    void drawDroplet(int dropletId) const;
    void clear() { buffers_.clear(); }

private:
    DebugColorMode colorMode_ = DebugColorMode::EdgeLength; // set here
    std::unordered_map<int, GpuMeshBuffer> buffers_;
};

} // namespace wd
