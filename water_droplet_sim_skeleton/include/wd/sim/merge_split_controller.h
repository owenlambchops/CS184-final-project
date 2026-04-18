#pragma once
#include "wd/sim/droplet_factory.h"

namespace wd {

class MergeSplitController {
public:
    explicit MergeSplitController(DropletFactory factory);

    void process(std::vector<std::unique_ptr<Droplet>>& droplets, const ISurface& surface);

    double mergeDistanceFactor = 0.8;
    double splitElongationThreshold = 2.2;
    double splitMinVolume = 0.002;
    double postMergeDamping = 0.8;

    bool enableMerge = true;
    bool enableSplit = false;

private:
    bool shouldMerge(const Droplet& a, const Droplet& b) const;
    bool shouldSplit(const Droplet& d) const;

    DropletFactory factory_;
    int nextId_ = 10000;
};

} // namespace wd
