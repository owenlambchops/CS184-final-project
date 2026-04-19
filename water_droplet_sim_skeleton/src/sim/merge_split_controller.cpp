#include "wd/sim/merge_split_controller.h"
#include <utility>

namespace wd {

MergeSplitController::MergeSplitController(DropletFactory factory) : factory_(std::move(factory)) {}

void MergeSplitController::process(std::vector<std::unique_ptr<Droplet>>& droplets, const ISurface& surface) {
    if (enableMerge) {
        for (size_t i = 0; i < droplets.size(); ++i) {
            for (size_t j = i + 1; j < droplets.size(); ++j) {
                if (shouldMerge(*droplets[i], *droplets[j])) {
                    auto merged = factory_.spawnMerged(nextId_++, *droplets[i], *droplets[j], surface, postMergeDamping);
                    droplets.erase(droplets.begin() + static_cast<long>(j));
                    droplets.erase(droplets.begin() + static_cast<long>(i));
                    droplets.push_back(std::move(merged));
                    return;
                }
            }
        }
    }

    if (enableSplit) {
        for (size_t i = 0; i < droplets.size(); ++i) {
            if (shouldSplit(*droplets[i])) {
                int firstId = nextId_++;
                int secondId = nextId_++;
                auto children = factory_.spawnSplit(firstId, secondId, *droplets[i], surface, 0.5);
                droplets.erase(droplets.begin() + static_cast<long>(i));
                droplets.push_back(std::move(children.first));
                droplets.push_back(std::move(children.second));
                return;
            }
        }
    }
}

bool MergeSplitController::shouldMerge(const Droplet& a, const Droplet& b) const {
    double threshold = mergeDistanceFactor * (a.derived().footprintRadius + b.derived().footprintRadius);
    return (a.derived().centerOfMass - b.derived().centerOfMass).norm() < threshold;
}

bool MergeSplitController::shouldSplit(const Droplet& d) const {
    return d.targetVolume() >= splitMinVolume && d.derived().elongationRatio > splitElongationThreshold;
}

} // namespace wd
