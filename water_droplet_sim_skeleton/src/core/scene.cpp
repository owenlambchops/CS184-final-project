#include "wd/core/scene.h"

namespace wd {

void Scene::setSurface(std::unique_ptr<ISurface> surface) { surface_ = std::move(surface); }
bool Scene::hasSurface() const { return static_cast<bool>(surface_); }
const ISurface& Scene::surface() const { return *surface_; }
ISurface& Scene::surface() { return *surface_; }

void Scene::setForceField(std::shared_ptr<CompositeForceField> field) { forceField_ = std::move(field); }
bool Scene::hasForceField() const { return static_cast<bool>(forceField_); }
const IForceField& Scene::forceField() const { return *forceField_; }
CompositeForceField& Scene::compositeForceField() { return *forceField_; }

std::vector<std::unique_ptr<Droplet>>& Scene::droplets() { return droplets_; }
const std::vector<std::unique_ptr<Droplet>>& Scene::droplets() const { return droplets_; }

} // namespace wd
