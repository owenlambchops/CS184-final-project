#include "wd/interaction/drag_interactor.h"

namespace wd {

DragInteractor::DragInteractor(std::shared_ptr<DragForceField> dragField)
    : dragField_(std::move(dragField)) {}

void DragInteractor::update(
    const InputState& input, const Camera& camera, int viewportW, int viewportH, Scene& scene) {
    if (!dragField_ || !scene.hasSurface()) return;

    if (input.leftReleased) {
        dragField_->setActive(false);
        return;
    }

    if (input.leftDown || input.leftPressed) {
        Ray ray = picker_.makeCameraRay(input.mouseX, input.mouseY, viewportW, viewportH, camera);
        PickHit hit = picker_.pickSurface(ray, scene.surface());
        if (hit.hit) {
            dragField_->setActive(true);
            dragField_->setCenter(hit.position);
        }
    }
}

} // namespace wd
