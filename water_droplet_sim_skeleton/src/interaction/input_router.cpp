#include "wd/interaction/input_router.h"

namespace wd {

InputRouter::InputRouter(GLFWwindow* window) : window_(window) {}

void InputRouter::beginFrame() {
    state_.leftPressed = state_.leftDown && !prevLeftDown_;
    state_.leftReleased = !state_.leftDown && prevLeftDown_;
    state_.rightPressed = state_.rightDown && !prevRightDown_;
    state_.rightReleased = !state_.rightDown && prevRightDown_;
    prevLeftDown_ = state_.leftDown;
    prevRightDown_ = state_.rightDown;
}

void InputRouter::setMousePosition(double x, double y) {
    state_.mouseX = x;
    state_.mouseY = y;
}

void InputRouter::setLeftButton(bool down) {
    state_.leftDown = down;
}

void InputRouter::setRightButton(bool down) {
    state_.rightDown = down;
}

void InputRouter::addScroll(double y) {
    state_.scrollY += y;
}

void InputRouter::clearFrameDeltas() {
    state_.scrollY = 0.0;
}

} // namespace wd
