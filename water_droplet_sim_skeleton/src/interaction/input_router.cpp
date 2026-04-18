#include "wd/interaction/input_router.h"

namespace wd {

InputRouter::InputRouter(GLFWwindow* window) : window_(window) {}

void InputRouter::beginFrame() {
    state_.leftPressed = state_.leftDown && !prevLeftDown_;
    state_.leftReleased = !state_.leftDown && prevLeftDown_;
    prevLeftDown_ = state_.leftDown;
}

void InputRouter::setMousePosition(double x, double y) {
    state_.mouseX = x;
    state_.mouseY = y;
}

void InputRouter::setLeftButton(bool down) {
    state_.leftDown = down;
}

} // namespace wd
