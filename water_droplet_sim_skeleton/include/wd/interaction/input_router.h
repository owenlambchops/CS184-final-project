#pragma once
struct GLFWwindow;

namespace wd {

struct InputState {
    double mouseX = 0.0;
    double mouseY = 0.0;
    bool leftDown = false;
    bool leftPressed = false;
    bool leftReleased = false;
};

class InputRouter {
public:
    explicit InputRouter(GLFWwindow* window);

    void beginFrame();
    const InputState& state() const { return state_; }

    void setMousePosition(double x, double y);
    void setLeftButton(bool down);

private:
    GLFWwindow* window_ = nullptr;
    InputState state_;
    bool prevLeftDown_ = false;
};

} // namespace wd
