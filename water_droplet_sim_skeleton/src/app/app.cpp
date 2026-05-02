#include "wd/app/app.h"
#include "wd/sim/droplet_factory.h"
#include "wd/sim/droplet_template.h"
#include "wd/surface/plane_surface.h"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <sstream>
#include <string>

namespace wd {

namespace {

glm::vec3 toGlm(const Vec3& v) {
    return glm::vec3(static_cast<float>(v.x()),
                     static_cast<float>(v.y()),
                     static_cast<float>(v.z()));
}

std::string loadTextFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Failed to open shader file: " << std::filesystem::absolute(path)
                  << " (cwd: " << std::filesystem::current_path() << ")\n";
        return {};
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

unsigned int compileShader(unsigned int type, const std::filesystem::path& path) {
    const std::string source = loadTextFile(path);
    if (source.empty()) return 0;

    const char* sourcePtr = source.c_str();
    const unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &sourcePtr, nullptr);
    glCompileShader(shader);

    int success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success == GL_TRUE) return shader;

    int logLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(std::max(logLength, 1), '\0');
    glGetShaderInfoLog(shader, logLength, nullptr, log.data());
    std::cerr << "Shader compile failed: " << path << "\n" << log << "\n";
    glDeleteShader(shader);
    return 0;
}

unsigned int createShaderProgram(
    const std::filesystem::path& vertexPath, const std::filesystem::path& fragmentPath) {
    const unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexPath);
    if (vertexShader == 0) return 0;

    const unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentPath);
    if (fragmentShader == 0) {
        glDeleteShader(vertexShader);
        return 0;
    }

    const unsigned int program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    int success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success == GL_TRUE) return program;

    int logLength = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
    std::string log(std::max(logLength, 1), '\0');
    glGetProgramInfoLog(program, logLength, nullptr, log.data());
    std::cerr << "Program link failed: " << vertexPath << " + " << fragmentPath << "\n"
              << log << "\n";
    glDeleteProgram(program);
    return 0;
}

constexpr std::array<float, 24> kPlaneVertices = {
    -3.0f, 0.0f, -3.0f, 0.0f, 1.0f, 0.0f,
     3.0f, 0.0f, -3.0f, 0.0f, 1.0f, 0.0f,
     3.0f, 0.0f,  3.0f, 0.0f, 1.0f, 0.0f,
    -3.0f, 0.0f,  3.0f, 0.0f, 1.0f, 0.0f,
};

constexpr std::array<unsigned int, 6> kPlaneIndices = {0, 1, 2, 0, 2, 3};

const glm::vec3 kLightDir = glm::normalize(glm::vec3(0.4f, 0.8f, 0.2f));

void setMat4(unsigned int program, const char* name, const glm::mat4& value) {
    const int location = glGetUniformLocation(program, name);
    if (location >= 0) glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
}

void setVec3(unsigned int program, const char* name, const glm::vec3& value) {
    const int location = glGetUniformLocation(program, name);
    if (location >= 0) glUniform3fv(location, 1, glm::value_ptr(value));
}

} // namespace

int App::run() {
    if (!initializeWindow()) return 1;
    if (!initialize()) {
        shutdown();
        return 1;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        processInput();
        update();
        render();
        glfwSwapBuffers(window_);
    }

    shutdown();
    return 0;
}

bool App::initializeWindow() {
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialize GLFW.\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    window_ = glfwCreateWindow(width_, height_, "Water Droplet Simulator", nullptr, nullptr);
    if (window_ == nullptr) {
        std::cerr << "Failed to create GLFW window.\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) {
        std::cerr << "Failed to load OpenGL functions via glad.\n";
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &App::framebufferSizeCallback);
    glfwGetFramebufferSize(window_, &width_, &height_);

    initializeGlState();
    glViewport(0, 0, width_, height_);
    return true;
}

bool App::initialize() {
    buildDefaultScene();
    camera_.setPosition(Vec3(2.2, 1.35, 2.2));

    renderer_ = std::make_unique<RefractiveRenderer>();
    renderer_->initialize(width_, height_);
    input_ = std::make_unique<InputRouter>(window_);
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    ui_ = std::make_unique<UiController>();
    logger_ = std::make_unique<ExperimentLogger>();

    if (!initializeRenderResources()) {
        std::cerr << "Failed to initialize milestone render resources.\n";
        return false;
    }

    return true;
}

void App::initializeGlState() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);
}

bool App::initializeRenderResources() {
    dropletProgram_ = createShaderProgram("assets/shaders/droplet.vert", "assets/shaders/droplet.frag");
    if (dropletProgram_ == 0) return false;

    planeProgram_ = createShaderProgram("assets/shaders/scene.vert", "assets/shaders/scene.frag");
    if (planeProgram_ == 0) return false;

    initializePlaneMesh();
    return planeVao_ != 0 && planeEbo_ != 0;
}

void App::initializePlaneMesh() {
    planeIndexCount_ = static_cast<int>(kPlaneIndices.size());

    glGenVertexArrays(1, &planeVao_);
    glGenBuffers(1, &planeVbo_);
    glGenBuffers(1, &planeEbo_);

    glBindVertexArray(planeVao_);

    glBindBuffer(GL_ARRAY_BUFFER, planeVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(kPlaneVertices.size() * sizeof(float)),
                 kPlaneVertices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(kPlaneIndices.size() * sizeof(unsigned int)),
                 kPlaneIndices.data(),
                 GL_STATIC_DRAW);

    constexpr int stride = 6 * static_cast<int>(sizeof(float));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void App::buildDefaultScene() {
    scene_.setSurface(std::make_unique<PlaneSurface>(Vec3::Zero(), Vec3::UnitY()));

    auto composite = std::make_shared<CompositeForceField>();
    composite->addField(std::make_shared<ConstantForceField>(gravityLikeForce_));
    dragField_ = std::make_shared<DragForceField>();
    composite->addField(dragField_);
    scene_.setForceField(composite);

    auto tpl = DropletTemplate::CreateSphericalMesh(3, 0.20);
    DropletFactory factory(tpl);

    SpawnDesc desc;
    desc.anchorWorld = Vec3(0.0, 0.0, 0.0);
    desc.initialVelocity = Vec3::Zero();
    desc.targetVolume = 0.0; // Use spawn-time fallback to initial droplet volume.
    desc.material = defaultMaterial_;
    scene_.droplets().push_back(factory.spawn(1, desc, scene_.surface()));

    MergeSplitController mergeSplit(factory);
    sim_ = std::make_unique<SimulationSystem>(solverParams_, std::move(mergeSplit));
}

void App::restartSimulation() {
    dropletCache_.clear();
    scene_ = Scene{};
    sim_.reset();
    dragField_.reset();

    buildDefaultScene();
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    singleStepRequested_ = false;
}

void App::processInput() {
    if (window_ == nullptr) return;

    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    const bool pauseKeyDown = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (pauseKeyDown && !pauseKeyWasDown_) {
        paused_ = !paused_;
    }
    pauseKeyWasDown_ = pauseKeyDown;

    const bool stepKeyDown = glfwGetKey(window_, GLFW_KEY_N) == GLFW_PRESS;
    if (paused_ && stepKeyDown && !stepKeyWasDown_) {
        singleStepRequested_ = true;
    }
    stepKeyWasDown_ = stepKeyDown;

    const bool restartKeyDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
    if (restartKeyDown && !restartKeyWasDown_) {
        restartSimulation();
    }
    restartKeyWasDown_ = restartKeyDown;

    if (input_) {
        double mouseX = 0.0;
        double mouseY = 0.0;
        glfwGetCursorPos(window_, &mouseX, &mouseY);
        input_->setMousePosition(mouseX, mouseY);
        input_->setLeftButton(glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    }
}

void App::update() {
    if (input_) input_->beginFrame();
    if (dragInteractor_ && input_) {
        dragInteractor_->update(input_->state(), camera_, width_, height_, scene_);
    }
    if (sim_ && (!paused_ || singleStepRequested_)) {
        sim_->step(scene_);
        singleStepRequested_ = false;
    }
}

void App::render() {
    using Clock = std::chrono::high_resolution_clock;
    const auto start = Clock::now();

    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const glm::vec3 eye = toGlm(camera_.position());
    const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const float aspect = height_ > 0 ? static_cast<float>(width_) / static_cast<float>(height_) : 1.0f;
    const glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
    const glm::mat4 model(1.0f);

    if (planeProgram_ != 0 && planeVao_ != 0) {
        glUseProgram(planeProgram_);
        setMat4(planeProgram_, "uModel", model);
        setMat4(planeProgram_, "uView", view);
        setMat4(planeProgram_, "uProj", proj);

        glBindVertexArray(planeVao_);
        glDrawElements(GL_TRIANGLES, planeIndexCount_, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    if (dropletProgram_ != 0) {
        dropletCache_.sync(scene_.droplets());

        glUseProgram(dropletProgram_);
        setMat4(dropletProgram_, "uModel", model);
        setMat4(dropletProgram_, "uView", view);
        setMat4(dropletProgram_, "uProj", proj);
        setVec3(dropletProgram_, "uLightDir", kLightDir);
        setVec3(dropletProgram_, "uViewPos", eye);

        for (const auto& droplet : scene_.droplets()) {
            dropletCache_.drawDroplet(droplet->id());
        }
    }

    glUseProgram(0);

    const auto end = Clock::now();
    RenderStats stats;
    stats.frameWidth = width_;
    stats.frameHeight = height_;
    stats.renderMs = std::chrono::duration<double, std::milli>(end - start).count();

    if (logger_ && sim_) logger_->record(sim_->timeSec(), scene_, sim_->stats(), stats);
}

void App::destroyRenderResources() {
    dropletCache_.clear();

    if (planeEbo_ != 0) glDeleteBuffers(1, &planeEbo_);
    if (planeVbo_ != 0) glDeleteBuffers(1, &planeVbo_);
    if (planeVao_ != 0) glDeleteVertexArrays(1, &planeVao_);
    if (planeProgram_ != 0) glDeleteProgram(planeProgram_);
    if (dropletProgram_ != 0) glDeleteProgram(dropletProgram_);

    dropletProgram_ = 0;
    planeProgram_ = 0;
    planeVao_ = 0;
    planeVbo_ = 0;
    planeEbo_ = 0;
    planeIndexCount_ = 0;
}

void App::shutdown() {
    destroyRenderResources();

    logger_.reset();
    ui_.reset();
    dragInteractor_.reset();
    input_.reset();
    sim_.reset();
    renderer_.reset();

    if (window_ != nullptr) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    glfwTerminate();
}

void App::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(window));
    if (app == nullptr) return;

    app->width_ = width;
    app->height_ = height;
    glViewport(0, 0, width, height);
    if (app->renderer_) app->renderer_->resize(width, height);
}

} // namespace wd
