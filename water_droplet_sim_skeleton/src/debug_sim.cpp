// ============================================================
//  debug_sim.cpp  —  minimal debug viewer for the water drop sim
//
//  Renders: droplet mesh (solid Phong + wireframe) + surface plane grid.
//  No refraction, caustics, HDR environment, or refractive renderer.
//
//  ── WHAT TO EDIT ───────────────────────────────────────────
//
//  Forces / plane / starting position:
//    Edit the constants in the "Debug Configuration" section below.
//    Gravity, spawn position, surface friction, and plane size are all
//    compile-time constants here for quick iteration.
//    Most parameters are also tunable at runtime via the ImGui panel.
//
//  Number of mesh vertices:
//    Change kMeshSubdivisions (0 = 12 verts/20 tris, 1 = 42/80,
//    2 = 162/320, 3 = 642/1280, 4 = 2562/5120).
//
//  Starting position:
//    Change kSpawnPos.  Set kAutoSpawn = false to suppress the
//    automatic first droplet and use the ImGui "Spawn" button instead.
//
//  Solver operator flags (which physics are on):
//    Toggle enableCurvatureFlow / enableViscosity / etc. in the
//    SolverParams block below, or use the ImGui checkboxes at runtime.
//
//  REBUILD AND RUN:
//    cd water_droplet_sim_skeleton/build
//    cmake --build . --target wd_debug
//    ./wd_debug
//
//  CAMERA:  right-click drag = orbit   |  scroll = zoom
//  KEYS:    Space = pause   N = single step   Esc = quit
// ============================================================

#include "wd/core/scene.h"
#include "wd/core/types.h"
#include "wd/forces/iforce_field.h"
#include "wd/sim/cgl_math.h"
#include "wd/sim/droplet_factory.h"
#include "wd/sim/droplet_template.h"
#include "wd/sim/merge_split_controller.h"
#include "wd/sim/simulation_system.h"
#include "wd/surface/plane_surface.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ── Debug Configuration ──────────────────────────────────────────────────────
// Edit these and rebuild.  Most are also tunable live in the ImGui panel.
// ─────────────────────────────────────────────────────────────────────────────

// Icosphere subdivision level — see comment above for vertex counts.
static constexpr int    kMeshSubdivisions   = 3;
// Rest radius of the droplet (world units).
static constexpr double kDropletRadius      = 0.20;
// World-space spawn position of the first droplet.
static const     wd::Vec3 kSpawnPos         = wd::Vec3(0.0, 1.0, 0.0);
// Automatically spawn one droplet at startup.
static constexpr bool   kAutoSpawn          = true;

// Gravity acceleration applied to every vertex (world Y).
static constexpr double kGravityY           = -9.81;
// Plane surface friction coefficient (mu in Zhang eq. 2).
static constexpr double kSurfaceFriction    = 0.02;
// Surface tension coefficient (gamma / density used in curvature flow).
static constexpr double kSurfaceTension     = 6500.0;
// Linear velocity damping (mu in viscosity operator).
static constexpr double kViscousDamping     = 8.0;
// Laplacian viscosity coefficient (eta in viscosity operator).
static constexpr double kLaplacianViscosity = 15.0;
// Volume preservation stiffness (k_v in curvature flow pressure term).
static constexpr double kVolumeStiffness    = 6500.0;

// Substeps per display frame (more = more stable, more expensive).
static constexpr int    kSubsteps           = 2;
// Physics timestep in seconds.
static constexpr double kDt                 = 1.0 / 120.0;

// Half-size of the rendered plane grid (world units).
static constexpr float  kPlaneHalfSize      = 2.0f;
// Grid line spacing on the plane.
static constexpr float  kGridSpacing        = 0.25f;

// ─────────────────────────────────────────────────────────────────────────────

namespace {

// ── GLFW scroll accumulator ───────────────────────────────────────────────────
double gScrollY = 0.0;
void scrollCb(GLFWwindow*, double, double dy) { gScrollY += dy; }

// ── Shader utilities ──────────────────────────────────────────────────────────

GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::cerr << "[shader] compile error:\n" << log << "\n";
    }
    return s;
}

GLuint linkProgram(const char* vertSrc, const char* fragSrc) {
    GLuint vs = compileShader(GL_VERTEX_SHADER,   vertSrc);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragSrc);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(prog, sizeof(log), nullptr, log);
        std::cerr << "[shader] link error:\n" << log << "\n";
    }
    return prog;
}

// ── Embedded GLSL shaders ─────────────────────────────────────────────────────

// Mesh solid: interleaved pos(loc 0) + normal(loc 1), Phong lighting.
const char* kMeshVert = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
out vec3 vNormal;
out vec3 vPos;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNormal = normalize(uNormalMat * aNormal);
    vPos    = aPos;
}
)glsl";

const char* kMeshFrag = R"glsl(
#version 330 core
in  vec3 vNormal;
in  vec3 vPos;
uniform vec3 uCamPos;
uniform vec3 uColor;
out vec4 FragColor;
const vec3 kLight = vec3(0.45, 0.82, 0.36);
void main() {
    vec3 n   = normalize(vNormal);
    vec3 v   = normalize(uCamPos - vPos);
    float d  = max(dot(n, kLight), 0.0);
    float sp = pow(max(dot(reflect(-kLight, n), v), 0.0), 24.0);
    vec3 col = uColor * (0.20 + 0.65 * d) + vec3(0.35) * sp;
    FragColor = vec4(col, 1.0);
}
)glsl";

// Flat: position only (loc 0), uniform RGBA color.
// Used for wireframe overlay and plane grid lines.
const char* kFlatVert = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main() { gl_Position = uMVP * vec4(aPos, 1.0); }
)glsl";

const char* kFlatFrag = R"glsl(
#version 330 core
uniform vec4 uColor;
out vec4 FragColor;
void main() { FragColor = uColor; }
)glsl";

// ── Orbit camera ──────────────────────────────────────────────────────────────

struct OrbitCam {
    float az   = 0.70f;  // azimuth  (radians around Y)
    float el   = 0.45f;  // elevation (radians above XZ plane)
    float dist = 4.0f;
    glm::vec3 target = {0.f, 0.15f, 0.f};

    glm::vec3 eye() const {
        return target + dist * glm::vec3(
            std::cos(el) * std::sin(az),
            std::sin(el),
            std::cos(el) * std::cos(az)
        );
    }
    glm::mat4 view()        const { return glm::lookAt(eye(), target, {0.f, 1.f, 0.f}); }
    glm::mat4 proj(float a) const { return glm::perspective(glm::radians(45.f), a, 0.01f, 100.f); }
};

// ── GPU buffers for one droplet mesh ─────────────────────────────────────────
// VBO layout: interleaved [ pos.x pos.y pos.z  nx ny nz ] per vertex (6 floats).
// triEbo : face index triples  → GL_TRIANGLES solid pass.
// edgeEbo: edge index pairs    → GL_LINES wireframe pass.

struct MeshGpu {
    GLuint vao = 0, vbo = 0, triEbo = 0, edgeEbo = 0;
    int nTris = 0, nEdges = 0;

    void init() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &triEbo);
        glGenBuffers(1, &edgeEbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    void upload(const wd::Droplet& drop) {
        const auto& X = drop.positions();
        const auto& N = drop.derived().vertexNormals;
        const auto& F = drop.faces();
        nTris = F.rows();

        // Interleaved position + normal
        std::vector<float> vdata;
        vdata.reserve(static_cast<size_t>(X.rows()) * 6);
        for (int i = 0; i < X.rows(); ++i) {
            vdata.push_back(static_cast<float>(X(i, 0)));
            vdata.push_back(static_cast<float>(X(i, 1)));
            vdata.push_back(static_cast<float>(X(i, 2)));
            if (i < N.rows()) {
                vdata.push_back(static_cast<float>(N(i, 0)));
                vdata.push_back(static_cast<float>(N(i, 1)));
                vdata.push_back(static_cast<float>(N(i, 2)));
            } else {
                vdata.push_back(0.f); vdata.push_back(1.f); vdata.push_back(0.f);
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vdata.size() * sizeof(float)),
                     vdata.data(), GL_DYNAMIC_DRAW);

        // Triangle EBO
        std::vector<unsigned int> triIdx;
        triIdx.reserve(static_cast<size_t>(F.rows()) * 3);
        for (int fi = 0; fi < F.rows(); ++fi) {
            triIdx.push_back(static_cast<unsigned int>(F(fi, 0)));
            triIdx.push_back(static_cast<unsigned int>(F(fi, 1)));
            triIdx.push_back(static_cast<unsigned int>(F(fi, 2)));
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(triIdx.size() * sizeof(unsigned int)),
                     triIdx.data(), GL_DYNAMIC_DRAW);

        // Edge EBO — unique edges from face topology
        const auto edges = wd::buildUniqueEdgesFromFaces(F);
        nEdges = static_cast<int>(edges.size());
        std::vector<unsigned int> edgeIdx;
        edgeIdx.reserve(edges.size() * 2);
        for (const auto& e : edges) {
            edgeIdx.push_back(static_cast<unsigned int>(e.x()));
            edgeIdx.push_back(static_cast<unsigned int>(e.y()));
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(edgeIdx.size() * sizeof(unsigned int)),
                     edgeIdx.data(), GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // Solid triangles — call with polygon offset enabled to avoid z-fighting.
    void drawSolid() const {
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triEbo);
        glDrawElements(GL_TRIANGLES, nTris * 3, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    // Wireframe edges on top of the solid.
    void drawWire() const {
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEbo);
        glDrawElements(GL_LINES, nEdges * 2, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    void destroy() {
        glDeleteBuffers(1, &edgeEbo);
        glDeleteBuffers(1, &triEbo);
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
        vao = vbo = triEbo = edgeEbo = 0;
    }
};

// ── GPU buffers for the plane grid ───────────────────────────────────────────

struct PlaneGpu {
    GLuint vao = 0, vbo = 0;
    int nVerts = 0;

    void init(float half, float spacing) {
        std::vector<float> pts;
        for (float x = -half; x <= half + 1e-4f; x += spacing) {
            pts.push_back(x); pts.push_back(0.f); pts.push_back(-half);
            pts.push_back(x); pts.push_back(0.f); pts.push_back( half);
        }
        for (float z = -half; z <= half + 1e-4f; z += spacing) {
            pts.push_back(-half); pts.push_back(0.f); pts.push_back(z);
            pts.push_back( half); pts.push_back(0.f); pts.push_back(z);
        }
        nVerts = static_cast<int>(pts.size() / 3);

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(pts.size() * sizeof(float)),
                     pts.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    void draw() const {
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, nVerts);
        glBindVertexArray(0);
    }

    void destroy() {
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
        vao = vbo = 0;
    }
};

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

int main() {
    // ── 1. GLFW + OpenGL ─────────────────────────────────────────────────────
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "glfwInit failed\n";
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    int winW = 1280, winH = 800;
    GLFWwindow* win = glfwCreateWindow(winW, winH, "WD Debug Viewer", nullptr, nullptr);
    if (!win) { std::cerr << "glfwCreateWindow failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glfwSetScrollCallback(win, scrollCb);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        std::cerr << "gladLoadGL failed\n"; glfwTerminate(); return 1;
    }

    // ── 2. ImGui ─────────────────────────────────────────────────────────────
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(win, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ── 3. Simulation scene ───────────────────────────────────────────────────
    // Edit kXxx constants at top of file or the SolverParams block here.

    wd::MaterialParams material;
    material.surfaceTension     = kSurfaceTension;
    material.viscousDamping     = kViscousDamping;
    material.laplacianViscosity = kLaplacianViscosity;
    material.volumeStiffness    = kVolumeStiffness;

    wd::SolverParams solverParams;
    solverParams.substeps = kSubsteps;
    solverParams.dt       = kDt;
    // Toggle individual operators here or via ImGui at runtime:
    solverParams.enableExternalForce  = true;
    solverParams.enableCurvatureFlow  = true;
    solverParams.enableViscosity      = true;
    solverParams.enableCollision      = true;
    solverParams.enableVolumeCorrect  = true;
    solverParams.enableContactAngle   = true;

    wd::Scene scene;
    scene.setSurface(std::make_unique<wd::PlaneSurface>(wd::Vec3::Zero(), wd::Vec3::UnitY()));
    scene.surface().material().friction           = kSurfaceFriction;
    scene.surface().material().contactLineDamping = 15.0;

    // Force field: gravity only (no drag in debug mode).
    // To adjust gravity at runtime use the ImGui "Gravity" slider.
    auto gravityField = std::make_shared<wd::ConstantForceField>(wd::Vec3(0.0, kGravityY, 0.0));
    auto composite    = std::make_shared<wd::CompositeForceField>();
    composite->addField(gravityField);
    scene.setForceField(composite);

    auto dropletTemplate = wd::DropletTemplate::CreateSphericalMesh(kMeshSubdivisions, kDropletRadius);
    wd::DropletFactory factory(dropletTemplate);
    wd::MergeSplitController mergeSplit(factory);
    auto sim = std::make_unique<wd::SimulationSystem>(solverParams, std::move(mergeSplit));

    int nextId = 1;
    auto spawnAt = [&](wd::Vec3 pos) {
        wd::SpawnDesc desc;
        desc.anchorWorld     = pos;
        desc.initialVelocity = wd::Vec3::Zero();
        desc.targetVolume    = 0.0;
        desc.material        = material;
        scene.droplets().push_back(factory.spawn(nextId++, desc, scene.surface()));
    };
    if (kAutoSpawn) spawnAt(kSpawnPos);

    // ── 4. Shaders ───────────────────────────────────────────────────────────
    GLuint meshProg = linkProgram(kMeshVert, kMeshFrag);
    GLuint flatProg = linkProgram(kFlatVert, kFlatFrag);

    // Precompute uniform locations
    GLint locMeshMVP    = glGetUniformLocation(meshProg, "uMVP");
    GLint locMeshNorm   = glGetUniformLocation(meshProg, "uNormalMat");
    GLint locMeshCam    = glGetUniformLocation(meshProg, "uCamPos");
    GLint locMeshColor  = glGetUniformLocation(meshProg, "uColor");
    GLint locFlatMVP    = glGetUniformLocation(flatProg, "uMVP");
    GLint locFlatColor  = glGetUniformLocation(flatProg, "uColor");

    // ── 5. GPU geometry ───────────────────────────────────────────────────────
    MeshGpu meshGpu;
    meshGpu.init();

    PlaneGpu planeGpu;
    planeGpu.init(kPlaneHalfSize, kGridSpacing);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ── 6. Runtime state ─────────────────────────────────────────────────────
    OrbitCam cam;
    bool paused    = false;
    bool stepOnce  = false;
    bool spaceWas  = false;
    bool nWas      = false;
    bool rWas      = false;
    bool rmbWas    = false;

    double lastMx  = 0.0, lastMy = 0.0;

    double prevTime  = glfwGetTime();
    double fpsTimer  = 0.0;
    int    fpsFrames = 0;
    float  fps       = 0.f;

    // ── Oscillation diagnostic state ─────────────────────────────────────────
    // Detects vertices whose displacement direction reversed since last frame.
    // A 2-frame sign-flip is the fingerprint of a P-controller limit cycle.
    wd::MatX3d diagPrevPos;
    wd::MatX3d diagPrevDelta;
    int    diagOscCount       = 0;   // total oscillating verts
    int    diagOscSurface     = 0;   // oscillating verts near the plane (Y < R*0.25)
    int    diagOscBulk        = 0;   // oscillating verts in the bulk
    double diagOscMaxAmp      = 0.0;
    static constexpr double kOscAmpThresh  = 1e-6;  // ignore sub-numerical noise
    static constexpr double kSurfaceYLimit = kDropletRadius * 0.25; // "near surface" threshold

    // ImGui editable spawn position (initialised to config default).
    float spawnXYZ[3] = {
        static_cast<float>(kSpawnPos.x()),
        static_cast<float>(kSpawnPos.y()),
        static_cast<float>(kSpawnPos.z())
    };
    // Gravity vec mirrored for ImGui sliders.
    float gravXYZ[3] = { 0.f, static_cast<float>(kGravityY), 0.f };

    // Defined after gravXYZ so the lambda can capture it by reference.
    auto resetSim = [&]() {
        scene.droplets().clear();
        nextId = 1;
        wd::MergeSplitController ms2(factory);
        sim = std::make_unique<wd::SimulationSystem>(solverParams, std::move(ms2));
        gravityField->setForce(wd::Vec3(gravXYZ[0], gravXYZ[1], gravXYZ[2]));
        if (kAutoSpawn) spawnAt(kSpawnPos);
        paused = false;
    };

    // ── 7. Main loop ─────────────────────────────────────────────────────────
    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();

        const double now = glfwGetTime();
        const double dt  = now - prevTime;
        prevTime = now;
        ++fpsFrames;
        fpsTimer += dt;
        if (fpsTimer >= 0.5) {
            fps      = static_cast<float>(fpsFrames) / static_cast<float>(fpsTimer);
            fpsFrames = 0;
            fpsTimer  = 0.0;
        }

        glfwGetFramebufferSize(win, &winW, &winH);
        glViewport(0, 0, winW, winH);

        // ── Camera orbit ──────────────────────────────────────────────────────
        const bool imguiMouse = ImGui::GetIO().WantCaptureMouse;
        double mx, my;
        glfwGetCursorPos(win, &mx, &my);
        const bool rmbNow = glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (rmbNow && rmbWas && !imguiMouse) {
            const float dx = static_cast<float>(mx - lastMx) * 0.008f;
            const float dy = static_cast<float>(my - lastMy) * 0.008f;
            cam.az += dx;
            cam.el  = std::clamp(cam.el - dy, -1.4f, 1.4f);
        }
        rmbWas = rmbNow;
        lastMx = mx; lastMy = my;

        if (!imguiMouse && gScrollY != 0.0) {
            cam.dist = std::clamp(cam.dist - static_cast<float>(gScrollY) * 0.25f, 0.3f, 20.f);
        }
        gScrollY = 0.0;

        // ── Keyboard shortcuts ────────────────────────────────────────────────
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            const bool spaceNow = glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS;
            if (spaceNow && !spaceWas) paused = !paused;
            spaceWas = spaceNow;

            const bool nNow = glfwGetKey(win, GLFW_KEY_N) == GLFW_PRESS;
            if (nNow && !nWas && paused) stepOnce = true;
            nWas = nNow;

            const bool rNow = glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS;
            if (rNow && !rWas) resetSim();
            rWas = rNow;

            if (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(win, GLFW_TRUE);
        }

        // ── Simulation step ───────────────────────────────────────────────────
        if (!paused || stepOnce) {
            // Sync live ImGui tweaks into the sim before each step.
            sim->singleDropletSolver().params() = solverParams;
            gravityField->setForce(wd::Vec3(gravXYZ[0], gravXYZ[1], gravXYZ[2]));
            for (auto& dropPtr : scene.droplets()) dropPtr->material() = material;

            sim->step(scene);
            stepOnce = false;

            // ── Oscillation diagnostic update ─────────────────────────────
            // After each sim step, check each vertex: did its displacement
            // direction reverse from the previous frame?  A consistent
            // sign-flip (dot product < 0) between consecutive displacements
            // means the vertex is ping-ponging — a 2-step limit cycle.
            // Vertices are split into surface-zone (Y < kSurfaceYLimit) and
            // bulk to help localise which operator is responsible.
            if (!scene.droplets().empty()) {
                const wd::MatX3d& curPos = scene.droplets().front()->positions();
                if (diagPrevPos.rows() == curPos.rows()) {
                    const wd::MatX3d delta = curPos - diagPrevPos;
                    if (diagPrevDelta.rows() == delta.rows()) {
                        diagOscCount   = 0;
                        diagOscSurface = 0;
                        diagOscBulk    = 0;
                        diagOscMaxAmp  = 0.0;
                        for (int vi = 0; vi < delta.rows(); ++vi) {
                            const double amp = delta.row(vi).norm();
                            if (amp < kOscAmpThresh) continue;
                            if (delta.row(vi).dot(diagPrevDelta.row(vi)) < 0.0) {
                                ++diagOscCount;
                                diagOscMaxAmp = std::max(diagOscMaxAmp, amp);
                                if (curPos(vi, 1) < kSurfaceYLimit)
                                    ++diagOscSurface;
                                else
                                    ++diagOscBulk;
                            }
                        }
                    }
                    diagPrevDelta = delta;
                }
                diagPrevPos = curPos;
            }
        }

        // ── ImGui panel ───────────────────────────────────────────────────────
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_Once);
        ImGui::SetNextWindowSize({310, 0}, ImGuiCond_Once);
        ImGui::Begin("WD Debug");

        // Stats
        ImGui::Text("FPS %.1f  |  t = %.3f s", fps, static_cast<float>(sim->timeSec()));
        ImGui::Text("Droplets: %d", static_cast<int>(scene.droplets().size()));
        if (!scene.droplets().empty()) {
            const auto& d = *scene.droplets().front();
            ImGui::Text("Verts %d  |  Faces %d",
                        static_cast<int>(d.positions().rows()),
                        static_cast<int>(d.faces().rows()));
            ImGui::Text("Volume %.5f  (target %.5f)",
                        d.derived().currentVolume, d.targetVolume());
            ImGui::Text("Vol error %.2e",
                        std::abs(d.derived().currentVolume - d.targetVolume()));
        }

        ImGui::Separator();
        ImGui::TextDisabled("RMB drag=orbit  Scroll=zoom  Space=pause  N=step  R=reset");

        // Sim controls
        ImGui::Separator();
        if (ImGui::Button(paused ? "Resume (Space)" : "Pause (Space)")) paused = !paused;
        ImGui::SameLine();
        if (ImGui::Button("Step (N)") && paused) stepOnce = true;
        ImGui::SameLine();
        if (ImGui::Button("Reset (R)")) resetSim();

        // Spawn
        ImGui::Separator();
        ImGui::Text("Spawn");
        ImGui::SetNextItemWidth(200);
        ImGui::InputFloat3("##spawnxyz", spawnXYZ);
        ImGui::SameLine();
        if (ImGui::Button("Spawn")) {
            spawnAt(wd::Vec3(spawnXYZ[0], spawnXYZ[1], spawnXYZ[2]));
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear")) {
            scene.droplets().clear();
            nextId = 1;
        }

        // Forces
        ImGui::Separator();
        ImGui::Text("Forces");
        ImGui::SetNextItemWidth(200);
        ImGui::SliderFloat3("Gravity##g", gravXYZ, -20.f, 20.f);

        // Material
        ImGui::Separator();
        ImGui::Text("Material");
        {
            float v = static_cast<float>(material.surfaceTension);
            if (ImGui::SliderFloat("Surface Tension", &v, 0.f, 8000.f)) material.surfaceTension = v;
        }
        {
            float v = static_cast<float>(material.viscousDamping);
            if (ImGui::SliderFloat("Viscous Damp", &v, 0.f, 30.f)) material.viscousDamping = v;
        }
        {
            float v = static_cast<float>(material.laplacianViscosity);
            if (ImGui::SliderFloat("Laplacian Visc", &v, 0.f, 50.f)) material.laplacianViscosity = v;
        }
        {
            float v = static_cast<float>(material.volumeStiffness);
            if (ImGui::SliderFloat("Volume Stiffness", &v, 0.f, 10000.f)) material.volumeStiffness = v;
        }
        ImGui::Checkbox("Local Vol Correct",  &material.enableLocalVolumeCorrection);
        ImGui::Checkbox("Global Vol Correct", &material.enableGlobalVolumeCorrection);

        // Surface
        ImGui::Separator();
        ImGui::Text("Surface");
        {
            float fr = static_cast<float>(scene.surface().material().friction);
            if (ImGui::SliderFloat("Friction", &fr, 0.f, 1.f))
                scene.surface().material().friction = fr;
        }
        {
            float adv = static_cast<float>(scene.surface().material().advContactAngleDeg);
            if (ImGui::SliderFloat("Adv Contact Angle", &adv, 0.f, 180.f))
                scene.surface().material().advContactAngleDeg = adv;
        }
        {
            float rec = static_cast<float>(scene.surface().material().recContactAngleDeg);
            if (ImGui::SliderFloat("Rec Contact Angle", &rec, 0.f, 180.f))
                scene.surface().material().recContactAngleDeg = rec;
        }
        {
            float cs = static_cast<float>(scene.surface().material().contactStiffness);
            if (ImGui::SliderFloat("Contact Stiffness", &cs, 0.f, 60.f))
                scene.surface().material().contactStiffness = cs;
        }
        {
            float cld = static_cast<float>(scene.surface().material().contactLineDamping);
            if (ImGui::SliderFloat("Contact Line Damping", &cld, 0.f, 50.f))
                scene.surface().material().contactLineDamping = cld;
        }
        {
            float ad = static_cast<float>(solverParams.adhesionDistance);
            if (ImGui::SliderFloat("Adhesion Dist", &ad, 0.f, 0.1f))
                solverParams.adhesionDistance = static_cast<double>(ad);
        }

        // Solver
        ImGui::Separator();
        ImGui::Text("Solver operators");
        ImGui::Checkbox("External Force",  &solverParams.enableExternalForce);
        ImGui::Checkbox("Curvature Flow",  &solverParams.enableCurvatureFlow);
        ImGui::Checkbox("Viscosity",       &solverParams.enableViscosity);
        ImGui::Checkbox("Collision",       &solverParams.enableCollision);
        ImGui::Checkbox("Volume Correct",  &solverParams.enableVolumeCorrect);
        ImGui::Checkbox("Contact Angle",   &solverParams.enableContactAngle);
        ImGui::SliderInt("Substeps",       &solverParams.substeps, 1, 16);
        {
            float vd = static_cast<float>(solverParams.vertexDamping);
            if (ImGui::SliderFloat("Vertex Damping", &vd, 0.f, 5.f))
                solverParams.vertexDamping = vd;
        }

        // ── Oscillation diagnostic panel ──────────────────────────────────
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Oscillation Diagnostic")) {
            const int totalVerts = scene.droplets().empty()
                ? 0 : static_cast<int>(scene.droplets().front()->positions().rows());

            ImGui::Text("Reversing verts:  %d / %d", diagOscCount, totalVerts);
            ImGui::Text("  surface zone (Y<%.2f): %d", static_cast<float>(kSurfaceYLimit), diagOscSurface);
            ImGui::Text("  bulk:                  %d", diagOscBulk);
            ImGui::Text("Max reversal amp: %.2e", diagOscMaxAmp);
            ImGui::TextDisabled("(vertex displacement reversed direction last frame)");

            // Stability ratio for the l=2 shape mode:
            // omega_2 * dt_sub < 2 required for Verlet stability.
            // omega_2 = sqrt(surfaceTension * 6 / (density * R^2))
            // Ratio > 1 = underdamped/unstable for that mode.
            if (!scene.droplets().empty()) {
                const auto& d = *scene.droplets().front();
                const double R   = d.derived().meanEdgeLength > 0
                    ? std::cbrt(3.0 * d.derived().restVolume / (4.0 * 3.14159265)) : kDropletRadius;
                const double rho = std::max(d.material().density, 1e-8);
                const double gam = d.material().surfaceTension;
                const double dtSub = solverParams.dt / std::max(solverParams.substeps, 1);
                const double omega2 = std::sqrt(gam * 6.0 / (rho * R * R));
                const double ratio  = omega2 * dtSub / 2.0; // <1 = stable, >1 = unstable
                ImGui::Spacing();
                ImGui::Text("l=2 mode  omega=%.0f rad/s", static_cast<float>(omega2));
                ImGui::Text("Verlet stability ratio: %.2f  (%s)",
                            static_cast<float>(ratio),
                            ratio < 1.0 ? "STABLE" : ratio < 2.0 ? "marginal" : "UNSTABLE");
                ImGui::TextDisabled("ratio = omega2*dt_sub/2 -- keep below 1");
            }

            ImGui::Spacing();
            ImGui::TextDisabled("Isolation guide:");
            ImGui::TextDisabled("If bulk >> surface  -> curvature flow instability");
            ImGui::TextDisabled("  fix: lower Surface Tension or raise Substeps");
            ImGui::TextDisabled("If surface >> bulk  -> collision/volume interaction");
            ImGui::TextDisabled("  fix: set Volume Stiffness=0, or lower Collision Pushout");
            ImGui::TextDisabled("Try: set Volume Stiffness=0 -> does count drop?");
            ImGui::TextDisabled("Try: disable Curvature Flow -> does count drop?");

            // Expose collision pushout epsilon (not in main panel).
            ImGui::Separator();
            {
                float peps = static_cast<float>(solverParams.collisionPushoutEps);
                if (ImGui::SliderFloat("Collision Pushout Eps", &peps, 0.f, 1e-2f, "%.2e"))
                    solverParams.collisionPushoutEps = static_cast<double>(peps);
            }
            {
                ImGui::Text("maxInternalAccel: %.1f  (SolverParams)", static_cast<float>(solverParams.maxInternalAccel));
                float mia = static_cast<float>(solverParams.maxInternalAccel);
                if (ImGui::SliderFloat("Max Internal Accel", &mia, 1.f, 5000.f, "%.0f"))
                    solverParams.maxInternalAccel = static_cast<double>(mia);
            }
        }

        ImGui::End();
        ImGui::Render();

        // ── Render ────────────────────────────────────────────────────────────
        glClearColor(0.10f, 0.11f, 0.13f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const float aspect = (winH > 0) ? static_cast<float>(winW) / static_cast<float>(winH) : 1.f;
        const glm::mat4 VP  = cam.proj(aspect) * cam.view();
        const glm::mat4 M   = glm::mat4(1.f);  // droplet lives in world space
        const glm::mat4 MVP = VP * M;
        const glm::mat3 N3  = glm::mat3(glm::transpose(glm::inverse(M)));
        const glm::vec3 eye = cam.eye();

        // Plane grid
        glUseProgram(flatProg);
        glUniformMatrix4fv(locFlatMVP, 1, GL_FALSE, glm::value_ptr(MVP));
        glUniform4f(locFlatColor, 0.45f, 0.50f, 0.55f, 1.f);
        planeGpu.draw();

        // Droplets
        for (const auto& dropPtr : scene.droplets()) {
            meshGpu.upload(*dropPtr);

            // Solid pass — polygon offset pushes faces back so wireframe sits on top.
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.f, 1.f);
            glUseProgram(meshProg);
            glUniformMatrix4fv(locMeshMVP,   1, GL_FALSE, glm::value_ptr(MVP));
            glUniformMatrix3fv(locMeshNorm,  1, GL_FALSE, glm::value_ptr(N3));
            glUniform3fv(locMeshCam,  1, glm::value_ptr(eye));
            glUniform3f(locMeshColor, 0.28f, 0.58f, 0.92f);
            meshGpu.drawSolid();
            glDisable(GL_POLYGON_OFFSET_FILL);

            // Wireframe overlay
            glUseProgram(flatProg);
            glUniformMatrix4fv(locFlatMVP, 1, GL_FALSE, glm::value_ptr(MVP));
            glUniform4f(locFlatColor, 0.05f, 0.05f, 0.10f, 1.f);
            meshGpu.drawWire();
        }

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    // ── 8. Cleanup ────────────────────────────────────────────────────────────
    meshGpu.destroy();
    planeGpu.destroy();
    glDeleteProgram(meshProg);
    glDeleteProgram(flatProg);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
