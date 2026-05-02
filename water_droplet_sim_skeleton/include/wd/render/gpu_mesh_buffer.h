#pragma once
#include "wd/core/types.h"

namespace wd {

// Thin wrapper around the GPU-side mesh data for a single droplet. Owns the
// OpenGL buffer handles and provides upload / draw entry points so the
// renderer never touches raw GL state.
class GpuMeshBuffer {
public:
    GpuMeshBuffer() = default;
    ~GpuMeshBuffer();

    // Do not allow copy or assign
    GpuMeshBuffer(const GpuMeshBuffer&) = delete;
    GpuMeshBuffer& operator=(const GpuMeshBuffer&) = delete;
    GpuMeshBuffer(GpuMeshBuffer&& other) noexcept;
    GpuMeshBuffer& operator=(GpuMeshBuffer&& other) noexcept;

    // Allocate GL objects and upload the (static) triangle index buffer.
    // Called once per droplet when it is first spawned.
    void initialize(const MatX3i& faces);

    // Re-upload per-vertex positions and normals for the current frame.
    // Called every frame after the CPU solver finishes updating the droplet.
    void updateVertices(const MatX3d& positions, const MatX3d& normals, const Eigen::VectorXd& debugValues);

    // Issue a glDrawElements call to rasterize this droplet's triangles.
    void draw() const;

private:
    void release();

    unsigned int vao_ = 0;     // Vertex Array Object: describes the vertex attribute layout.
    unsigned int vboPos_ = 0;  // Vertex Buffer Object: per-vertex positions on the GPU.
    unsigned int vboNrm_ = 0;  // Vertex Buffer Object: per-vertex normals on the GPU.
    unsigned int vboDbg_ = 0;  // Vertex Buffer Object: per-vertex debug scalar.
    unsigned int ebo_ = 0;     // Element Buffer Object: triangle vertex indices.
    int indexCount_ = 0;       // Number of indices to draw (triangle count * 3).
};

} // namespace wd
