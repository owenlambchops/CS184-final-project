#include "wd/render/gpu_mesh_buffer.h"

namespace wd {

void GpuMeshBuffer::initialize(const MatX3i& faces) {
    indexCount_ = static_cast<int>(faces.rows() * 3);
    // TODO: create VAO/VBO/EBO and upload triangle indices.
}

void GpuMeshBuffer::updateVertices(const MatX3d&, const MatX3d&) {
    // TODO: upload positions and normals.
}

void GpuMeshBuffer::draw() const {
    // TODO: glBindVertexArray(vao_); glDrawElements(...);
}

} // namespace wd
