#include "wd/render/gpu_mesh_buffer.h"
#include <cassert>
#include <glad/glad.h>
#include <utility>

namespace wd {

namespace {
using GpuAttribMat = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
}

GpuMeshBuffer::~GpuMeshBuffer() {
    release();
}

GpuMeshBuffer::GpuMeshBuffer(GpuMeshBuffer&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vboPos_(std::exchange(other.vboPos_, 0)),
      vboNrm_(std::exchange(other.vboNrm_, 0)),
      vboDbg_(std::exchange(other.vboDbg_, 0)),
      ebo_(std::exchange(other.ebo_, 0)),
      indexCount_(std::exchange(other.indexCount_, 0)) {}

GpuMeshBuffer& GpuMeshBuffer::operator=(GpuMeshBuffer&& other) noexcept {
    if (this == &other) return *this;

    release();
    vao_ = std::exchange(other.vao_, 0);
    vboPos_ = std::exchange(other.vboPos_, 0);
    vboNrm_ = std::exchange(other.vboNrm_, 0);
    vboDbg_ = std::exchange(other.vboDbg_, 0);
    ebo_ = std::exchange(other.ebo_, 0);
    indexCount_ = std::exchange(other.indexCount_, 0);
    return *this;
}

void GpuMeshBuffer::initialize(const MatX3i& faces) {
    release();
    indexCount_ = static_cast<int>(faces.rows() * 3);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vboPos_);
    glGenBuffers(1, &vboNrm_);
    glGenBuffers(1, &vboDbg_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(faces.size() * sizeof(MatX3i::Scalar)),
                 faces.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vboNrm_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, vboDbg_);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void GpuMeshBuffer::updateVertices(const MatX3d& positions, const MatX3d& normals, const Eigen::VectorXd& debugValues) {
    assert(vao_ != 0);
    assert(positions.rows() == normals.rows());
    assert(debugValues.size() == positions.rows());

    const GpuAttribMat positionsGpu = positions.cast<float>();
    const GpuAttribMat normalsGpu = normals.cast<float>();

    glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(positionsGpu.size() * sizeof(GpuAttribMat::Scalar)),
                 positionsGpu.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboNrm_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(normalsGpu.size() * sizeof(GpuAttribMat::Scalar)),
                 normalsGpu.data(),
                 GL_DYNAMIC_DRAW);

    const Eigen::VectorXf debugGpu = debugValues.cast<float>();
    glBindBuffer(GL_ARRAY_BUFFER, vboDbg_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(debugGpu.size() * sizeof(Eigen::VectorXf::Scalar)),
                 debugGpu.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void GpuMeshBuffer::draw() const {
    if (vao_ == 0 || indexCount_ <= 0) return;

    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void GpuMeshBuffer::release() {
    if (ebo_ != 0) glDeleteBuffers(1, &ebo_);
    if (vboDbg_ != 0) glDeleteBuffers(1, &vboDbg_);
    if (vboNrm_ != 0) glDeleteBuffers(1, &vboNrm_);
    if (vboPos_ != 0) glDeleteBuffers(1, &vboPos_);
    if (vao_ != 0) glDeleteVertexArrays(1, &vao_);

    vao_ = 0;
    vboPos_ = 0;
    vboNrm_ = 0;
    vboDbg_ = 0;
    ebo_ = 0;
    indexCount_ = 0;
}

} // namespace wd
