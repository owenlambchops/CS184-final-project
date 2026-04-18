#pragma once
#include "wd/core/types.h"

namespace wd {

class GpuMeshBuffer {
public:
    void initialize(const MatX3i& faces);
    void updateVertices(const MatX3d& positions, const MatX3d& normals);
    void draw() const;

private:
    unsigned int vao_ = 0;
    unsigned int vboPos_ = 0;
    unsigned int vboNrm_ = 0;
    unsigned int ebo_ = 0;
    int indexCount_ = 0;
};

} // namespace wd
