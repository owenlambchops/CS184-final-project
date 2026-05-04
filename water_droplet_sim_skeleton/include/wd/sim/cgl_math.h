#pragma once

#include "wd/core/types.h"
#include <utility>

namespace wd {

using Face = Eigen::Vector3i;

std::vector<Vec3> toVec3List(const MatX3d& X);
std::vector<Face> toFaceList(const MatX3i& F);
std::vector<std::vector<int>> buildNeighbours(const MatX3i& F, int nVertices);
std::vector<Eigen::Vector2i> buildUniqueEdgesFromFaces(const MatX3i& F);
double computeMeanEdgeLengthFromFaces(const MatX3d& X, const MatX3i& F);

Weights computeCotangentWeightsCgl(const std::vector<Vec3>& x, const std::vector<Face>& faces);
std::pair<std::vector<Vec3>, double> computeVertexNormalsAndVolumeCgl(
        const std::vector<Vec3>& x,
        const std::vector<Face>& faces);
void computeLaplaciansCgl(
        const std::vector<Vec3>& x,
        const std::vector<Vec3>& v,
        const std::vector<std::vector<int>>& neighbours,
        const Weights& w,
        std::vector<Vec3>& deltaX,
        std::vector<Vec3>& deltaV);

} // namespace wd
