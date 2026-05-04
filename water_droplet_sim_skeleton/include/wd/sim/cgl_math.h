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

// Convert two triangle meshes (vertices+faces) into a boolean union using CGAL.
// Returns true on success and fills outX/outF with the resulting mesh.
bool booleanUnionMeshes(
        const MatX3d& A_X,
        const MatX3i& A_F,
        const MatX3d& B_X,
        const MatX3i& B_F,
        MatX3d& outX,
        MatX3i& outF);
} // namespace wd
