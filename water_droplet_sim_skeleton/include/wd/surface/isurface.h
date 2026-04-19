#pragma once
#include "wd/core/types.h"

namespace wd {

// Abstract interface for any support surface a droplet can sit on or slide
// along (flat plane, vertical glass, height field, etc.). The solver only
// depends on this interface, so swapping surface types requires no changes to
// the simulation code.
class ISurface {
public:
    virtual ~ISurface() = default;

    // Return the closest point on the surface to worldPoint, together with
    // its normal and tangent frame. Used by collision projection and by the
    // contact-line operator when it needs full local surface information.
    virtual SurfaceSample closestSample(const Vec3& worldPoint) const = 0;

    // Project worldPoint onto the surface and return the projected position.
    // Lighter-weight than closestSample when only the point is needed (e.g.
    // pushing a vertex that has penetrated the surface back onto it).
    virtual Vec3 projectPoint(const Vec3& worldPoint) const = 0;

    // Return the outward unit normal at (or nearest to) worldPoint. Used to
    // decompose forces into tangential and normal components.
    virtual Vec3 normalAt(const Vec3& worldPoint) const = 0;

    // Intersect a ray with the surface. On hit, fill outHit with position,
    // normal, and distance and return true; otherwise return false. Used by
    // mouse picking to convert a screen ray into a world-space contact point.
    virtual bool raycast(const Ray& ray, PickHit& outHit) const = 0;

    // Return an axis-aligned bounding box that contains the surface (or the
    // relevant simulation region of it). Used for culling and broad-phase
    // spatial queries.
    virtual AABB bounds() const = 0;
};

} // namespace wd
