#include "internal.h"

static eco_bt_shape *wrap(btCollisionShape *shape)
{
    eco_bt_shape *s = new (std::nothrow) eco_bt_shape();
    if (s == nullptr) {
        delete shape;
        return nullptr;
    }

    s->shape = shape;
    s->mesh = nullptr;
    return s;
}

eco_bt_shape *eco_bt_shape_empty(void)
{
    return wrap(new btEmptyShape());
}

eco_bt_shape *eco_bt_shape_sphere(float radius)
{
    return wrap(new btSphereShape(radius));
}

eco_bt_shape *eco_bt_shape_box(const eco_bt_vec3 *half_extents)
{
    return wrap(new btBoxShape(to_bt(half_extents)));
}

eco_bt_shape *eco_bt_shape_capsule(float radius, float height, int32_t axis)
{
    if (axis == 0) {
        return wrap(new btCapsuleShapeX(radius, height));
    }

    if (axis == 2) {
        return wrap(new btCapsuleShapeZ(radius, height));
    }

    return wrap(new btCapsuleShape(radius, height));
}

eco_bt_shape *eco_bt_shape_cylinder(const eco_bt_vec3 *half_extents, int32_t axis)
{
    btVector3 half = to_bt(half_extents);

    if (axis == 0) {
        return wrap(new btCylinderShapeX(half));
    }

    if (axis == 2) {
        return wrap(new btCylinderShapeZ(half));
    }

    return wrap(new btCylinderShape(half));
}

eco_bt_shape *eco_bt_shape_cone(float radius, float height, int32_t axis)
{
    if (axis == 0) {
        return wrap(new btConeShapeX(radius, height));
    }

    if (axis == 2) {
        return wrap(new btConeShapeZ(radius, height));
    }

    return wrap(new btConeShape(radius, height));
}

eco_bt_shape *eco_bt_shape_plane(const eco_bt_vec3 *normal, float constant)
{
    return wrap(new btStaticPlaneShape(to_bt(normal), constant));
}

/*
 * Bullet copies the points. Optimizing drops the interior ones, which is
 * what anyone building a hull from mesh vertices wants. Simplifying goes
 * further: btShapeHull rebuilds the hull from at most a few dozen of its
 * own support points, far cheaper to collide for a dense mesh. Those
 * support points carry the margin, so they are sampled with it off.
 *
 * A hull of support points lies inside the original. HullLibrary pads a
 * flat, collinear or single point cloud into a box instead of failing,
 * so a rebuild that reaches outside the original was made up, and the
 * optimized hull stays.
 */
eco_bt_shape *eco_bt_shape_hull(const eco_bt_vec3 *points, int32_t count, uint32_t simplify)
{
    btConvexHullShape *hull = new btConvexHullShape(
        reinterpret_cast<const btScalar *>(points), count, sizeof(eco_bt_vec3));
    hull->optimizeConvexHull();

    if (!simplify) {
        return wrap(hull);
    }

    btScalar margin = hull->getMargin();
    hull->setMargin(0);

    btShapeHull reduced(hull);
    if (!reduced.buildHull(0) || reduced.numVertices() == 0) {
        hull->setMargin(margin);
        return wrap(hull);
    }

    btConvexHullShape *small = new btConvexHullShape(
        reinterpret_cast<const btScalar *>(reduced.getVertexPointer()), reduced.numVertices(), sizeof(btVector3));
    small->setMargin(0);

    btTransform identity = btTransform::getIdentity();
    btVector3 min, max, smallMin, smallMax;
    hull->getAabb(identity, min, max);
    small->getAabb(identity, smallMin, smallMax);

    // how far the rebuild reaches past the original on each axis
    btScalar slack = (max - min).length() * btScalar(1e-4) + SIMD_EPSILON;
    btVector3 past = min - smallMin;
    past.setMax(smallMax - max);

    if (past.x() > slack || past.y() > slack || past.z() > slack) {
        delete small;
        hull->setMargin(margin);
        return wrap(hull);
    }

    small->setMargin(margin);
    delete hull;
    return wrap(small);
}

eco_bt_shape *eco_bt_shape_spheres(const eco_bt_vec3 *centres, const float *radii, int32_t count)
{
    std::vector<btVector3> at = to_bt(centres, count);
    std::vector<btScalar> r(radii, radii + (count > 0 ? count : 0));
    return wrap(new btMultiSphereShape(at.data(), r.data(), count));
}

/*
 * The vertex and index copies back a btTriangleIndexVertexArray, which
 * both mesh shapes read through for as long as they live.
 */
static eco_bt_shape *mesh_of(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
)
{
    eco_bt_shape *s = wrap(nullptr);
    if (s == nullptr) {
        return nullptr;
    }

    s->vertices.assign(vertices, vertices + vertex_count);
    s->indices.assign(indices, indices + index_count);
    s->mesh = new btTriangleIndexVertexArray(
        index_count / 3,
        s->indices.data(),
        3 * sizeof(int32_t),
        vertex_count,
        reinterpret_cast<btScalar *>(s->vertices.data()),
        sizeof(eco_bt_vec3));
    return s;
}

eco_bt_shape *eco_bt_shape_mesh(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
)
{
    eco_bt_shape *s = mesh_of(vertices, vertex_count, indices, index_count);
    if (s == nullptr) {
        return nullptr;
    }

    s->shape = new btBvhTriangleMeshShape(s->mesh, true);
    return s;
}

eco_bt_shape *eco_bt_shape_gimpact(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
)
{
    eco_bt_shape *s = mesh_of(vertices, vertex_count, indices, index_count);
    if (s == nullptr) {
        return nullptr;
    }

    btGImpactMeshShape *shape = new btGImpactMeshShape(s->mesh);
    shape->updateBound();
    s->shape = shape;
    return s;
}

/*
 * Children are borrowed: Echo keeps every child Shape alive for as long
 * as the compound.
 */
eco_bt_shape *eco_bt_shape_compound(void)
{
    return wrap(new btCompoundShape(true));
}

void eco_bt_shape_compound_add(eco_bt_shape *s, eco_bt_shape *child, const eco_bt_transform *at)
{
    static_cast<btCompoundShape *>(s->shape)->addChildShape(to_bt(at), child->shape);
}

void eco_bt_shape_compound_set(eco_bt_shape *s, int32_t index, const eco_bt_transform *at)
{
    static_cast<btCompoundShape *>(s->shape)->updateChildTransform(index, to_bt(at), true);
}

eco_bt_shape *eco_bt_shape_heightfield(
    int32_t width,
    int32_t length,
    const float *heights,
    float min_height,
    float max_height,
    int32_t axis,
    uint32_t flip_quad_edges
)
{
    eco_bt_shape *s = wrap(nullptr);
    if (s == nullptr) {
        return nullptr;
    }

    s->heights.assign(heights, heights + (size_t)width * (size_t)length);
    s->shape = new btHeightfieldTerrainShape(
        width, length, s->heights.data(), min_height, max_height, axis, flip_quad_edges != 0);
    return s;
}

eco_bt_shape *eco_bt_shape_heightfield_borrowed(
    int32_t width,
    int32_t length,
    const float *heights,
    float min_height,
    float max_height,
    int32_t axis,
    uint32_t flip_quad_edges,
    uint32_t accelerate
)
{
    eco_bt_shape *s = wrap(nullptr);
    if (s == nullptr) {
        return nullptr;
    }

    btHeightfieldTerrainShape *field = new btHeightfieldTerrainShape(
        width, length, heights, min_height, max_height, axis, flip_quad_edges != 0);
    if (accelerate != 0) {
        field->buildAccelerator();
    }

    s->shape = field;
    return s;
}

void eco_bt_shape_destroy(eco_bt_shape *s)
{
    if (s == nullptr) {
        return;
    }

    delete s->shape;
    delete s->mesh;
    delete s;
}

/*
 * GImpact only marks its boxes stale on a scaling or margin change, and
 * collides against the stale ones until someone rebuilds them. A refit
 * is not enough: each part's quantized BVH clamps new boxes to the range
 * it was first built over, so a grown mesh is rebuilt from scratch.
 */
static void refresh_bound(btCollisionShape *shape)
{
    if (shape->getShapeType() != GIMPACT_SHAPE_PROXYTYPE) {
        return;
    }

    btGImpactShapeInterface *gimpact = static_cast<btGImpactShapeInterface *>(shape);
    if (gimpact->getGImpactShapeType() == CONST_GIMPACT_TRIMESH_SHAPE) {
        btGImpactMeshShape *mesh = static_cast<btGImpactMeshShape *>(gimpact);
        for (int i = 0; i < mesh->getMeshPartCount(); i++) {
            btGImpactMeshShapePart *part = mesh->getMeshPart(i);
            // getBoxSet is const only; the part owns the set and is not
            part->lockChildShapes();
            const_cast<btGImpactBoxSet *>(part->getBoxSet())->buildSet();
            part->unlockChildShapes();
        }
    }

    gimpact->updateBound();
}

void eco_bt_shape_set_scaling(eco_bt_shape *s, const eco_bt_vec3 *scaling)
{
    s->shape->setLocalScaling(to_bt(scaling));
    refresh_bound(s->shape);
}

void eco_bt_shape_get_scaling(eco_bt_shape *s, eco_bt_vec3 *out)
{
    store(out, s->shape->getLocalScaling());
}

void eco_bt_shape_inertia(eco_bt_shape *s, float mass, eco_bt_vec3 *out)
{
    store(out, inertia_of(s->shape, mass));
}

float eco_bt_shape_get_margin(eco_bt_shape *s)
{
    return s->shape->getMargin();
}

void eco_bt_shape_set_margin(eco_bt_shape *s, float margin)
{
    s->shape->setMargin(margin);
    refresh_bound(s->shape);
}

void eco_bt_shape_aabb(eco_bt_shape *s, const eco_bt_transform *at, eco_bt_aabb *out)
{
    btVector3 min;
    btVector3 max;
    s->shape->getAabb(to_bt(at), min, max);
    store(out, min, max);
}

float eco_bt_shape_bounds(eco_bt_shape *s, eco_bt_vec3 *centre)
{
    btVector3 c;
    btScalar radius;
    s->shape->getBoundingSphere(c, radius);
    store(centre, c);
    return radius;
}

uint32_t eco_bt_shape_convex(eco_bt_shape *s)
{
    return s->shape->isConvex() ? 1u : 0u;
}

/*
 * Shapes Bullet can only collide while they stand still: a static plane,
 * a heightfield, a BVH triangle mesh, or a compound holding one.
 */
static bool static_only(const btCollisionShape *shape)
{
    switch (shape->getShapeType()) {
        case STATIC_PLANE_PROXYTYPE:
        case TERRAIN_SHAPE_PROXYTYPE:
        case TRIANGLE_MESH_SHAPE_PROXYTYPE:
        case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
            return true;
        default:
            break;
    }

    if (shape->isCompound()) {
        const btCompoundShape *compound = static_cast<const btCompoundShape *>(shape);
        for (int i = 0; i < compound->getNumChildShapes(); i++) {
            if (static_only(compound->getChildShape(i))) {
                return true;
            }
        }
    }

    return false;
}

uint32_t eco_bt_shape_static_only(eco_bt_shape *s)
{
    return static_only(s->shape) ? 1u : 0u;
}

int32_t eco_bt_shape_kind(eco_bt_shape *s)
{
    switch (s->shape->getShapeType()) {
        case SPHERE_SHAPE_PROXYTYPE:
            return 1;
        case BOX_SHAPE_PROXYTYPE:
            return 2;
        case CAPSULE_SHAPE_PROXYTYPE:
            return 3;
        case CYLINDER_SHAPE_PROXYTYPE:
            return 4;
        case CONE_SHAPE_PROXYTYPE:
            return 5;
        case STATIC_PLANE_PROXYTYPE:
            return 6;
        case CONVEX_HULL_SHAPE_PROXYTYPE:
            return 7;
        case MULTI_SPHERE_SHAPE_PROXYTYPE:
            return 8;
        case TERRAIN_SHAPE_PROXYTYPE:
            return 9;
        case TRIANGLE_MESH_SHAPE_PROXYTYPE:
            return 10;
        case GIMPACT_SHAPE_PROXYTYPE:
            return 11;
        case COMPOUND_SHAPE_PROXYTYPE:
            return 12;
        default:
            return 0;
    }
}
