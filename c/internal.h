#ifndef ECO_BT_INTERNAL_H
#define ECO_BT_INTERNAL_H

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "LinearMath/btThreads.h"
#include "bt.h"

#include <new>
#include <utility>
#include <vector>

/*
 * Collects debug lines instead of drawing them. x,y,z,r,g,b per vertex,
 * two vertices per line, cleared before every draw.
 */
class EcoDebugDraw : public btIDebugDraw
{
public:
    std::vector<float> lines;
    int mode = DBG_NoDebug;

    void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) override
    {
        vertex(from, color);
        vertex(to, color);
    }

    void drawContactPoint(
        const btVector3 &point,
        const btVector3 &normal,
        btScalar distance,
        int lifeTime,
        const btVector3 &color
    ) override
    {
        (void)distance;
        (void)lifeTime;
        drawLine(point, point + normal * btScalar(0.1), color);
    }

    void reportErrorWarning(const char *warning) override
    {
        (void)warning;
    }

    void draw3dText(const btVector3 &location, const char *text) override
    {
        (void)location;
        (void)text;
    }

    void setDebugMode(int m) override
    {
        mode = m;
    }

    int getDebugMode() const override
    {
        return mode;
    }

private:
    void vertex(const btVector3 &p, const btVector3 &c)
    {
        lines.push_back(p.x());
        lines.push_back(p.y());
        lines.push_back(p.z());
        lines.push_back(c.x());
        lines.push_back(c.y());
        lines.push_back(c.z());
    }
};

/*
 * What a collision object's user pointer points at. Every wrapper that
 * owns a collision object starts with one, so a hit or a manifold can be
 * traced back to its wrapper and its id. A user pointer is either NULL or
 * one of these.
 */
enum eco_bt_kind : uint32_t {
    ECO_BT_BODY = 1,
    ECO_BT_GHOST = 2,
    ECO_BT_CHARACTER = 3,
    ECO_BT_SOFT = 4,
};

struct eco_bt_object {
    uint64_t id;
    eco_bt_kind kind;
};

uint64_t eco_bt_next_id();

static inline void tag(btCollisionObject *o, eco_bt_object *obj, eco_bt_kind kind)
{
    obj->id = eco_bt_next_id();
    obj->kind = kind;
    o->setUserPointer(obj);
}

static inline const eco_bt_object *object_of(const btCollisionObject *o)
{
    return static_cast<const eco_bt_object *>(o->getUserPointer());
}

static inline uint64_t id_of(const btCollisionObject *o)
{
    const eco_bt_object *obj = object_of(o);
    return obj == nullptr ? 0 : obj->id;
}

static inline bool is_body(const btCollisionObject *o)
{
    const eco_bt_object *obj = object_of(o);
    return obj != nullptr && obj->kind == ECO_BT_BODY;
}

/*
 * The one rule for touching: a point at or below zero distance. Points
 * above it are the manifold's breaking threshold, a near miss.
 */
static inline bool touches(const btManifoldPoint &p)
{
    return p.getDistance() <= btScalar(0);
}

static inline bool touches(const btPersistentManifold *m)
{
    for (int i = 0; i < m->getNumContacts(); i++) {
        if (touches(m->getContactPoint(i))) {
            return true;
        }
    }

    return false;
}

/*
 * Everything a btSoftRigidDynamicsWorld needs, owned in one place. It is
 * a discrete dynamics world that can also carry soft bodies, and costs
 * nothing extra without them.
 *
 * Bodies, constraints, ghosts, characters and vehicles are not owned:
 * Echo holds those. The last two are listed so destroy can take their
 * actions out first and tell them they are no longer in a world.
 *
 * `hits`, `contacts` and `ids` hold the last query's result until Echo
 * copies it. `touching` and `events` are the contact tracker's state:
 * `touching` stays sorted, and `seen` and `next` are its scratch, kept
 * here so a step allocates nothing once they have grown.
 */
struct eco_bt_world {
    btSoftBodyRigidBodyCollisionConfiguration *config;
    btCollisionDispatcher *dispatcher;
    btDbvtBroadphase *broadphase;
    btGhostPairCallback *ghosts;
    btMLCPSolverInterface *mlcp;
    btConstraintSolver *solver;

    /*
     * A parallel world's solver for islands too large to hand one thread:
     * it batches their contacts and solves the batches across threads. A
     * pile is one island, so without it the pile's step runs on one core.
     * Null in a plain world.
     */
    btConstraintSolver *large;

    /*
     * The world every call steps and queries. `soft` is the same world
     * when it can hold soft bodies, and null for a parallel world:
     * Bullet's multi-threaded world is rigid only.
     */
    btDiscreteDynamicsWorld *world;
    btSoftRigidDynamicsWorld *soft;
    EcoDebugDraw *debug;
    std::vector<struct eco_bt_character *> characters;
    std::vector<struct eco_bt_vehicle *> vehicles;

    std::vector<eco_bt_hit> hits;
    std::vector<eco_bt_contact> contacts;
    std::vector<uint64_t> ids;

    bool tracking;
    std::vector<std::pair<uint64_t, uint64_t>> touching;
    std::vector<std::pair<uint64_t, uint64_t>> next;
    std::vector<std::pair<std::pair<uint64_t, uint64_t>, int>> seen;
    std::vector<eco_bt_event> events;
};

/*
 * The shape's own storage. Bullet keeps pointers into heights, vertices
 * and indices instead of copying, so they live exactly as long as the
 * shape. `mesh` is the index array a triangle mesh shape reads through.
 */
struct eco_bt_shape {
    btCollisionShape *shape;
    std::vector<float> heights;
    std::vector<eco_bt_vec3> vertices;
    std::vector<int32_t> indices;
    btTriangleIndexVertexArray *mesh;
};

struct eco_bt_body {
    eco_bt_object obj;
    btRigidBody *body;
    btDefaultMotionState *motion;
};

struct eco_bt_constraint {
    btTypedConstraint *constraint;
    btJointFeedback feedback;
};

/*
 * `ids` holds the last overlap or touching query, like the world's.
 * `manifolds` is scratch for the touching query.
 */
struct eco_bt_ghost {
    eco_bt_object obj;
    btPairCachingGhostObject *ghost;
    std::vector<uint64_t> ids;
    btManifoldArray manifolds;
};

struct eco_bt_character {
    eco_bt_object obj;
    btPairCachingGhostObject *ghost;
    btKinematicCharacterController *controller;
};

/*
 * btDefaultVehicleRaycaster with a world that can change: a vehicle is
 * built before it is added, and may move between worlds.
 */
class EcoVehicleRaycaster : public btVehicleRaycaster
{
public:
    btDynamicsWorld *world = nullptr;
    const btCollisionObject *chassis = nullptr;

    void *castRay(const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result) override;
};

struct eco_bt_vehicle {
    EcoVehicleRaycaster raycaster;
    btRaycastVehicle *vehicle;
};

/*
 * `info` is the soft body's own world info while it is in no world.
 * Adding it points the body at the world's instead, removing points it
 * back, so the body never outlives the info it reads.
 */
struct eco_bt_soft {
    eco_bt_object obj;
    btSoftBody *soft;
    btSoftBodyWorldInfo info;
};

static inline btVector3 to_bt(const eco_bt_vec3 *v)
{
    return btVector3(v->x, v->y, v->z);
}

static inline btQuaternion to_bt(const eco_bt_quat *q)
{
    return btQuaternion(q->x, q->y, q->z, q->w);
}

static inline btTransform to_bt(const eco_bt_transform *t)
{
    return btTransform(to_bt(&t->rotation), to_bt(&t->origin));
}

static inline std::vector<btVector3> to_bt(const eco_bt_vec3 *v, int32_t count)
{
    std::vector<btVector3> out;
    out.reserve(count > 0 ? (size_t)count : 0);
    for (int32_t i = 0; i < count; i++) {
        out.push_back(to_bt(&v[i]));
    }

    return out;
}

/*
 * What `shape` resists turning with at `mass`. Zero for a static body.
 */
static inline btVector3 inertia_of(const btCollisionShape *shape, btScalar mass)
{
    btVector3 inertia(0, 0, 0);
    if (mass != btScalar(0)) {
        shape->calculateLocalInertia(mass, inertia);
    }

    return inertia;
}

static inline void store(eco_bt_vec3 *out, const btVector3 &v)
{
    if (out == nullptr) {
        return;
    }

    out->x = v.x();
    out->y = v.y();
    out->z = v.z();
}

static inline void store(eco_bt_transform *out, const btTransform &t)
{
    if (out == nullptr) {
        return;
    }

    store(&out->origin, t.getOrigin());

    btQuaternion q = t.getRotation();
    out->rotation.x = q.x();
    out->rotation.y = q.y();
    out->rotation.z = q.z();
    out->rotation.w = q.w();
}

static inline void store(eco_bt_aabb *out, const btVector3 &min, const btVector3 &max)
{
    if (out == nullptr) {
        return;
    }

    store(&out->min, min);
    store(&out->max, max);
}

/*
 * One manifold point as the ABI carries it.
 */
static inline eco_bt_contact contact_of(
    const btCollisionObject *a,
    const btCollisionObject *b,
    const btManifoldPoint &p
)
{
    eco_bt_contact c;
    c.a = id_of(a);
    c.b = id_of(b);
    store(&c.point_a, p.getPositionWorldOnA());
    store(&c.point_b, p.getPositionWorldOnB());
    store(&c.normal, p.m_normalWorldOnB);
    c.distance = p.getDistance();
    c.impulse = p.getAppliedImpulse();
    return c;
}

/*
 * The same contact seen from b's side.
 */
static inline void flip(eco_bt_contact *c)
{
    std::swap(c->a, c->b);
    std::swap(c->point_a, c->point_b);
    c->normal.x = -c->normal.x;
    c->normal.y = -c->normal.y;
    c->normal.z = -c->normal.z;
}

/*
 * Copy up to `count` of `from` into `out`.
 */
template <typename T>
static inline void copy_out(const std::vector<T> &from, T *out, size_t count)
{
    if (out == nullptr) {
        return;
    }

    size_t n = from.size() < count ? from.size() : count;
    for (size_t i = 0; i < n; i++) {
        out[i] = from[i];
    }
}

/*
 * The contact tracker's view of the world: pairs of ids, smaller first.
 */
static inline std::pair<uint64_t, uint64_t> pair_of(uint64_t a, uint64_t b)
{
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

#endif
