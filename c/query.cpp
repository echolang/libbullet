#include "internal.h"

#include <algorithm>

/*
 * Every query here answers with bodies: ghosts, characters and soft
 * bodies never hit. The query's own group is "everything", so only the
 * caller's mask and each body's own mask decide.
 */

static bool is_body(const btBroadphaseProxy *proxy)
{
    return is_body(static_cast<const btCollisionObject *>(proxy->m_clientObject));
}

static bool admits(const btBroadphaseProxy *proxy, int32_t mask)
{
    return (proxy->m_collisionFilterGroup & mask) != 0 && is_body(proxy);
}

/*
 * One of Bullet's result callbacks, narrowed to bodies in `mask`. With
 * `skip` set, that one object never counts: a wheel ray cast from inside
 * its own chassis.
 */
template <typename Base>
struct BodiesOnly : public Base
{
    const btCollisionObject *skip = nullptr;

    template <typename... Args>
    explicit BodiesOnly(int32_t mask, Args &&...args) : Base(std::forward<Args>(args)...)
    {
        this->m_collisionFilterGroup = btBroadphaseProxy::AllFilter;
        this->m_collisionFilterMask = mask;
    }

    bool needsCollision(btBroadphaseProxy *proxy) const override
    {
        return proxy->m_clientObject != skip && Base::needsCollision(proxy) && is_body(proxy);
    }
};

static const btCollisionObject *skipped(const eco_bt_body *ignore)
{
    return ignore == nullptr ? nullptr : ignore->body;
}

typedef BodiesOnly<btCollisionWorld::ClosestRayResultCallback> ClosestBody;
typedef BodiesOnly<btCollisionWorld::AllHitsRayResultCallback> EveryBody;
typedef BodiesOnly<btCollisionWorld::ClosestConvexResultCallback> SweptBody;

/*
 * Collects contact points at or below zero distance, with `self` always
 * on the a side, flipping the ones Bullet hands over swapped. With `ids`
 * set, only the other side's id is kept, once.
 */
struct Contacts : public BodiesOnly<btCollisionWorld::ContactResultCallback>
{
    const btCollisionObject *self;
    std::vector<eco_bt_contact> *contacts;
    std::vector<uint64_t> *ids;

    Contacts(const btCollisionObject *self, int32_t mask) : BodiesOnly(mask), self(self), contacts(nullptr), ids(nullptr)
    {
    }

    btScalar addSingleResult(
        btManifoldPoint &cp,
        const btCollisionObjectWrapper *w0,
        int part0,
        int index0,
        const btCollisionObjectWrapper *w1,
        int part1,
        int index1
    ) override
    {
        (void)part0;
        (void)index0;
        (void)part1;
        (void)index1;

        // closest point algorithms also report near misses within the margins
        if (!touches(cp)) {
            return 0;
        }

        const btCollisionObject *o0 = w0->getCollisionObject();
        const btCollisionObject *o1 = w1->getCollisionObject();
        bool swapped = self != nullptr && o1 == self;

        // contactTest finishes one pair before the next, so a repeat is the last id
        if (ids != nullptr) {
            uint64_t other = id_of(swapped ? o0 : o1);
            if (other != 0 && (ids->empty() || ids->back() != other)) {
                ids->push_back(other);
            }
            return 0;
        }

        eco_bt_contact c = contact_of(o0, o1, cp);
        if (swapped) {
            flip(&c);
        }

        contacts->push_back(c);
        return 0;
    }
};

static void hit_of(
    eco_bt_hit *out,
    const btCollisionObject *object,
    const btVector3 &point,
    const btVector3 &normal,
    btScalar fraction
)
{
    out->id = id_of(object);
    store(&out->point, point);
    store(&out->normal, normal.normalized());
    out->fraction = fraction;
}

uint32_t eco_bt_world_ray(
    eco_bt_world *w,
    const eco_bt_vec3 *from,
    const eco_bt_vec3 *to,
    int32_t mask,
    const eco_bt_body *ignore,
    eco_bt_hit *hit
)
{
    btVector3 a = to_bt(from);
    btVector3 b = to_bt(to);
    ClosestBody result(mask, a, b);
    result.skip = skipped(ignore);

    w->world->rayTest(a, b, result);

    if (!result.hasHit()) {
        return 0;
    }

    hit_of(hit, result.m_collisionObject, result.m_hitPointWorld, result.m_hitNormalWorld,
        result.m_closestHitFraction);
    return 1;
}

size_t eco_bt_world_rays(eco_bt_world *w, const eco_bt_vec3 *from, const eco_bt_vec3 *to, int32_t mask)
{
    btVector3 a = to_bt(from);
    btVector3 b = to_bt(to);
    EveryBody result(mask, a, b);

    w->hits.clear();
    w->world->rayTest(a, b, result);

    for (int i = 0; i < result.m_collisionObjects.size(); i++) {
        eco_bt_hit h;
        hit_of(&h, result.m_collisionObjects[i], result.m_hitPointWorld[i], result.m_hitNormalWorld[i],
            result.m_hitFractions[i]);
        w->hits.push_back(h);
    }

    std::sort(w->hits.begin(), w->hits.end(), [](const eco_bt_hit &l, const eco_bt_hit &r) {
        return l.fraction < r.fraction;
    });

    return w->hits.size();
}

void eco_bt_world_hits_copy(eco_bt_world *w, eco_bt_hit *out, size_t count)
{
    copy_out(w->hits, out, count);
}

/*
 * A sweep that ignores surfaces it moves away from or along. A shape
 * that starts inside something reports that thing at fraction 0 with a
 * normal pointing out; skipping it lets the shape back out instead of
 * sticking.
 */
struct FacingSweep : public SweptBody
{
    btVector3 motion;
    bool facing;

    FacingSweep(int32_t mask, const btVector3 &from, const btVector3 &to, bool facing)
        : SweptBody(mask, from, to), motion(to - from), facing(facing)
    {
    }

    btScalar addSingleResult(btCollisionWorld::LocalConvexResult &r, bool normalInWorldSpace) override
    {
        btVector3 normal = normalInWorldSpace
            ? r.m_hitNormalLocal
            : r.m_hitCollisionObject->getWorldTransform().getBasis() * r.m_hitNormalLocal;
        if (facing && normal.dot(motion) >= btScalar(0)) {
            return m_closestHitFraction;
        }

        return SweptBody::addSingleResult(r, normalInWorldSpace);
    }
};

uint32_t eco_bt_world_sweep(
    eco_bt_world *w,
    eco_bt_shape *shape,
    const eco_bt_transform *from,
    const eco_bt_transform *to,
    int32_t mask,
    uint32_t facing,
    const eco_bt_body *ignore,
    eco_bt_hit *hit
)
{
    btTransform a = to_bt(from);
    btTransform b = to_bt(to);
    FacingSweep result(mask, a.getOrigin(), b.getOrigin(), facing != 0);
    result.skip = skipped(ignore);

    w->world->convexSweepTest(static_cast<btConvexShape *>(shape->shape), a, b, result);

    if (!result.hasHit()) {
        return 0;
    }

    hit_of(hit, result.m_hitCollisionObject, result.m_hitPointWorld, result.m_hitNormalWorld,
        result.m_closestHitFraction);
    return 1;
}

size_t eco_bt_world_overlap(eco_bt_world *w, eco_bt_shape *shape, const eco_bt_transform *at, int32_t mask)
{
    btCollisionObject probe;
    probe.setCollisionShape(shape->shape);
    probe.setWorldTransform(to_bt(at));

    Contacts result(&probe, mask);
    w->ids.clear();
    result.ids = &w->ids;

    w->world->contactTest(&probe, result);
    return w->ids.size();
}

struct InBox : public btBroadphaseAabbCallback
{
    int32_t mask;
    std::vector<uint64_t> *ids;

    bool process(const btBroadphaseProxy *proxy) override
    {
        if (admits(proxy, mask)) {
            ids->push_back(id_of(static_cast<const btCollisionObject *>(proxy->m_clientObject)));
        }

        return true;
    }
};

size_t eco_bt_world_aabb_test(eco_bt_world *w, const eco_bt_aabb *box, int32_t mask)
{
    InBox result;
    result.mask = mask;
    result.ids = &w->ids;

    w->ids.clear();
    w->broadphase->aabbTest(to_bt(&box->min), to_bt(&box->max), result);
    return w->ids.size();
}

void eco_bt_world_ids_copy(eco_bt_world *w, uint64_t *out, size_t count)
{
    copy_out(w->ids, out, count);
}

size_t eco_bt_world_body_contacts(eco_bt_world *w, eco_bt_body *body, int32_t mask)
{
    Contacts result(body->body, mask);
    w->contacts.clear();
    result.contacts = &w->contacts;

    w->world->contactTest(body->body, result);
    return w->contacts.size();
}

size_t eco_bt_world_pair_contacts(eco_bt_world *w, eco_bt_body *a, eco_bt_body *b)
{
    Contacts result(a->body, btBroadphaseProxy::AllFilter);
    w->contacts.clear();
    result.contacts = &w->contacts;

    w->world->contactPairTest(a->body, b->body, result);
    return w->contacts.size();
}

/*
 * What the last step left in the dispatcher: every point at or below
 * zero distance, between two bodies.
 */
size_t eco_bt_world_touching(eco_bt_world *w)
{
    w->contacts.clear();
    int manifolds = w->dispatcher->getNumManifolds();

    for (int i = 0; i < manifolds; i++) {
        btPersistentManifold *m = w->dispatcher->getManifoldByIndexInternal(i);
        const btCollisionObject *a = m->getBody0();
        const btCollisionObject *b = m->getBody1();

        if (!is_body(a) || !is_body(b)) {
            continue;
        }

        for (int j = 0; j < m->getNumContacts(); j++) {
            const btManifoldPoint &p = m->getContactPoint(j);
            if (touches(p)) {
                w->contacts.push_back(contact_of(a, b, p));
            }
        }
    }

    return w->contacts.size();
}

void eco_bt_world_contacts_copy(eco_bt_world *w, eco_bt_contact *out, size_t count)
{
    copy_out(w->contacts, out, count);
}
