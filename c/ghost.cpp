#include "internal.h"

/*
 * A ghost is a collision object that only watches: no contact response,
 * so bodies pass through it, and a pair cache of its own so asking what
 * it overlaps costs nothing.
 */
eco_bt_ghost *eco_bt_ghost_create(eco_bt_shape *shape, const eco_bt_transform *at)
{
    eco_bt_ghost *g = new (std::nothrow) eco_bt_ghost();
    if (g == nullptr) {
        return nullptr;
    }

    g->ghost = new btPairCachingGhostObject();
    g->ghost->setCollisionShape(shape->shape);
    g->ghost->setWorldTransform(to_bt(at));
    g->ghost->setCollisionFlags(g->ghost->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    tag(g->ghost, &g->obj, ECO_BT_GHOST);
    return g;
}

void eco_bt_ghost_destroy(eco_bt_ghost *g)
{
    if (g == nullptr) {
        return;
    }

    delete g->ghost;
    delete g;
}

uint64_t eco_bt_ghost_id(eco_bt_ghost *g)
{
    return g->obj.id;
}

void eco_bt_ghost_get_transform(eco_bt_ghost *g, eco_bt_transform *out)
{
    store(out, g->ghost->getWorldTransform());
}

void eco_bt_ghost_set_transform(eco_bt_ghost *g, const eco_bt_transform *at)
{
    g->ghost->setWorldTransform(to_bt(at));
}

uint32_t eco_bt_ghost_in_world(eco_bt_ghost *g)
{
    return g->ghost->getBroadphaseHandle() != nullptr ? 1u : 0u;
}

size_t eco_bt_ghost_overlaps(eco_bt_ghost *g)
{
    g->ids.clear();

    for (int i = 0; i < g->ghost->getNumOverlappingObjects(); i++) {
        uint64_t id = id_of(g->ghost->getOverlappingObject(i));
        if (id != 0) {
            g->ids.push_back(id);
        }
    }

    return g->ids.size();
}

void eco_bt_ghost_overlaps_copy(eco_bt_ghost *g, uint64_t *out, size_t count)
{
    copy_out(g->ids, out, count);
}

/*
 * The ghost's own cache only knows the broadphase pairs. The narrowphase
 * for each lives in the world's pair cache, where the dispatcher left
 * its manifolds on the last step.
 */
size_t eco_bt_ghost_touching(eco_bt_ghost *g, eco_bt_world *w)
{
    g->ids.clear();

    btBroadphasePairArray &pairs = g->ghost->getOverlappingPairCache()->getOverlappingPairArray();

    for (int i = 0; i < pairs.size(); i++) {
        btBroadphasePair *pair = w->world->getPairCache()->findPair(pairs[i].m_pProxy0, pairs[i].m_pProxy1);
        if (pair == nullptr || pair->m_algorithm == nullptr) {
            continue;
        }

        g->manifolds.clear();
        pair->m_algorithm->getAllContactManifolds(g->manifolds);

        // one pair is one other object, however many manifolds it has
        for (int j = 0; j < g->manifolds.size(); j++) {
            btPersistentManifold *m = g->manifolds[j];
            const btCollisionObject *other = m->getBody0() == g->ghost ? m->getBody1() : m->getBody0();

            if (is_body(other) && touches(m)) {
                g->ids.push_back(id_of(other));
                break;
            }
        }
    }

    return g->ids.size();
}
