#include "internal.h"

#include <algorithm>
#include <atomic>

int32_t eco_bt_version(void)
{
    return btGetVersion();
}

/*
 * One task scheduler for the process: Bullet keeps it in a global and
 * every parallel world steps through it. Built by the first parallel
 * world, torn down by eco_bt_shutdown.
 */
static btITaskScheduler *g_scheduler = nullptr;

int32_t eco_bt_max_threads(void)
{
    if (g_scheduler == nullptr) {
        g_scheduler = btCreateDefaultTaskScheduler();
        if (g_scheduler == nullptr) {
            return 1;
        }
    }

    return g_scheduler->getMaxNumThreads();
}

void eco_bt_shutdown(void)
{
    if (g_scheduler == nullptr) {
        return;
    }

    btSetTaskScheduler(btGetSequentialTaskScheduler());
    delete g_scheduler;
    g_scheduler = nullptr;
}

void eco_bt_set_sleep_seconds(float seconds)
{
    gDeactivationTime = seconds;
}

float eco_bt_sleep_seconds(void)
{
    return gDeactivationTime;
}

uint64_t eco_bt_next_id()
{
    static std::atomic<uint64_t> next(1);
    return next.fetch_add(1);
}

/*
 * The contact tracker, run after every substep while tracking is on. A
 * pair of bodies begins touching once one of its points is at or below
 * zero distance, and stays touching for as long as its manifold keeps any
 * point at all. That gap is Bullet's contact breaking threshold, and it
 * keeps a resting pair from flickering between began and ended.
 *
 * `seen` and `touching` are both sorted, so one walk over each diffs
 * them. Began events come first, then ended ones, each in pair order.
 */
static void on_tick(btDynamicsWorld *world, btScalar dt)
{
    (void)dt;

    eco_bt_world *w = static_cast<eco_bt_world *>(world->getWorldUserInfo());
    if (w == nullptr || !w->tracking) {
        return;
    }

    // 1: has points, 2: has a point at or below zero distance
    w->seen.clear();
    int manifolds = w->dispatcher->getNumManifolds();

    for (int i = 0; i < manifolds; i++) {
        btPersistentManifold *m = w->dispatcher->getManifoldByIndexInternal(i);
        if (m->getNumContacts() == 0 || !is_body(m->getBody0()) || !is_body(m->getBody1())) {
            continue;
        }

        int state = touches(m) ? 2 : 1;
        w->seen.push_back(std::make_pair(pair_of(id_of(m->getBody0()), id_of(m->getBody1())), state));
    }

    std::sort(w->seen.begin(), w->seen.end());
    w->next.clear();

    size_t t = 0;
    for (size_t i = 0; i < w->seen.size();) {
        std::pair<uint64_t, uint64_t> pair = w->seen[i].first;
        int state = w->seen[i].second;
        for (i++; i < w->seen.size() && w->seen[i].first == pair; i++) {
            state = std::max(state, w->seen[i].second);
        }

        while (t < w->touching.size() && w->touching[t] < pair) {
            t++;
        }

        bool was = t < w->touching.size() && w->touching[t] == pair;

        if (was || state == 2) {
            w->next.push_back(pair);
        }

        if (!was && state == 2) {
            w->events.push_back(eco_bt_event{pair.first, pair.second, 1});
        }
    }

    size_t n = 0;
    for (const auto &pair : w->touching) {
        while (n < w->next.size() && w->next[n] < pair) {
            n++;
        }

        if (n == w->next.size() || w->next[n] != pair) {
            w->events.push_back(eco_bt_event{pair.first, pair.second, 0});
        }
    }

    w->touching.swap(w->next);
}

/*
 * Stop tracking every pair `id` is in, queuing an ended event for each.
 * Only bodies are ever tracked.
 */
static void forget(eco_bt_world *w, uint64_t id)
{
    auto gone = [id](const std::pair<uint64_t, uint64_t> &pair) {
        return pair.first == id || pair.second == id;
    };

    for (const auto &pair : w->touching) {
        if (gone(pair)) {
            w->events.push_back(eco_bt_event{pair.first, pair.second, 0});
        }
    }

    w->touching.erase(std::remove_if(w->touching.begin(), w->touching.end(), gone), w->touching.end());
}

/*
 * The soft body collider caches distance fields per shape pointer. Drop a
 * shape's cells when it leaves the world, so a later shape allocated at
 * the same address does not inherit them.
 */
static void forget_shape(eco_bt_world *w, btCollisionShape *shape)
{
    // without soft bodies the cache is empty, and walking it is not free
    if (w->soft == nullptr || w->soft->getWorldInfo().m_sparsesdf.ncells == 0) {
        return;
    }

    if (shape->isCompound()) {
        btCompoundShape *compound = static_cast<btCompoundShape *>(shape);
        for (int i = 0; i < compound->getNumChildShapes(); i++) {
            forget_shape(w, compound->getChildShape(i));
        }
    }

    w->soft->getWorldInfo().m_sparsesdf.RemoveReferences(shape);
}

eco_bt_world *eco_bt_world_create(const eco_bt_vec3 *gravity, int32_t solver)
{
    eco_bt_world *w = new (std::nothrow) eco_bt_world();
    if (w == nullptr) {
        return nullptr;
    }

    w->config = new btSoftBodyRigidBodyCollisionConfiguration();
    w->dispatcher = new btCollisionDispatcher(w->config);
    w->broadphase = new btDbvtBroadphase();
    w->ghosts = new btGhostPairCallback();
    w->broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(w->ghosts);
    btGImpactCollisionAlgorithm::registerAlgorithm(w->dispatcher);

    w->mlcp = nullptr;
    if (solver == 1) {
        w->solver = new btNNCGConstraintSolver();
    } else if (solver == 2) {
        w->mlcp = new btDantzigSolver();
        w->solver = new btMLCPSolver(w->mlcp);
    } else {
        w->solver = new btSequentialImpulseConstraintSolver();
    }

    w->large = nullptr;
    w->soft = new btSoftRigidDynamicsWorld(w->dispatcher, w->broadphase, w->solver, w->config);
    w->world = w->soft;
    w->world->setInternalTickCallback(on_tick, w, false);
    w->debug = nullptr;
    w->tracking = false;

    // MLCP solves each island as one system; batching islands defeats it
    if (w->mlcp != nullptr) {
        w->world->getSolverInfo().m_minimumSolverBatchSize = 1;
    }

    if (gravity != nullptr) {
        eco_bt_world_set_gravity(w, gravity);
    }

    return w;
}

/*
 * Bullet's multi-threaded world: collision pairs, island solving and
 * integration split across the process task scheduler, one sequential
 * impulse solver per thread. Rigid bodies only. The step still returns
 * when every thread is done, so everything around it is unchanged.
 */
eco_bt_world *eco_bt_world_create_parallel(const eco_bt_vec3 *gravity, int32_t threads)
{
    if (eco_bt_max_threads() < 2) {
        return nullptr;
    }

    eco_bt_world *w = new (std::nothrow) eco_bt_world();
    if (w == nullptr) {
        return nullptr;
    }

    btSetTaskScheduler(g_scheduler);
    g_scheduler->setNumThreads(threads);
    int32_t running = g_scheduler->getNumThreads();

    w->config = new btSoftBodyRigidBodyCollisionConfiguration();
    w->dispatcher = new btCollisionDispatcherMt(w->config);
    w->broadphase = new btDbvtBroadphase();
    w->ghosts = new btGhostPairCallback();
    w->broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(w->ghosts);
    btGImpactCollisionAlgorithm::registerAlgorithm(w->dispatcher);

    w->mlcp = nullptr;
    btConstraintSolverPoolMt *pool = new btConstraintSolverPoolMt(running);
    w->solver = pool;
    w->large = new btSequentialImpulseConstraintSolverMt();
    w->soft = nullptr;
    w->world = new btDiscreteDynamicsWorldMt(w->dispatcher, w->broadphase, pool, w->large, w->config);
    w->world->setInternalTickCallback(on_tick, w, false);
    w->debug = nullptr;
    w->tracking = false;

    if (gravity != nullptr) {
        eco_bt_world_set_gravity(w, gravity);
    }

    return w;
}

/*
 * Actions and constraints first, then collision objects, so nothing is
 * deleted while the world still points at it. Soft bodies get their own
 * world info back. Echo frees the objects themselves once its handles
 * drop.
 */
void eco_bt_world_destroy(eco_bt_world *w)
{
    if (w == nullptr) {
        return;
    }

    for (eco_bt_vehicle *v : w->vehicles) {
        w->world->removeAction(v->vehicle);
        v->raycaster.world = nullptr;
    }

    for (eco_bt_character *c : w->characters) {
        w->world->removeAction(c->controller);
    }

    for (int i = w->world->getNumConstraints() - 1; i >= 0; i--) {
        w->world->removeConstraint(w->world->getConstraint(i));
    }

    btCollisionObjectArray &objects = w->world->getCollisionObjectArray();
    for (int i = objects.size() - 1; i >= 0; i--) {
        btCollisionObject *object = objects[i];
        btRigidBody *body = btRigidBody::upcast(object);
        btSoftBody *soft = btSoftBody::upcast(object);

        if (body != nullptr) {
            w->world->removeRigidBody(body);
        } else if (soft != nullptr && w->soft != nullptr) {
            w->soft->removeSoftBody(soft);

            const eco_bt_object *obj = object_of(soft);
            if (obj != nullptr && obj->kind == ECO_BT_SOFT) {
                eco_bt_soft *s = reinterpret_cast<eco_bt_soft *>(const_cast<eco_bt_object *>(obj));
                soft->m_worldInfo = &s->info;
            }
        } else {
            w->world->removeCollisionObject(object);
        }
    }

    w->world->setDebugDrawer(nullptr);

    delete w->world;
    delete w->solver;
    delete w->large;
    delete w->mlcp;
    delete w->ghosts;
    delete w->broadphase;
    delete w->dispatcher;
    delete w->config;
    delete w->debug;
    delete w;
}

/*
 * The soft body world info keeps its own gravity, so both move together.
 */
void eco_bt_world_set_gravity(eco_bt_world *w, const eco_bt_vec3 *gravity)
{
    btVector3 g = to_bt(gravity);
    w->world->setGravity(g);
    if (w->soft != nullptr) {
        w->soft->getWorldInfo().m_gravity = g;
    }
}

void eco_bt_world_get_gravity(eco_bt_world *w, eco_bt_vec3 *out)
{
    store(out, w->world->getGravity());
}

int32_t eco_bt_world_step(eco_bt_world *w, float dt, int32_t max_sub_steps, float fixed_step)
{
    int32_t steps = w->world->stepSimulation(dt, max_sub_steps, fixed_step);

    // only soft bodies fill the distance field cache
    if (w->soft != nullptr && w->soft->getSoftBodyArray().size() != 0) {
        w->soft->getWorldInfo().m_sparsesdf.GarbageCollect();
    }

    return steps;
}

/*
 * maxSubSteps 0 is Bullet's variable step: one internal step of exactly
 * `dt`, with nothing left over to interpolate.
 */
void eco_bt_world_advance(eco_bt_world *w, float dt)
{
    w->world->stepSimulation(dt, 0, dt);

    if (w->soft != nullptr && w->soft->getSoftBodyArray().size() != 0) {
        w->soft->getWorldInfo().m_sparsesdf.GarbageCollect();
    }
}

/*
 * The world transform, not the motion state: a caller that steps with
 * advance interpolates on its own side. Kinematic bodies sit in the same
 * list but go where their owner puts them, so they are skipped.
 */
size_t eco_bt_world_moving(eco_bt_world *w, eco_bt_pose *out, size_t capacity)
{
    btAlignedObjectArray<btRigidBody *> &bodies = w->world->getNonStaticRigidBodies();
    size_t n = 0;

    for (int i = 0; i < bodies.size(); i++) {
        btRigidBody *body = bodies[i];
        if (body->isStaticOrKinematicObject() || !body->isActive()) {
            continue;
        }

        if (out != nullptr && n < capacity) {
            out[n].slot = body->getUserIndex();
            store(&out[n].at, body->getWorldTransform());
        }
        n++;
    }

    return n;
}

void eco_bt_world_reserve(eco_bt_world *w, int32_t count)
{
    if (count > 0) {
        w->world->getCollisionObjectArray().reserve(count);
    }
}

void eco_bt_world_refresh(eco_bt_world *w, eco_bt_body *body)
{
    w->world->updateSingleAabb(body->body);
}

void eco_bt_world_get_solver(eco_bt_world *w, eco_bt_solver *out)
{
    const btContactSolverInfo &info = w->world->getSolverInfo();
    out->iterations = info.m_numIterations;
    out->erp = info.m_erp;
    out->erp2 = info.m_erp2;
    out->cfm = info.m_globalCfm;
    out->friction_erp = info.m_frictionERP;
    out->warmstarting = info.m_warmstartingFactor;
    out->restitution_threshold = info.m_restitutionVelocityThreshold;
    out->split_threshold = info.m_splitImpulsePenetrationThreshold;
    out->split_impulse = info.m_splitImpulse ? 1u : 0u;
    out->min_batch = info.m_minimumSolverBatchSize;
}

void eco_bt_world_set_solver(eco_bt_world *w, const eco_bt_solver *s)
{
    btContactSolverInfo &info = w->world->getSolverInfo();
    info.m_numIterations = s->iterations;
    info.m_erp = s->erp;
    info.m_erp2 = s->erp2;
    info.m_globalCfm = s->cfm;
    info.m_frictionERP = s->friction_erp;
    info.m_warmstartingFactor = s->warmstarting;
    info.m_restitutionVelocityThreshold = s->restitution_threshold;
    info.m_splitImpulsePenetrationThreshold = s->split_threshold;
    info.m_splitImpulse = s->split_impulse != 0;
    info.m_minimumSolverBatchSize = s->min_batch;
}

void eco_bt_world_clear_forces(eco_bt_world *w)
{
    w->world->clearForces();
}

void eco_bt_world_set_update_all_aabbs(eco_bt_world *w, uint32_t on)
{
    w->world->setForceUpdateAllAabbs(on != 0);
}

void eco_bt_world_set_air_density(eco_bt_world *w, float density)
{
    if (w->soft != nullptr) {
        w->soft->getWorldInfo().air_density = density;
    }
}

void eco_bt_world_add_body(eco_bt_world *w, eco_bt_body *body)
{
    w->world->addRigidBody(body->body);
}

void eco_bt_world_add_body_filtered(eco_bt_world *w, eco_bt_body *body, int32_t group, int32_t mask)
{
    w->world->addRigidBody(body->body, group, mask);
}

void eco_bt_world_remove_body(eco_bt_world *w, eco_bt_body *body)
{
    w->world->removeRigidBody(body->body);
    forget_shape(w, body->body->getCollisionShape());
    forget(w, body->obj.id);
}

void eco_bt_world_add_constraint(eco_bt_world *w, eco_bt_constraint *constraint, uint32_t no_collide)
{
    w->world->addConstraint(constraint->constraint, no_collide != 0);
}

void eco_bt_world_remove_constraint(eco_bt_world *w, eco_bt_constraint *constraint)
{
    w->world->removeConstraint(constraint->constraint);
}

void eco_bt_world_add_ghost(eco_bt_world *w, eco_bt_ghost *ghost, int32_t group, int32_t mask)
{
    w->world->addCollisionObject(ghost->ghost, group, mask);
}

void eco_bt_world_remove_ghost(eco_bt_world *w, eco_bt_ghost *ghost)
{
    w->world->removeCollisionObject(ghost->ghost);
    forget_shape(w, ghost->ghost->getCollisionShape());
}

void eco_bt_world_add_character(eco_bt_world *w, eco_bt_character *c, int32_t group, int32_t mask)
{
    w->world->addCollisionObject(c->ghost, group, mask);
    w->world->addAction(c->controller);
    w->characters.push_back(c);
}

void eco_bt_world_remove_character(eco_bt_world *w, eco_bt_character *c)
{
    w->world->removeAction(c->controller);
    w->world->removeCollisionObject(c->ghost);
    forget_shape(w, c->ghost->getCollisionShape());
    w->characters.erase(std::remove(w->characters.begin(), w->characters.end(), c), w->characters.end());
}

/*
 * A vehicle's chassis must not fall asleep: a sleeping body ignores the
 * wheels' forces, and nothing else would wake it.
 */
void eco_bt_world_add_vehicle(eco_bt_world *w, eco_bt_vehicle *v)
{
    v->raycaster.world = w->world;
    v->vehicle->getRigidBody()->setActivationState(DISABLE_DEACTIVATION);
    w->world->addAction(v->vehicle);
    w->vehicles.push_back(v);
}

void eco_bt_world_remove_vehicle(eco_bt_world *w, eco_bt_vehicle *v)
{
    w->world->removeAction(v->vehicle);
    v->raycaster.world = nullptr;
    w->vehicles.erase(std::remove(w->vehicles.begin(), w->vehicles.end(), v), w->vehicles.end());
}

uint32_t eco_bt_world_add_soft(eco_bt_world *w, eco_bt_soft *s, int32_t group, int32_t mask)
{
    if (w->soft == nullptr) {
        return 0u;
    }

    s->soft->m_worldInfo = &w->soft->getWorldInfo();
    w->soft->addSoftBody(s->soft, group, mask);
    return 1u;
}

void eco_bt_world_remove_soft(eco_bt_world *w, eco_bt_soft *s)
{
    if (w->soft == nullptr) {
        return;
    }

    w->soft->removeSoftBody(s->soft);
    s->soft->m_worldInfo = &s->info;
}

void eco_bt_world_track_contacts(eco_bt_world *w, uint32_t on)
{
    w->tracking = on != 0;

    if (!w->tracking) {
        w->touching.clear();
        w->events.clear();
    }
}

size_t eco_bt_world_events(eco_bt_world *w)
{
    return w->events.size();
}

void eco_bt_world_events_take(eco_bt_world *w, eco_bt_event *out, size_t count)
{
    copy_out(w->events, out, count);
    w->events.clear();
}

void eco_bt_world_debug_mode(
    eco_bt_world *w,
    uint32_t wireframe,
    uint32_t aabb,
    uint32_t contacts,
    uint32_t constraints
)
{
    int mode = btIDebugDraw::DBG_NoDebug;

    if (wireframe) {
        mode |= btIDebugDraw::DBG_DrawWireframe;
    }

    if (aabb) {
        mode |= btIDebugDraw::DBG_DrawAabb;
    }

    if (contacts) {
        mode |= btIDebugDraw::DBG_DrawContactPoints;
    }

    if (constraints) {
        mode |= btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits;
    }

    if (mode == btIDebugDraw::DBG_NoDebug) {
        w->world->setDebugDrawer(nullptr);
        return;
    }

    if (w->debug == nullptr) {
        w->debug = new (std::nothrow) EcoDebugDraw();
        if (w->debug == nullptr) {
            return;
        }
    }

    w->debug->setDebugMode(mode);
    w->world->setDebugDrawer(w->debug);
}

size_t eco_bt_world_debug_draw(eco_bt_world *w)
{
    if (w->debug == nullptr || w->world->getDebugDrawer() == nullptr) {
        return 0;
    }

    w->debug->lines.clear();
    w->world->debugDrawWorld();
    return w->debug->lines.size();
}

void eco_bt_world_debug_copy(eco_bt_world *w, float *out, size_t count)
{
    if (w->debug == nullptr) {
        return;
    }

    copy_out(w->debug->lines, out, count);
}
