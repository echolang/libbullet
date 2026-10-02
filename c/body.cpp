#include "internal.h"

eco_bt_body *eco_bt_body_create(eco_bt_shape *shape, float mass, const eco_bt_transform *at)
{
    eco_bt_body *b = new (std::nothrow) eco_bt_body();
    if (b == nullptr) {
        return nullptr;
    }

    b->motion = new btDefaultMotionState(to_bt(at));
    btRigidBody::btRigidBodyConstructionInfo info(mass, b->motion, shape->shape, inertia_of(shape->shape, mass));
    b->body = new btRigidBody(info);
    tag(b->body, &b->obj, ECO_BT_BODY);
    return b;
}

void eco_bt_body_destroy(eco_bt_body *b)
{
    if (b == nullptr) {
        return;
    }

    delete b->body;
    delete b->motion;
    delete b;
}

/*
 * The motion state, not the body: it is the interpolated transform when
 * the world steps with sub steps, which is what a renderer wants.
 */
void eco_bt_body_get_transform(eco_bt_body *b, eco_bt_transform *out)
{
    btTransform t;
    b->motion->getWorldTransform(t);
    store(out, t);
}

void eco_bt_body_set_transform(eco_bt_body *b, const eco_bt_transform *at)
{
    btTransform t = to_bt(at);
    b->body->setWorldTransform(t);
    b->body->setInterpolationWorldTransform(t);
    b->motion->setWorldTransform(t);
    b->body->activate(true);
}

/*
 * A kinematic body reads its motion state every step and derives its
 * velocity from how far that moved, so this is how you drive one.
 */
void eco_bt_body_move(eco_bt_body *b, const eco_bt_transform *at)
{
    b->motion->setWorldTransform(to_bt(at));
    b->body->activate(true);
}

void eco_bt_body_set_slot(eco_bt_body *b, int32_t slot)
{
    b->body->setUserIndex(slot);
}

int32_t eco_bt_body_slot(eco_bt_body *b)
{
    return b->body->getUserIndex();
}

void eco_bt_body_set_debug_draw(eco_bt_body *b, uint32_t on)
{
    int flags = b->body->getCollisionFlags();
    if (on != 0) {
        flags &= ~btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT;
    } else {
        flags |= btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT;
    }

    b->body->setCollisionFlags(flags);
}

uint64_t eco_bt_body_id(eco_bt_body *b)
{
    return b->obj.id;
}

void eco_bt_body_get_velocity(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getLinearVelocity());
}

void eco_bt_body_set_velocity(eco_bt_body *b, const eco_bt_vec3 *v)
{
    b->body->setLinearVelocity(to_bt(v));
    b->body->activate(true);
}

void eco_bt_body_get_angular_velocity(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getAngularVelocity());
}

void eco_bt_body_set_angular_velocity(eco_bt_body *b, const eco_bt_vec3 *v)
{
    b->body->setAngularVelocity(to_bt(v));
    b->body->activate(true);
}

float eco_bt_body_get_mass(eco_bt_body *b)
{
    btScalar inv = b->body->getInvMass();
    if (inv == btScalar(0)) {
        return 0.0f;
    }

    return btScalar(1) / inv;
}

void eco_bt_body_set_mass(eco_bt_body *b, float mass)
{
    b->body->setMassProps(mass, inertia_of(b->body->getCollisionShape(), mass));
    b->body->updateInertiaTensor();
    b->body->activate(true);
}

float eco_bt_body_inverse_mass(eco_bt_body *b)
{
    return b->body->getInvMass();
}

void eco_bt_body_local_inertia(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getLocalInertia());
}

void eco_bt_body_inverse_inertia(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getInvInertiaDiagLocal());
}

void eco_bt_body_velocity_at(eco_bt_body *b, const eco_bt_vec3 *rel, eco_bt_vec3 *out)
{
    store(out, b->body->getVelocityInLocalPoint(to_bt(rel)));
}

float eco_bt_body_get_friction(eco_bt_body *b)
{
    return b->body->getFriction();
}

void eco_bt_body_set_friction(eco_bt_body *b, float friction)
{
    b->body->setFriction(friction);
}

void eco_bt_body_set_anisotropic_friction(eco_bt_body *b, const eco_bt_vec3 *friction)
{
    b->body->setAnisotropicFriction(to_bt(friction));
}

float eco_bt_body_get_rolling_friction(eco_bt_body *b)
{
    return b->body->getRollingFriction();
}

void eco_bt_body_set_rolling_friction(eco_bt_body *b, float friction)
{
    b->body->setRollingFriction(friction);
}

float eco_bt_body_get_spinning_friction(eco_bt_body *b)
{
    return b->body->getSpinningFriction();
}

void eco_bt_body_set_spinning_friction(eco_bt_body *b, float friction)
{
    b->body->setSpinningFriction(friction);
}

float eco_bt_body_get_restitution(eco_bt_body *b)
{
    return b->body->getRestitution();
}

void eco_bt_body_set_restitution(eco_bt_body *b, float restitution)
{
    b->body->setRestitution(restitution);
}

void eco_bt_body_set_damping(eco_bt_body *b, float linear, float angular)
{
    b->body->setDamping(linear, angular);
}

float eco_bt_body_linear_damping(eco_bt_body *b)
{
    return b->body->getLinearDamping();
}

float eco_bt_body_angular_damping(eco_bt_body *b)
{
    return b->body->getAngularDamping();
}

void eco_bt_body_set_contact_stiffness(eco_bt_body *b, float stiffness, float damping)
{
    b->body->setContactStiffnessAndDamping(stiffness, damping);
}

/*
 * A sleeping body ignores forces, so every push wakes it first.
 */
void eco_bt_body_apply_force(eco_bt_body *b, const eco_bt_vec3 *force, const eco_bt_vec3 *rel)
{
    b->body->activate(true);
    b->body->applyForce(to_bt(force), to_bt(rel));
}

void eco_bt_body_apply_central_force(eco_bt_body *b, const eco_bt_vec3 *force)
{
    b->body->activate(true);
    b->body->applyCentralForce(to_bt(force));
}

void eco_bt_body_apply_impulse(eco_bt_body *b, const eco_bt_vec3 *impulse, const eco_bt_vec3 *rel)
{
    b->body->activate(true);
    b->body->applyImpulse(to_bt(impulse), to_bt(rel));
}

void eco_bt_body_apply_central_impulse(eco_bt_body *b, const eco_bt_vec3 *impulse)
{
    b->body->activate(true);
    b->body->applyCentralImpulse(to_bt(impulse));
}

void eco_bt_body_apply_torque(eco_bt_body *b, const eco_bt_vec3 *torque)
{
    b->body->activate(true);
    b->body->applyTorque(to_bt(torque));
}

void eco_bt_body_apply_torque_impulse(eco_bt_body *b, const eco_bt_vec3 *torque)
{
    b->body->activate(true);
    b->body->applyTorqueImpulse(to_bt(torque));
}

/*
 * Pushes that leave the sleep state alone: a body that is asleep stays
 * asleep (and the push only shows once something wakes it), and one that
 * is awake keeps its sleep timer, so a body held still by small pushes
 * every step can still fall asleep. A suspension spring is the case.
 */
void eco_bt_body_push(eco_bt_body *b, const eco_bt_vec3 *impulse, const eco_bt_vec3 *rel)
{
    b->body->applyImpulse(to_bt(impulse), to_bt(rel));
}

void eco_bt_body_push_central(eco_bt_body *b, const eco_bt_vec3 *impulse)
{
    b->body->applyCentralImpulse(to_bt(impulse));
}

void eco_bt_body_push_torque(eco_bt_body *b, const eco_bt_vec3 *torque)
{
    b->body->applyTorqueImpulse(to_bt(torque));
}

void eco_bt_body_total_force(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getTotalForce());
}

void eco_bt_body_total_torque(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getTotalTorque());
}

void eco_bt_body_clear_forces(eco_bt_body *b)
{
    b->body->clearForces();
}

void eco_bt_body_get_gravity(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getGravity());
}

/*
 * Own gravity sticks: the flag keeps the world from overwriting it on
 * add and on every World::setGravity.
 */
void eco_bt_body_set_gravity(eco_bt_body *b, const eco_bt_vec3 *gravity)
{
    b->body->setFlags(b->body->getFlags() | BT_DISABLE_WORLD_GRAVITY);
    b->body->setGravity(to_bt(gravity));
    b->body->activate(true);
}

void eco_bt_body_get_linear_factor(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getLinearFactor());
}

void eco_bt_body_set_linear_factor(eco_bt_body *b, const eco_bt_vec3 *factor)
{
    b->body->setLinearFactor(to_bt(factor));
}

void eco_bt_body_get_angular_factor(eco_bt_body *b, eco_bt_vec3 *out)
{
    store(out, b->body->getAngularFactor());
}

void eco_bt_body_set_angular_factor(eco_bt_body *b, const eco_bt_vec3 *factor)
{
    b->body->setAngularFactor(to_bt(factor));
}

void eco_bt_body_set_ccd(eco_bt_body *b, float threshold, float radius)
{
    b->body->setCcdMotionThreshold(threshold);
    b->body->setCcdSweptSphereRadius(radius);
}

void eco_bt_body_set_sleep_thresholds(eco_bt_body *b, float linear, float angular)
{
    b->body->setSleepingThresholds(linear, angular);
}

void eco_bt_body_aabb(eco_bt_body *b, eco_bt_aabb *out)
{
    btVector3 min;
    btVector3 max;
    b->body->getAabb(min, max);
    store(out, min, max);
}

void eco_bt_body_activate(eco_bt_body *b)
{
    b->body->activate(true);
}

void eco_bt_body_sleep(eco_bt_body *b)
{
    b->body->forceActivationState(ISLAND_SLEEPING);
}

void eco_bt_body_set_always_awake(eco_bt_body *b, uint32_t on)
{
    if (on) {
        b->body->setActivationState(DISABLE_DEACTIVATION);
        return;
    }

    b->body->forceActivationState(ACTIVE_TAG);
}

uint32_t eco_bt_body_awake(eco_bt_body *b)
{
    return b->body->isActive() ? 1u : 0u;
}

uint32_t eco_bt_body_in_world(eco_bt_body *b)
{
    return b->body->isInWorld() ? 1u : 0u;
}

int32_t eco_bt_body_kind(eco_bt_body *b)
{
    if (b->body->isKinematicObject()) {
        return 2;
    }

    if (b->body->isStaticObject()) {
        return 1;
    }

    return 0;
}

/*
 * Kinematic is Bullet's static with a flag: mass zero, moved by hand.
 * It sleeps once it has stood still for the sleep time, like any body,
 * and each move wakes it; an awake kinematic body wakes everything it
 * touches every step, so one that never slept would hold every pile it
 * leans on awake. Turning it off leaves a static body; give it a mass
 * to make it dynamic again. Either way, re-add it to its world so
 * Bullet files it under the right broadphase group.
 */
void eco_bt_body_set_kinematic(eco_bt_body *b, uint32_t on)
{
    if (on) {
        // setMassProps(0) marks the body static; kinematic goes on top
        b->body->setMassProps(0, btVector3(0, 0, 0));
        b->body->setLinearVelocity(btVector3(0, 0, 0));
        b->body->setAngularVelocity(btVector3(0, 0, 0));
        b->body->setCollisionFlags(b->body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        b->body->activate(true);
        return;
    }

    b->body->setCollisionFlags(b->body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
    b->body->forceActivationState(ACTIVE_TAG);
}

uint32_t eco_bt_body_sensor(eco_bt_body *b)
{
    return b->body->hasContactResponse() ? 0u : 1u;
}

void eco_bt_body_set_sensor(eco_bt_body *b, uint32_t on)
{
    int flags = b->body->getCollisionFlags();

    if (on) {
        b->body->setCollisionFlags(flags | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    } else {
        b->body->setCollisionFlags(flags & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
    }
}
