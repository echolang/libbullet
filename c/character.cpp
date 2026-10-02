#include "internal.h"

/*
 * Bullet's kinematic character controller: a convex ghost it slides
 * along the world by sweeps, with steps, slopes and jumping. It is an
 * action, so the world drives it every step.
 *
 * The controller assumes shapes are built Z up, and turning its up vector
 * turns the ghost to match: a Y up capsule would end up lying on its
 * side. Shapes here are built the way they are meant to stand, so every
 * change of up puts the ghost's own rotation back.
 */
eco_bt_character *eco_bt_character_create(
    eco_bt_shape *shape,
    const eco_bt_transform *at,
    float step_height,
    const eco_bt_vec3 *up
)
{
    eco_bt_character *c = new (std::nothrow) eco_bt_character();
    if (c == nullptr) {
        return nullptr;
    }

    c->ghost = new btPairCachingGhostObject();
    c->ghost->setWorldTransform(to_bt(at));
    c->ghost->setCollisionShape(shape->shape);
    c->ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
    tag(c->ghost, &c->obj, ECO_BT_CHARACTER);

    c->controller = new btKinematicCharacterController(
        c->ghost, static_cast<btConvexShape *>(shape->shape), step_height, to_bt(up));
    c->ghost->setWorldTransform(to_bt(at));
    return c;
}

void eco_bt_character_destroy(eco_bt_character *c)
{
    if (c == nullptr) {
        return;
    }

    delete c->controller;
    delete c->ghost;
    delete c;
}

uint64_t eco_bt_character_id(eco_bt_character *c)
{
    return c->obj.id;
}

void eco_bt_character_get_transform(eco_bt_character *c, eco_bt_transform *out)
{
    store(out, c->ghost->getWorldTransform());
}

void eco_bt_character_warp(eco_bt_character *c, const eco_bt_vec3 *origin)
{
    c->controller->warp(to_bt(origin));
}

void eco_bt_character_walk(eco_bt_character *c, const eco_bt_vec3 *per_step)
{
    c->controller->setWalkDirection(to_bt(per_step));
}

void eco_bt_character_set_velocity(eco_bt_character *c, const eco_bt_vec3 *velocity, float seconds)
{
    c->controller->setVelocityForTimeInterval(to_bt(velocity), seconds);
}

void eco_bt_character_get_velocity(eco_bt_character *c, eco_bt_vec3 *out)
{
    store(out, c->controller->getLinearVelocity());
}

void eco_bt_character_jump(eco_bt_character *c, const eco_bt_vec3 *velocity)
{
    c->controller->jump(to_bt(velocity));
}

uint32_t eco_bt_character_can_jump(eco_bt_character *c)
{
    return c->controller->canJump() ? 1u : 0u;
}

uint32_t eco_bt_character_grounded(eco_bt_character *c)
{
    return c->controller->onGround() ? 1u : 0u;
}

void eco_bt_character_set_jump_speed(eco_bt_character *c, float speed)
{
    c->controller->setJumpSpeed(speed);
}

void eco_bt_character_set_fall_speed(eco_bt_character *c, float speed)
{
    c->controller->setFallSpeed(speed);
}

void eco_bt_character_set_max_jump_height(eco_bt_character *c, float height)
{
    c->controller->setMaxJumpHeight(height);
}

void eco_bt_character_set_max_slope(eco_bt_character *c, float radians)
{
    c->controller->setMaxSlope(radians);
}

float eco_bt_character_get_max_slope(eco_bt_character *c)
{
    return c->controller->getMaxSlope();
}

void eco_bt_character_set_gravity(eco_bt_character *c, const eco_bt_vec3 *gravity)
{
    c->controller->setGravity(to_bt(gravity));
}

void eco_bt_character_get_gravity(eco_bt_character *c, eco_bt_vec3 *out)
{
    store(out, c->controller->getGravity());
}

void eco_bt_character_set_step_height(eco_bt_character *c, float height)
{
    c->controller->setStepHeight(height);
}

float eco_bt_character_get_step_height(eco_bt_character *c)
{
    return c->controller->getStepHeight();
}

void eco_bt_character_set_up(eco_bt_character *c, const eco_bt_vec3 *up)
{
    btTransform at = c->ghost->getWorldTransform();
    c->controller->setUp(to_bt(up));
    c->ghost->setWorldTransform(at);
}

void eco_bt_character_set_max_penetration(eco_bt_character *c, float depth)
{
    c->controller->setMaxPenetrationDepth(depth);
}

uint32_t eco_bt_character_in_world(eco_bt_character *c)
{
    return c->ghost->getBroadphaseHandle() != nullptr ? 1u : 0u;
}
