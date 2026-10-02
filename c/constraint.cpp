#include "internal.h"

/*
 * Echo keeps one class per constraint kind, so a hinge function is only
 * ever handed a hinge. The casts below lean on that. Hinge2 is a Spring2
 * six dof underneath, so the sixdof calls serve it too.
 *
 * Every constraint reports feedback: it is a few vectors per step, and
 * applied impulse and breaking both read it.
 */

static eco_bt_constraint *wrap(btTypedConstraint *constraint)
{
    eco_bt_constraint *c = new (std::nothrow) eco_bt_constraint();
    if (c == nullptr) {
        delete constraint;
        return nullptr;
    }

    c->constraint = constraint;
    constraint->enableFeedback(true);
    constraint->setJointFeedback(&c->feedback);
    return c;
}

/*
 * A motor or target change means nothing to a sleeping body.
 */
static void wake(eco_bt_constraint *c)
{
    c->constraint->getRigidBodyA().activate();
    c->constraint->getRigidBodyB().activate();
}

/*
 * A constraint with b = NULL holds a to the world. Bullet's shared fixed
 * body stands in for b, sitting at the world origin, so the b-side frame
 * is a's frame taken to world space where a is right now.
 *
 * Sliders and six dofs measure positions and drive motors as B relative
 * to A, so for those the world goes on the A side instead, the way
 * Bullet's own single-body constructors do it: a positive motor then
 * moves the body forward, and a falling body reads a negative offset.
 */
static btRigidBody &other(eco_bt_body *b)
{
    return b == nullptr ? btTypedConstraint::getFixedBody() : *b->body;
}

template <typename T>
static auto b_side(eco_bt_body *a, eco_bt_body *b, const T *on_a, const T *on_b) -> decltype(to_bt(on_a))
{
    if (b == nullptr) {
        return a->body->getCenterOfMassTransform() * to_bt(on_a);
    }

    return to_bt(on_b);
}

static btPoint2PointConstraint *as_point(eco_bt_constraint *c)
{
    return static_cast<btPoint2PointConstraint *>(c->constraint);
}

static btHingeConstraint *as_hinge(eco_bt_constraint *c)
{
    return static_cast<btHingeConstraint *>(c->constraint);
}

static btSliderConstraint *as_slider(eco_bt_constraint *c)
{
    return static_cast<btSliderConstraint *>(c->constraint);
}

static btGeneric6DofSpringConstraint *as_legacy(eco_bt_constraint *c)
{
    return static_cast<btGeneric6DofSpringConstraint *>(c->constraint);
}

static btGeneric6DofSpring2Constraint *as_sixdof(eco_bt_constraint *c)
{
    return static_cast<btGeneric6DofSpring2Constraint *>(c->constraint);
}

static btConeTwistConstraint *as_cone(eco_bt_constraint *c)
{
    return static_cast<btConeTwistConstraint *>(c->constraint);
}

static btGearConstraint *as_gear(eco_bt_constraint *c)
{
    return static_cast<btGearConstraint *>(c->constraint);
}

static btHinge2Constraint *as_hinge2(eco_bt_constraint *c)
{
    return static_cast<btHinge2Constraint *>(c->constraint);
}

static btUniversalConstraint *as_universal(eco_bt_constraint *c)
{
    return static_cast<btUniversalConstraint *>(c->constraint);
}

/* common */

void eco_bt_constraint_destroy(eco_bt_constraint *c)
{
    if (c == nullptr) {
        return;
    }

    delete c->constraint;
    delete c;
}

uint32_t eco_bt_constraint_enabled(eco_bt_constraint *c)
{
    return c->constraint->isEnabled() ? 1u : 0u;
}

void eco_bt_constraint_set_enabled(eco_bt_constraint *c, uint32_t on)
{
    c->constraint->setEnabled(on != 0);
}

float eco_bt_constraint_breaking_impulse(eco_bt_constraint *c)
{
    return c->constraint->getBreakingImpulseThreshold();
}

void eco_bt_constraint_set_breaking_impulse(eco_bt_constraint *c, float impulse)
{
    c->constraint->setBreakingImpulseThreshold(impulse);
}

float eco_bt_constraint_applied_impulse(eco_bt_constraint *c)
{
    return c->constraint->getAppliedImpulse();
}

void eco_bt_constraint_feedback(eco_bt_constraint *c, eco_bt_feedback *out)
{
    store(&out->force_a, c->feedback.m_appliedForceBodyA);
    store(&out->torque_a, c->feedback.m_appliedTorqueBodyA);
    store(&out->force_b, c->feedback.m_appliedForceBodyB);
    store(&out->torque_b, c->feedback.m_appliedTorqueBodyB);
}

void eco_bt_constraint_set_iterations(eco_bt_constraint *c, int32_t iterations)
{
    c->constraint->setOverrideNumSolverIterations(iterations);
}

void eco_bt_constraint_set_param(eco_bt_constraint *c, int32_t param, float value, int32_t axis)
{
    c->constraint->setParam(param, value, axis);
}

void eco_bt_constraint_set_debug_size(eco_bt_constraint *c, float size)
{
    c->constraint->setDbgDrawSize(size);
}

/* point to point */

eco_bt_constraint *eco_bt_point_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *pivot_a,
    const eco_bt_vec3 *pb
)
{
    return wrap(new btPoint2PointConstraint(*a->body, other(b), to_bt(pivot_a), b_side(a, b, pivot_a, pb)));
}

void eco_bt_point_set_pivots(eco_bt_constraint *c, const eco_bt_vec3 *pivot_a, const eco_bt_vec3 *pb)
{
    as_point(c)->setPivotA(to_bt(pivot_a));
    as_point(c)->setPivotB(to_bt(pb));
}

void eco_bt_point_get_pivots(eco_bt_constraint *c, eco_bt_vec3 *pivot_a, eco_bt_vec3 *pb)
{
    store(pivot_a, as_point(c)->getPivotInA());
    store(pb, as_point(c)->getPivotInB());
}

void eco_bt_point_set_tuning(eco_bt_constraint *c, float tau, float damping, float impulse_clamp)
{
    as_point(c)->m_setting.m_tau = tau;
    as_point(c)->m_setting.m_damping = damping;
    as_point(c)->m_setting.m_impulseClamp = impulse_clamp;
}

/* hinge */

eco_bt_constraint *eco_bt_hinge_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *pivot_a,
    const eco_bt_vec3 *pb,
    const eco_bt_vec3 *axis_a,
    const eco_bt_vec3 *axis_b
)
{
    btVector3 axis = to_bt(axis_a);
    btVector3 world_axis = b == nullptr ? a->body->getCenterOfMassTransform().getBasis() * axis : to_bt(axis_b);

    return wrap(new btHingeConstraint(
        *a->body, other(b), to_bt(pivot_a), b_side(a, b, pivot_a, pb), axis, world_axis));
}

eco_bt_constraint *eco_bt_hinge_create_frames(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb
)
{
    return wrap(new btHingeConstraint(*a->body, other(b), to_bt(frame_a), b_side(a, b, frame_a, fb)));
}

void eco_bt_hinge_set_limit(
    eco_bt_constraint *c,
    float low,
    float high,
    float softness,
    float bias,
    float relaxation
)
{
    as_hinge(c)->setLimit(low, high, softness, bias, relaxation);
}

float eco_bt_hinge_low(eco_bt_constraint *c)
{
    return as_hinge(c)->getLowerLimit();
}

float eco_bt_hinge_high(eco_bt_constraint *c)
{
    return as_hinge(c)->getUpperLimit();
}

float eco_bt_hinge_angle(eco_bt_constraint *c)
{
    return as_hinge(c)->getHingeAngle();
}

void eco_bt_hinge_set_motor(eco_bt_constraint *c, uint32_t on, float velocity, float max_impulse)
{
    as_hinge(c)->enableAngularMotor(on != 0, velocity, max_impulse);
    wake(c);
}

void eco_bt_hinge_set_motor_target(eco_bt_constraint *c, float angle, float dt)
{
    as_hinge(c)->setMotorTarget(angle, dt);
    wake(c);
}

void eco_bt_hinge_set_axis(eco_bt_constraint *c, const eco_bt_vec3 *axis_a)
{
    btVector3 axis = to_bt(axis_a);
    as_hinge(c)->setAxis(axis);
}

void eco_bt_hinge_set_frames(eco_bt_constraint *c, const eco_bt_transform *frame_a, const eco_bt_transform *fb)
{
    as_hinge(c)->setFrames(to_bt(frame_a), to_bt(fb));
}

/* slider */

eco_bt_constraint *eco_bt_slider_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb,
    uint32_t linear_frame_a
)
{
    if (b == nullptr) {
        return wrap(new btSliderConstraint(
            btTypedConstraint::getFixedBody(), *a->body, b_side(a, b, frame_a, fb), to_bt(frame_a), true));
    }

    return wrap(new btSliderConstraint(*a->body, *b->body, to_bt(frame_a), to_bt(fb), linear_frame_a != 0));
}

void eco_bt_slider_set_linear_limit(eco_bt_constraint *c, float low, float high)
{
    as_slider(c)->setLowerLinLimit(low);
    as_slider(c)->setUpperLinLimit(high);
}

void eco_bt_slider_set_angular_limit(eco_bt_constraint *c, float low, float high)
{
    as_slider(c)->setLowerAngLimit(low);
    as_slider(c)->setUpperAngLimit(high);
}

float eco_bt_slider_position(eco_bt_constraint *c)
{
    return as_slider(c)->getLinearPos();
}

float eco_bt_slider_angle(eco_bt_constraint *c)
{
    return as_slider(c)->getAngularPos();
}

void eco_bt_slider_set_linear_motor(eco_bt_constraint *c, uint32_t on, float velocity, float max_force)
{
    as_slider(c)->setPoweredLinMotor(on != 0);
    as_slider(c)->setTargetLinMotorVelocity(velocity);
    as_slider(c)->setMaxLinMotorForce(max_force);
    wake(c);
}

void eco_bt_slider_set_angular_motor(eco_bt_constraint *c, uint32_t on, float velocity, float max_force)
{
    as_slider(c)->setPoweredAngMotor(on != 0);
    as_slider(c)->setTargetAngMotorVelocity(velocity);
    as_slider(c)->setMaxAngMotorForce(max_force);
    wake(c);
}

void eco_bt_slider_set_frames(eco_bt_constraint *c, const eco_bt_transform *frame_a, const eco_bt_transform *fb)
{
    as_slider(c)->setFrames(to_bt(frame_a), to_bt(fb));
}

/* legacy six dof */

eco_bt_constraint *eco_bt_legacy_sixdof_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb,
    uint32_t linear_frame_a
)
{
    return wrap(new btGeneric6DofSpringConstraint(
        *a->body, other(b), to_bt(frame_a), b_side(a, b, frame_a, fb), linear_frame_a != 0));
}

void eco_bt_legacy_sixdof_set_linear(eco_bt_constraint *c, const eco_bt_vec3 *lower, const eco_bt_vec3 *upper)
{
    as_legacy(c)->setLinearLowerLimit(to_bt(lower));
    as_legacy(c)->setLinearUpperLimit(to_bt(upper));
}

void eco_bt_legacy_sixdof_set_angular(eco_bt_constraint *c, const eco_bt_vec3 *lower, const eco_bt_vec3 *upper)
{
    as_legacy(c)->setAngularLowerLimit(to_bt(lower));
    as_legacy(c)->setAngularUpperLimit(to_bt(upper));
}

/* six dof (spring 2) */

eco_bt_constraint *eco_bt_sixdof_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb
)
{
    if (b == nullptr) {
        return wrap(new btGeneric6DofSpring2Constraint(
            btTypedConstraint::getFixedBody(), *a->body, b_side(a, b, frame_a, fb), to_bt(frame_a)));
    }

    return wrap(new btGeneric6DofSpring2Constraint(*a->body, *b->body, to_bt(frame_a), to_bt(fb)));
}

void eco_bt_sixdof_set_frames(eco_bt_constraint *c, const eco_bt_transform *frame_a, const eco_bt_transform *fb)
{
    as_sixdof(c)->setFrames(to_bt(frame_a), to_bt(fb));
}

void eco_bt_sixdof_set_linear_lower(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setLinearLowerLimit(to_bt(v));
}

void eco_bt_sixdof_set_linear_upper(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setLinearUpperLimit(to_bt(v));
}

void eco_bt_sixdof_set_angular_lower(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setAngularLowerLimit(to_bt(v));
}

void eco_bt_sixdof_set_angular_lower_reversed(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setAngularLowerLimitReversed(to_bt(v));
}

void eco_bt_sixdof_set_angular_upper(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setAngularUpperLimit(to_bt(v));
}

void eco_bt_sixdof_set_angular_upper_reversed(eco_bt_constraint *c, const eco_bt_vec3 *v)
{
    as_sixdof(c)->setAngularUpperLimitReversed(to_bt(v));
}

void eco_bt_sixdof_set_limit(eco_bt_constraint *c, int32_t axis, float low, float high)
{
    as_sixdof(c)->setLimit(axis, low, high);
}

void eco_bt_sixdof_set_limit_reversed(eco_bt_constraint *c, int32_t axis, float low, float high)
{
    as_sixdof(c)->setLimitReversed(axis, low, high);
}

void eco_bt_sixdof_set_axis(eco_bt_constraint *c, const eco_bt_vec3 *axis1, const eco_bt_vec3 *axis2)
{
    as_sixdof(c)->setAxis(to_bt(axis1), to_bt(axis2));
}

void eco_bt_sixdof_set_bounce(eco_bt_constraint *c, int32_t index, float bounce)
{
    as_sixdof(c)->setBounce(index, bounce);
}

void eco_bt_sixdof_enable_motor(eco_bt_constraint *c, int32_t index, uint32_t on)
{
    as_sixdof(c)->enableMotor(index, on != 0);
    wake(c);
}

void eco_bt_sixdof_set_servo(eco_bt_constraint *c, int32_t index, uint32_t on)
{
    as_sixdof(c)->setServo(index, on != 0);
    wake(c);
}

void eco_bt_sixdof_set_target_velocity(eco_bt_constraint *c, int32_t index, float velocity)
{
    as_sixdof(c)->setTargetVelocity(index, velocity);
    wake(c);
}

void eco_bt_sixdof_set_servo_target(eco_bt_constraint *c, int32_t index, float target)
{
    as_sixdof(c)->setServoTarget(index, target);
    wake(c);
}

void eco_bt_sixdof_set_max_motor_force(eco_bt_constraint *c, int32_t index, float force)
{
    as_sixdof(c)->setMaxMotorForce(index, force);
    wake(c);
}

void eco_bt_sixdof_enable_spring(eco_bt_constraint *c, int32_t index, uint32_t on)
{
    as_sixdof(c)->enableSpring(index, on != 0);
}

void eco_bt_sixdof_set_stiffness(eco_bt_constraint *c, int32_t index, float stiffness, uint32_t limit_if_needed)
{
    as_sixdof(c)->setStiffness(index, stiffness, limit_if_needed != 0);
}

void eco_bt_sixdof_set_damping(eco_bt_constraint *c, int32_t index, float damping, uint32_t limit_if_needed)
{
    as_sixdof(c)->setDamping(index, damping, limit_if_needed != 0);
}

void eco_bt_sixdof_set_equilibrium(eco_bt_constraint *c)
{
    as_sixdof(c)->setEquilibriumPoint();
}

float eco_bt_sixdof_angle(eco_bt_constraint *c, int32_t axis)
{
    as_sixdof(c)->calculateTransforms();
    return as_sixdof(c)->getAngle(axis);
}

float eco_bt_sixdof_position(eco_bt_constraint *c, int32_t axis)
{
    as_sixdof(c)->calculateTransforms();
    return as_sixdof(c)->getRelativePivotPosition(axis);
}

/* cone twist */

eco_bt_constraint *eco_bt_cone_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb
)
{
    return wrap(new btConeTwistConstraint(*a->body, other(b), to_bt(frame_a), b_side(a, b, frame_a, fb)));
}

void eco_bt_cone_set_limit(
    eco_bt_constraint *c,
    float swing1,
    float swing2,
    float twist,
    float softness,
    float bias,
    float relaxation
)
{
    as_cone(c)->setLimit(swing1, swing2, twist, softness, bias, relaxation);
}

void eco_bt_cone_set_damping(eco_bt_constraint *c, float damping)
{
    as_cone(c)->setDamping(damping);
}

void eco_bt_cone_set_motor(eco_bt_constraint *c, uint32_t on, float max_impulse)
{
    as_cone(c)->enableMotor(on != 0);
    as_cone(c)->setMaxMotorImpulse(max_impulse);
    wake(c);
}

void eco_bt_cone_set_motor_target(eco_bt_constraint *c, const eco_bt_quat *target)
{
    as_cone(c)->setMotorTarget(to_bt(target));
    wake(c);
}

void eco_bt_cone_set_frames(eco_bt_constraint *c, const eco_bt_transform *frame_a, const eco_bt_transform *fb)
{
    as_cone(c)->setFrames(to_bt(frame_a), to_bt(fb));
}

float eco_bt_cone_twist(eco_bt_constraint *c)
{
    return as_cone(c)->getTwistAngle();
}

float eco_bt_cone_swing1(eco_bt_constraint *c)
{
    return as_cone(c)->getSwingSpan1();
}

float eco_bt_cone_swing2(eco_bt_constraint *c)
{
    return as_cone(c)->getSwingSpan2();
}

/* fixed */

eco_bt_constraint *eco_bt_fixed_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *fb
)
{
    return wrap(new btFixedConstraint(*a->body, other(b), to_bt(frame_a), b_side(a, b, frame_a, fb)));
}

/* gear */

eco_bt_constraint *eco_bt_gear_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *axis_a,
    const eco_bt_vec3 *axis_b,
    float ratio
)
{
    return wrap(new btGearConstraint(*a->body, *b->body, to_bt(axis_a), to_bt(axis_b), ratio));
}

float eco_bt_gear_get_ratio(eco_bt_constraint *c)
{
    return as_gear(c)->getRatio();
}

void eco_bt_gear_set_ratio(eco_bt_constraint *c, float ratio)
{
    as_gear(c)->setRatio(ratio);
}

/* hinge2 */

eco_bt_constraint *eco_bt_hinge2_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *anchor,
    const eco_bt_vec3 *axis1,
    const eco_bt_vec3 *axis2
)
{
    btVector3 at = to_bt(anchor);
    btVector3 one = to_bt(axis1);
    btVector3 two = to_bt(axis2);
    return wrap(new btHinge2Constraint(*a->body, *b->body, at, one, two));
}

void eco_bt_hinge2_set_steering(eco_bt_constraint *c, float low, float high)
{
    as_hinge2(c)->setLowerLimit(low);
    as_hinge2(c)->setUpperLimit(high);
}

/* universal */

eco_bt_constraint *eco_bt_universal_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *anchor,
    const eco_bt_vec3 *axis1,
    const eco_bt_vec3 *axis2
)
{
    return wrap(new btUniversalConstraint(*a->body, *b->body, to_bt(anchor), to_bt(axis1), to_bt(axis2)));
}

void eco_bt_universal_set_limit(eco_bt_constraint *c, float low1, float high1, float low2, float high2)
{
    as_universal(c)->setLowerLimit(low1, low2);
    as_universal(c)->setUpperLimit(high1, high2);
}

float eco_bt_universal_angle1(eco_bt_constraint *c)
{
    as_universal(c)->calculateTransforms();
    return as_universal(c)->getAngle1();
}

float eco_bt_universal_angle2(eco_bt_constraint *c)
{
    as_universal(c)->calculateTransforms();
    return as_universal(c)->getAngle2();
}
