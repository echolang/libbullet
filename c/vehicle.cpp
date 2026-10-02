#include "internal.h"

/*
 * Like btDefaultVehicleRaycaster, except that a wheel never stands on
 * its own chassis, on anything without contact response, or on anything
 * that is not a rigid body: sensors, ghosts, characters and soft bodies
 * are thin air to a wheel.
 */
struct WheelRay : public btCollisionWorld::ClosestRayResultCallback
{
    const btCollisionObject *chassis;

    WheelRay(const btVector3 &from, const btVector3 &to, const btCollisionObject *chassis)
        : ClosestRayResultCallback(from, to), chassis(chassis)
    {
    }

    btScalar addSingleResult(btCollisionWorld::LocalRayResult &hit, bool normalInWorldSpace) override
    {
        const btCollisionObject *object = hit.m_collisionObject;
        if (object == chassis || !object->hasContactResponse() || btRigidBody::upcast(object) == nullptr) {
            return btScalar(1);
        }

        return ClosestRayResultCallback::addSingleResult(hit, normalInWorldSpace);
    }
};

void *EcoVehicleRaycaster::castRay(const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result)
{
    if (world == nullptr) {
        return nullptr;
    }

    WheelRay ray(from, to, chassis);
    world->rayTest(from, to, ray);

    if (!ray.hasHit()) {
        return nullptr;
    }

    // WheelRay only keeps rigid bodies
    result.m_hitPointInWorld = ray.m_hitPointWorld;
    result.m_hitNormalInWorld = ray.m_hitNormalWorld.normalized();
    result.m_distFraction = ray.m_closestHitFraction;
    return const_cast<btRigidBody *>(btRigidBody::upcast(ray.m_collisionObject));
}

eco_bt_vehicle *eco_bt_vehicle_create(eco_bt_body *chassis)
{
    eco_bt_vehicle *v = new (std::nothrow) eco_bt_vehicle();
    if (v == nullptr) {
        return nullptr;
    }

    // Bullet ignores the vehicle-wide tuning; each wheel gets its own
    v->raycaster.chassis = chassis->body;
    v->vehicle = new btRaycastVehicle(btRaycastVehicle::btVehicleTuning(), chassis->body, &v->raycaster);
    v->vehicle->setCoordinateSystem(0, 1, 2);
    return v;
}

void eco_bt_vehicle_destroy(eco_bt_vehicle *v)
{
    if (v == nullptr) {
        return;
    }

    delete v->vehicle;
    delete v;
}

/*
 * The suspension tuning is per wheel in Bullet too: addWheel copies it
 * from the tuning it is handed. Roll influence is not in the tuning, so
 * it is set on the wheel after.
 */
int32_t eco_bt_vehicle_add_wheel(eco_bt_vehicle *v, const eco_bt_wheel *w)
{
    btRaycastVehicle::btVehicleTuning tuning;
    tuning.m_suspensionStiffness = w->stiffness;
    tuning.m_suspensionCompression = w->compression;
    tuning.m_suspensionDamping = w->relaxation;
    tuning.m_frictionSlip = w->friction_slip;
    tuning.m_maxSuspensionTravelCm = w->max_travel_cm;
    tuning.m_maxSuspensionForce = w->max_force;

    btWheelInfo &wheel = v->vehicle->addWheel(
        to_bt(&w->connection),
        to_bt(&w->direction),
        to_bt(&w->axle),
        w->rest_length,
        w->radius,
        tuning,
        w->front != 0);
    wheel.m_rollInfluence = w->roll_influence;

    return v->vehicle->getNumWheels() - 1;
}

int32_t eco_bt_vehicle_wheel_count(eco_bt_vehicle *v)
{
    return v->vehicle->getNumWheels();
}

/*
 * updateWheelTransform clears the wheel's contact as a side effect, which
 * the next step's ray would set again. Put it back, so reading a wheel
 * changes nothing.
 */
void eco_bt_vehicle_wheel_state(eco_bt_vehicle *v, int32_t wheel, eco_bt_wheel_state *out)
{
    btWheelInfo &info = v->vehicle->getWheelInfo(wheel);
    btWheelInfo::RaycastInfo ray = info.m_raycastInfo;
    v->vehicle->updateWheelTransform(wheel, true);
    info.m_raycastInfo.m_isInContact = ray.m_isInContact;
    info.m_raycastInfo.m_groundObject = ray.m_groundObject;

    store(&out->transform, info.m_worldTransform);
    store(&out->contact_point, info.m_raycastInfo.m_contactPointWS);
    store(&out->contact_normal, info.m_raycastInfo.m_contactNormalWS);
    out->rotation = info.m_rotation;
    out->steering = info.m_steering;
    out->suspension = info.m_raycastInfo.m_suspensionLength;
    out->skid = info.m_skidInfo;
    out->engine = info.m_engineForce;
    out->brake = info.m_brake;
    out->contact = info.m_raycastInfo.m_isInContact ? 1u : 0u;
}

void eco_bt_vehicle_set_steering(eco_bt_vehicle *v, int32_t wheel, float radians)
{
    v->vehicle->setSteeringValue(radians, wheel);
}

void eco_bt_vehicle_set_engine(eco_bt_vehicle *v, int32_t wheel, float force)
{
    v->vehicle->applyEngineForce(force, wheel);
}

void eco_bt_vehicle_set_brake(eco_bt_vehicle *v, int32_t wheel, float force)
{
    v->vehicle->setBrake(force, wheel);
}

float eco_bt_vehicle_speed(eco_bt_vehicle *v)
{
    return v->vehicle->getCurrentSpeedKmHour();
}

void eco_bt_vehicle_forward(eco_bt_vehicle *v, eco_bt_vec3 *out)
{
    store(out, v->vehicle->getForwardVector());
}

void eco_bt_vehicle_set_axes(eco_bt_vehicle *v, int32_t right, int32_t up, int32_t forward)
{
    v->vehicle->setCoordinateSystem(right, up, forward);
}

void eco_bt_vehicle_reset(eco_bt_vehicle *v)
{
    v->vehicle->resetSuspension();
}

uint32_t eco_bt_vehicle_in_world(eco_bt_vehicle *v)
{
    return v->raycaster.world != nullptr ? 1u : 0u;
}
