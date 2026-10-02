#ifndef ECO_BT_H
#define ECO_BT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Echo-facing ABI. Bullet is C++, so every call Echo makes lands here
 * first. Objects with identity are opaque heap pointers; values cross as
 * the POD structs below, always through a pointer. Bools are uint32_t.
 * A create that returns NULL is out of memory.
 *
 * Every collision object this shim makes carries a uint64 id, unique for
 * the process. Queries and contacts report ids, and Echo maps them back
 * to its own handles. Id 0 is never handed out.
 *
 * Bulk results (hits, contacts, events, soft body nodes) are two calls:
 * the first runs the query, keeps the result on the C++ side and returns
 * how many there are; the matching *_copy writes up to `count` of them
 * into Echo's buffer.
 */

typedef struct eco_bt_world eco_bt_world;
typedef struct eco_bt_shape eco_bt_shape;
typedef struct eco_bt_body eco_bt_body;
typedef struct eco_bt_constraint eco_bt_constraint;
typedef struct eco_bt_ghost eco_bt_ghost;
typedef struct eco_bt_character eco_bt_character;
typedef struct eco_bt_vehicle eco_bt_vehicle;
typedef struct eco_bt_soft eco_bt_soft;

/*
 * Layout of bt::Vec3: 3 x float32, this order.
 */
typedef struct {
    float x;
    float y;
    float z;
} eco_bt_vec3;

/*
 * Layout of bt::Quat: 4 x float32, w last like btQuaternion.
 */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} eco_bt_quat;

/*
 * Layout of bt::Transform: origin, then rotation. No padding.
 */
typedef struct {
    eco_bt_vec3 origin;
    eco_bt_quat rotation;
} eco_bt_transform;

/*
 * Layout of bt::Aabb: the low corner, then the high one.
 */
typedef struct {
    eco_bt_vec3 min;
    eco_bt_vec3 max;
} eco_bt_aabb;

/*
 * A ray or sweep hit. `id` is the body's.
 */
typedef struct {
    uint64_t id;
    eco_bt_vec3 point;
    eco_bt_vec3 normal;
    float fraction;
} eco_bt_hit;

/*
 * One contact point between two objects. `normal` is on b, pointing at
 * a. `distance` is negative while they overlap.
 */
typedef struct {
    uint64_t a;
    uint64_t b;
    eco_bt_vec3 point_a;
    eco_bt_vec3 point_b;
    eco_bt_vec3 normal;
    float distance;
    float impulse;
} eco_bt_contact;

/*
 * A pair that started (began = 1) or stopped (began = 0) touching.
 */
typedef struct {
    uint64_t a;
    uint64_t b;
    uint32_t began;
} eco_bt_event;

/*
 * A dynamic body the last step moved: the slot its owner gave it
 * (eco_bt_body_set_slot) and its world transform.
 */
typedef struct {
    int32_t slot;
    eco_bt_transform at;
} eco_bt_pose;

/*
 * The world's btContactSolverInfo, the parts worth tuning.
 */
typedef struct {
    int32_t iterations;
    float erp;
    float erp2;
    float cfm;
    float friction_erp;
    float warmstarting;
    float restitution_threshold;
    float split_threshold;
    uint32_t split_impulse;
    int32_t min_batch;
} eco_bt_solver;

/*
 * What a constraint pushed on each body during the last step.
 */
typedef struct {
    eco_bt_vec3 force_a;
    eco_bt_vec3 torque_a;
    eco_bt_vec3 force_b;
    eco_bt_vec3 torque_b;
} eco_bt_feedback;

/*
 * A raycast vehicle wheel as it is added.
 */
typedef struct {
    eco_bt_vec3 connection;
    eco_bt_vec3 direction;
    eco_bt_vec3 axle;
    float rest_length;
    float radius;
    float stiffness;
    float compression;
    float relaxation;
    float friction_slip;
    float roll_influence;
    float max_travel_cm;
    float max_force;
    uint32_t front;
} eco_bt_wheel;

/*
 * A raycast vehicle wheel as it is now.
 */
typedef struct {
    eco_bt_transform transform;
    eco_bt_vec3 contact_point;
    eco_bt_vec3 contact_normal;
    float rotation;
    float steering;
    float suspension;
    float skid;
    float engine;
    float brake;
    uint32_t contact;
} eco_bt_wheel_state;

/*
 * The per soft body knobs of btSoftBody::Config.
 */
typedef struct {
    float damping;
    float drag;
    float lift;
    float pressure;
    float volume;
    float friction;
    float pose;
    float rigid_hardness;
    float kinetic_hardness;
    float soft_hardness;
    float anchor_hardness;
    int32_t position_iterations;
    int32_t velocity_iterations;
    int32_t drift_iterations;
    int32_t cluster_iterations;
} eco_bt_soft_config;

#ifdef __cplusplus
#define ECO_BT_ASSERT static_assert
#else
#define ECO_BT_ASSERT _Static_assert
#endif

ECO_BT_ASSERT(sizeof(eco_bt_vec3) == 12, "eco_bt_vec3 is 3 x float");
ECO_BT_ASSERT(offsetof(eco_bt_vec3, z) == 8, "z");
ECO_BT_ASSERT(sizeof(eco_bt_quat) == 16, "eco_bt_quat is 4 x float");
ECO_BT_ASSERT(offsetof(eco_bt_quat, w) == 12, "w");
ECO_BT_ASSERT(sizeof(eco_bt_transform) == 28, "eco_bt_transform is vec3 + quat");
ECO_BT_ASSERT(offsetof(eco_bt_transform, rotation) == 12, "rotation");
ECO_BT_ASSERT(sizeof(eco_bt_aabb) == 24, "eco_bt_aabb is 2 x vec3");
ECO_BT_ASSERT(sizeof(eco_bt_hit) == 40, "eco_bt_hit");
ECO_BT_ASSERT(offsetof(eco_bt_hit, fraction) == 32, "fraction");
ECO_BT_ASSERT(sizeof(eco_bt_contact) == 64, "eco_bt_contact");
ECO_BT_ASSERT(offsetof(eco_bt_contact, impulse) == 56, "impulse");
ECO_BT_ASSERT(sizeof(eco_bt_event) == 24, "eco_bt_event");
ECO_BT_ASSERT(sizeof(eco_bt_pose) == 32, "eco_bt_pose is int32 + transform");
ECO_BT_ASSERT(offsetof(eco_bt_pose, at) == 4, "at");
ECO_BT_ASSERT(sizeof(eco_bt_solver) == 40, "eco_bt_solver");
ECO_BT_ASSERT(sizeof(eco_bt_feedback) == 48, "eco_bt_feedback");
ECO_BT_ASSERT(sizeof(eco_bt_wheel) == 76, "eco_bt_wheel");
ECO_BT_ASSERT(sizeof(eco_bt_wheel_state) == 80, "eco_bt_wheel_state");
ECO_BT_ASSERT(sizeof(eco_bt_soft_config) == 60, "eco_bt_soft_config");

int32_t eco_bt_version(void);

/*
 * Seconds a body must stay under its sleep thresholds before it sleeps.
 * Bullet keeps this in one global, so it holds for every world in the
 * process. Bullet's default is 2.
 */
void eco_bt_set_sleep_seconds(float seconds);

/*
 * Threads the process task scheduler can run a parallel world on, the
 * calling thread included; 1 without threads. Builds the scheduler.
 */
int32_t eco_bt_max_threads(void);

/*
 * Stop and free the task scheduler's worker threads. Call it once no
 * parallel world steps any more and before the process unloads this
 * code: a sleeping worker is still a thread inside it.
 */
void eco_bt_shutdown(void);
float eco_bt_sleep_seconds(void);

/* world */

/*
 * solver: 0 sequential impulse, 1 NNCG, 2 MLCP Dantzig. Bullet's Lemke
 * MLCP solver is left out: it blows up on a box resting on a plane.
 */
eco_bt_world *eco_bt_world_create(const eco_bt_vec3 *gravity, int32_t solver);

/*
 * A rigid-only world stepped across `threads` threads of the process
 * task scheduler (clamped to what it has). Null when Bullet was built
 * without threads or the machine has one core.
 */
eco_bt_world *eco_bt_world_create_parallel(const eco_bt_vec3 *gravity, int32_t threads);
void eco_bt_world_destroy(eco_bt_world *world);
void eco_bt_world_set_gravity(eco_bt_world *world, const eco_bt_vec3 *gravity);
void eco_bt_world_get_gravity(eco_bt_world *world, eco_bt_vec3 *out);
int32_t eco_bt_world_step(eco_bt_world *world, float dt, int32_t max_sub_steps, float fixed_step);

/*
 * Exactly one step of `dt`: no accumulator, no motion state
 * interpolation. For a caller that runs its own fixed passes.
 */
void eco_bt_world_advance(eco_bt_world *world, float dt);

/*
 * Every awake dynamic body, as its slot and world transform. Writes up
 * to `capacity` and returns how many there are, so a short buffer can be
 * grown and the call repeated. Static, kinematic and sleeping bodies are
 * left out.
 */
size_t eco_bt_world_moving(eco_bt_world *world, eco_bt_pose *out, size_t capacity);

/*
 * Room for `count` collision objects in the world's object array, so
 * adding that many does not grow it.
 */
void eco_bt_world_reserve(eco_bt_world *world, int32_t count);

/*
 * Recompute one body's broadphase box now. With update-all-aabbs off a
 * static body moved by hand keeps its old box until this is called.
 */
void eco_bt_world_refresh(eco_bt_world *world, eco_bt_body *body);
void eco_bt_world_get_solver(eco_bt_world *world, eco_bt_solver *out);
void eco_bt_world_set_solver(eco_bt_world *world, const eco_bt_solver *settings);
void eco_bt_world_clear_forces(eco_bt_world *world);
void eco_bt_world_set_update_all_aabbs(eco_bt_world *world, uint32_t on);
void eco_bt_world_set_air_density(eco_bt_world *world, float density);

void eco_bt_world_add_body(eco_bt_world *world, eco_bt_body *body);
void eco_bt_world_add_body_filtered(eco_bt_world *world, eco_bt_body *body, int32_t group, int32_t mask);
void eco_bt_world_remove_body(eco_bt_world *world, eco_bt_body *body);
void eco_bt_world_add_constraint(eco_bt_world *world, eco_bt_constraint *constraint, uint32_t no_collide);
void eco_bt_world_remove_constraint(eco_bt_world *world, eco_bt_constraint *constraint);
void eco_bt_world_add_ghost(eco_bt_world *world, eco_bt_ghost *ghost, int32_t group, int32_t mask);
void eco_bt_world_remove_ghost(eco_bt_world *world, eco_bt_ghost *ghost);
void eco_bt_world_add_character(eco_bt_world *world, eco_bt_character *character, int32_t group, int32_t mask);
void eco_bt_world_remove_character(eco_bt_world *world, eco_bt_character *character);
void eco_bt_world_add_vehicle(eco_bt_world *world, eco_bt_vehicle *vehicle);
void eco_bt_world_remove_vehicle(eco_bt_world *world, eco_bt_vehicle *vehicle);
/* 0 for a parallel world, which holds no soft bodies. */
uint32_t eco_bt_world_add_soft(eco_bt_world *world, eco_bt_soft *soft, int32_t group, int32_t mask);
void eco_bt_world_remove_soft(eco_bt_world *world, eco_bt_soft *soft);

/*
 * All four zero detaches the drawer. Otherwise the world draws into a
 * buffer of x,y,z,r,g,b per vertex, two vertices per line.
 */
void eco_bt_world_debug_mode(
    eco_bt_world *world,
    uint32_t wireframe,
    uint32_t aabb,
    uint32_t contacts,
    uint32_t constraints
);
size_t eco_bt_world_debug_draw(eco_bt_world *world);
void eco_bt_world_debug_copy(eco_bt_world *world, float *out, size_t count);

/* queries: bodies only, filtered by mask */

/*
 * Closest body along from -> to. Returns 1 and fills *hit, or 0. A
 * non-null `ignore` never counts.
 */
uint32_t eco_bt_world_ray(
    eco_bt_world *world,
    const eco_bt_vec3 *from,
    const eco_bt_vec3 *to,
    int32_t mask,
    const eco_bt_body *ignore,
    eco_bt_hit *hit
);

/*
 * Every body along from -> to, nearest first. Copy with world_hits_copy.
 */
size_t eco_bt_world_rays(eco_bt_world *world, const eco_bt_vec3 *from, const eco_bt_vec3 *to, int32_t mask);
void eco_bt_world_hits_copy(eco_bt_world *world, eco_bt_hit *out, size_t count);

/*
 * `shape` must be convex; the caller checks. Returns 1 and fills *hit,
 * or 0. With `facing` set, only surfaces the shape moves into count, so
 * a shape that starts inside something can still move out of it. A
 * non-null `ignore` never counts.
 */
uint32_t eco_bt_world_sweep(
    eco_bt_world *world,
    eco_bt_shape *shape,
    const eco_bt_transform *from,
    const eco_bt_transform *to,
    int32_t mask,
    uint32_t facing,
    const eco_bt_body *ignore,
    eco_bt_hit *hit
);

/*
 * Ids of the bodies `shape` at `at` touches. Copy with world_ids_copy.
 */
size_t eco_bt_world_overlap(eco_bt_world *world, eco_bt_shape *shape, const eco_bt_transform *at, int32_t mask);
size_t eco_bt_world_aabb_test(eco_bt_world *world, const eco_bt_aabb *box, int32_t mask);
void eco_bt_world_ids_copy(eco_bt_world *world, uint64_t *out, size_t count);

/*
 * Contact points. Copy with world_contacts_copy.
 */
size_t eco_bt_world_body_contacts(eco_bt_world *world, eco_bt_body *body, int32_t mask);
size_t eco_bt_world_pair_contacts(eco_bt_world *world, eco_bt_body *a, eco_bt_body *b);
size_t eco_bt_world_touching(eco_bt_world *world);
void eco_bt_world_contacts_copy(eco_bt_world *world, eco_bt_contact *out, size_t count);

/*
 * With tracking on, every substep diffs the touching pairs and queues
 * began and ended events. Taking them drains the queue.
 */
void eco_bt_world_track_contacts(eco_bt_world *world, uint32_t on);
size_t eco_bt_world_events(eco_bt_world *world);
void eco_bt_world_events_take(eco_bt_world *world, eco_bt_event *out, size_t count);

/* shapes */

eco_bt_shape *eco_bt_shape_empty(void);
eco_bt_shape *eco_bt_shape_sphere(float radius);
eco_bt_shape *eco_bt_shape_box(const eco_bt_vec3 *half_extents);
eco_bt_shape *eco_bt_shape_capsule(float radius, float height, int32_t axis);
eco_bt_shape *eco_bt_shape_cylinder(const eco_bt_vec3 *half_extents, int32_t axis);
eco_bt_shape *eco_bt_shape_cone(float radius, float height, int32_t axis);
eco_bt_shape *eco_bt_shape_plane(const eco_bt_vec3 *normal, float constant);
eco_bt_shape *eco_bt_shape_hull(const eco_bt_vec3 *points, int32_t count, uint32_t simplify);
eco_bt_shape *eco_bt_shape_spheres(const eco_bt_vec3 *centres, const float *radii, int32_t count);
eco_bt_shape *eco_bt_shape_heightfield(
    int32_t width,
    int32_t length,
    const float *heights,
    float min_height,
    float max_height,
    int32_t axis,
    uint32_t flip_quad_edges
);

/*
 * A heightfield over the caller's heights, not a copy: they must stay
 * where they are, unchanged in size, for as long as the shape lives.
 * Heights written in place are seen at once, but `min_height` and
 * `max_height` are fixed here. With `accelerate`, a chunked min/max
 * tree is built once so rays skip empty chunks.
 */
eco_bt_shape *eco_bt_shape_heightfield_borrowed(
    int32_t width,
    int32_t length,
    const float *heights,
    float min_height,
    float max_height,
    int32_t axis,
    uint32_t flip_quad_edges,
    uint32_t accelerate
);

/*
 * Triangle meshes. The shape copies vertices and indices; `indices` is
 * three per triangle and already checked against `vertex_count`.
 */
eco_bt_shape *eco_bt_shape_mesh(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
);
eco_bt_shape *eco_bt_shape_gimpact(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
);

eco_bt_shape *eco_bt_shape_compound(void);
void eco_bt_shape_compound_add(eco_bt_shape *compound, eco_bt_shape *child, const eco_bt_transform *at);
void eco_bt_shape_compound_set(eco_bt_shape *compound, int32_t index, const eco_bt_transform *at);

void eco_bt_shape_destroy(eco_bt_shape *shape);
void eco_bt_shape_set_scaling(eco_bt_shape *shape, const eco_bt_vec3 *scaling);
void eco_bt_shape_get_scaling(eco_bt_shape *shape, eco_bt_vec3 *out);
void eco_bt_shape_inertia(eco_bt_shape *shape, float mass, eco_bt_vec3 *out);
float eco_bt_shape_get_margin(eco_bt_shape *shape);
void eco_bt_shape_set_margin(eco_bt_shape *shape, float margin);
void eco_bt_shape_aabb(eco_bt_shape *shape, const eco_bt_transform *at, eco_bt_aabb *out);
float eco_bt_shape_bounds(eco_bt_shape *shape, eco_bt_vec3 *centre);
uint32_t eco_bt_shape_convex(eco_bt_shape *shape);
uint32_t eco_bt_shape_static_only(eco_bt_shape *shape);

/*
 * 0 empty, 1 sphere, 2 box, 3 capsule, 4 cylinder, 5 cone, 6 plane,
 * 7 hull, 8 spheres, 9 heightfield, 10 mesh, 11 gimpact, 12 compound.
 */
int32_t eco_bt_shape_kind(eco_bt_shape *shape);

/* bodies */

eco_bt_body *eco_bt_body_create(eco_bt_shape *shape, float mass, const eco_bt_transform *at);
void eco_bt_body_destroy(eco_bt_body *body);
uint64_t eco_bt_body_id(eco_bt_body *body);

/*
 * The owner's own index for this body, -1 until set. World moving
 * reports it, so an owner maps a pose back without a lookup.
 */
void eco_bt_body_set_slot(eco_bt_body *body, int32_t slot);
int32_t eco_bt_body_slot(eco_bt_body *body);

/*
 * Whether the world's debug drawer draws this body. Off for a body whose
 * wireframe would be enormous, such as a large heightfield.
 */
void eco_bt_body_set_debug_draw(eco_bt_body *body, uint32_t on);
void eco_bt_body_get_transform(eco_bt_body *body, eco_bt_transform *out);
void eco_bt_body_set_transform(eco_bt_body *body, const eco_bt_transform *at);
void eco_bt_body_move(eco_bt_body *body, const eco_bt_transform *at);
void eco_bt_body_get_velocity(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_set_velocity(eco_bt_body *body, const eco_bt_vec3 *v);
void eco_bt_body_get_angular_velocity(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_set_angular_velocity(eco_bt_body *body, const eco_bt_vec3 *v);
void eco_bt_body_velocity_at(eco_bt_body *body, const eco_bt_vec3 *rel, eco_bt_vec3 *out);
float eco_bt_body_get_mass(eco_bt_body *body);
void eco_bt_body_set_mass(eco_bt_body *body, float mass);
float eco_bt_body_inverse_mass(eco_bt_body *body);
void eco_bt_body_local_inertia(eco_bt_body *body, eco_bt_vec3 *out);
/*
 * The inverse of the principal inertia, in the body frame (zero on a
 * static or kinematic body). The world tensor is R diag(this) R^T.
 */
void eco_bt_body_inverse_inertia(eco_bt_body *body, eco_bt_vec3 *out);
float eco_bt_body_get_friction(eco_bt_body *body);
void eco_bt_body_set_friction(eco_bt_body *body, float friction);
void eco_bt_body_set_anisotropic_friction(eco_bt_body *body, const eco_bt_vec3 *friction);
float eco_bt_body_get_rolling_friction(eco_bt_body *body);
void eco_bt_body_set_rolling_friction(eco_bt_body *body, float friction);
float eco_bt_body_get_spinning_friction(eco_bt_body *body);
void eco_bt_body_set_spinning_friction(eco_bt_body *body, float friction);
float eco_bt_body_get_restitution(eco_bt_body *body);
void eco_bt_body_set_restitution(eco_bt_body *body, float restitution);
void eco_bt_body_set_damping(eco_bt_body *body, float linear, float angular);
float eco_bt_body_linear_damping(eco_bt_body *body);
float eco_bt_body_angular_damping(eco_bt_body *body);
void eco_bt_body_set_contact_stiffness(eco_bt_body *body, float stiffness, float damping);
void eco_bt_body_apply_force(eco_bt_body *body, const eco_bt_vec3 *force, const eco_bt_vec3 *rel);
void eco_bt_body_apply_central_force(eco_bt_body *body, const eco_bt_vec3 *force);
void eco_bt_body_apply_impulse(eco_bt_body *body, const eco_bt_vec3 *impulse, const eco_bt_vec3 *rel);
void eco_bt_body_apply_central_impulse(eco_bt_body *body, const eco_bt_vec3 *impulse);
void eco_bt_body_apply_torque(eco_bt_body *body, const eco_bt_vec3 *torque);
void eco_bt_body_apply_torque_impulse(eco_bt_body *body, const eco_bt_vec3 *torque);
/*
 * The impulses above without the wake: a sleeping body stays asleep and
 * an awake one keeps its sleep timer.
 */
void eco_bt_body_push(eco_bt_body *body, const eco_bt_vec3 *impulse, const eco_bt_vec3 *rel);
void eco_bt_body_push_central(eco_bt_body *body, const eco_bt_vec3 *impulse);
void eco_bt_body_push_torque(eco_bt_body *body, const eco_bt_vec3 *torque);
void eco_bt_body_total_force(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_total_torque(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_clear_forces(eco_bt_body *body);
void eco_bt_body_get_gravity(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_set_gravity(eco_bt_body *body, const eco_bt_vec3 *gravity);
void eco_bt_body_get_linear_factor(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_set_linear_factor(eco_bt_body *body, const eco_bt_vec3 *factor);
void eco_bt_body_get_angular_factor(eco_bt_body *body, eco_bt_vec3 *out);
void eco_bt_body_set_angular_factor(eco_bt_body *body, const eco_bt_vec3 *factor);
void eco_bt_body_set_ccd(eco_bt_body *body, float threshold, float radius);
void eco_bt_body_set_sleep_thresholds(eco_bt_body *body, float linear, float angular);
void eco_bt_body_aabb(eco_bt_body *body, eco_bt_aabb *out);
void eco_bt_body_activate(eco_bt_body *body);
void eco_bt_body_sleep(eco_bt_body *body);
void eco_bt_body_set_always_awake(eco_bt_body *body, uint32_t on);
uint32_t eco_bt_body_awake(eco_bt_body *body);
uint32_t eco_bt_body_in_world(eco_bt_body *body);

/*
 * 0 dynamic, 1 fixed, 2 kinematic.
 */
int32_t eco_bt_body_kind(eco_bt_body *body);
void eco_bt_body_set_kinematic(eco_bt_body *body, uint32_t on);
uint32_t eco_bt_body_sensor(eco_bt_body *body);
void eco_bt_body_set_sensor(eco_bt_body *body, uint32_t on);

/* constraints, common */

void eco_bt_constraint_destroy(eco_bt_constraint *constraint);
uint32_t eco_bt_constraint_enabled(eco_bt_constraint *constraint);
void eco_bt_constraint_set_enabled(eco_bt_constraint *constraint, uint32_t on);
float eco_bt_constraint_breaking_impulse(eco_bt_constraint *constraint);
void eco_bt_constraint_set_breaking_impulse(eco_bt_constraint *constraint, float impulse);
float eco_bt_constraint_applied_impulse(eco_bt_constraint *constraint);
void eco_bt_constraint_feedback(eco_bt_constraint *constraint, eco_bt_feedback *out);
void eco_bt_constraint_set_iterations(eco_bt_constraint *constraint, int32_t iterations);

/*
 * param: 1 ERP, 2 stop ERP, 3 CFM, 4 stop CFM. axis -1 is every axis.
 */
void eco_bt_constraint_set_param(eco_bt_constraint *constraint, int32_t param, float value, int32_t axis);
void eco_bt_constraint_set_debug_size(eco_bt_constraint *constraint, float size);

/*
 * Every create below takes b = NULL for a constraint between a and the
 * world. The b-side arguments are then ignored.
 */

/* point to point */

eco_bt_constraint *eco_bt_point_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *pivot_a,
    const eco_bt_vec3 *pivot_b
);
void eco_bt_point_set_pivots(eco_bt_constraint *point, const eco_bt_vec3 *pivot_a, const eco_bt_vec3 *pivot_b);
void eco_bt_point_get_pivots(eco_bt_constraint *point, eco_bt_vec3 *pivot_a, eco_bt_vec3 *pivot_b);
void eco_bt_point_set_tuning(eco_bt_constraint *point, float tau, float damping, float impulse_clamp);

/* hinge */

eco_bt_constraint *eco_bt_hinge_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *pivot_a,
    const eco_bt_vec3 *pivot_b,
    const eco_bt_vec3 *axis_a,
    const eco_bt_vec3 *axis_b
);
eco_bt_constraint *eco_bt_hinge_create_frames(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b
);
void eco_bt_hinge_set_limit(
    eco_bt_constraint *hinge,
    float low,
    float high,
    float softness,
    float bias,
    float relaxation
);
float eco_bt_hinge_low(eco_bt_constraint *hinge);
float eco_bt_hinge_high(eco_bt_constraint *hinge);
float eco_bt_hinge_angle(eco_bt_constraint *hinge);
void eco_bt_hinge_set_motor(eco_bt_constraint *hinge, uint32_t on, float velocity, float max_impulse);
void eco_bt_hinge_set_motor_target(eco_bt_constraint *hinge, float angle, float dt);
void eco_bt_hinge_set_axis(eco_bt_constraint *hinge, const eco_bt_vec3 *axis_a);
void eco_bt_hinge_set_frames(eco_bt_constraint *hinge, const eco_bt_transform *frame_a, const eco_bt_transform *frame_b);

/* slider */

eco_bt_constraint *eco_bt_slider_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b,
    uint32_t linear_frame_a
);
void eco_bt_slider_set_linear_limit(eco_bt_constraint *slider, float low, float high);
void eco_bt_slider_set_angular_limit(eco_bt_constraint *slider, float low, float high);
float eco_bt_slider_position(eco_bt_constraint *slider);
float eco_bt_slider_angle(eco_bt_constraint *slider);
void eco_bt_slider_set_linear_motor(eco_bt_constraint *slider, uint32_t on, float velocity, float max_force);
void eco_bt_slider_set_angular_motor(eco_bt_constraint *slider, uint32_t on, float velocity, float max_force);
void eco_bt_slider_set_frames(eco_bt_constraint *slider, const eco_bt_transform *frame_a, const eco_bt_transform *frame_b);

/* legacy six dof */

eco_bt_constraint *eco_bt_legacy_sixdof_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b,
    uint32_t linear_frame_a
);
void eco_bt_legacy_sixdof_set_linear(eco_bt_constraint *c, const eco_bt_vec3 *lower, const eco_bt_vec3 *upper);
void eco_bt_legacy_sixdof_set_angular(eco_bt_constraint *c, const eco_bt_vec3 *lower, const eco_bt_vec3 *upper);

/* six dof (spring 2), also hinge2 */

eco_bt_constraint *eco_bt_sixdof_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b
);
void eco_bt_sixdof_set_frames(eco_bt_constraint *c, const eco_bt_transform *frame_a, const eco_bt_transform *frame_b);
void eco_bt_sixdof_set_linear_lower(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_linear_upper(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_angular_lower(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_angular_lower_reversed(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_angular_upper(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_angular_upper_reversed(eco_bt_constraint *c, const eco_bt_vec3 *v);
void eco_bt_sixdof_set_limit(eco_bt_constraint *c, int32_t axis, float low, float high);
void eco_bt_sixdof_set_limit_reversed(eco_bt_constraint *c, int32_t axis, float low, float high);
void eco_bt_sixdof_set_axis(eco_bt_constraint *c, const eco_bt_vec3 *axis1, const eco_bt_vec3 *axis2);
void eco_bt_sixdof_set_bounce(eco_bt_constraint *c, int32_t index, float bounce);
void eco_bt_sixdof_enable_motor(eco_bt_constraint *c, int32_t index, uint32_t on);
void eco_bt_sixdof_set_servo(eco_bt_constraint *c, int32_t index, uint32_t on);
void eco_bt_sixdof_set_target_velocity(eco_bt_constraint *c, int32_t index, float velocity);
void eco_bt_sixdof_set_servo_target(eco_bt_constraint *c, int32_t index, float target);
void eco_bt_sixdof_set_max_motor_force(eco_bt_constraint *c, int32_t index, float force);
void eco_bt_sixdof_enable_spring(eco_bt_constraint *c, int32_t index, uint32_t on);
void eco_bt_sixdof_set_stiffness(eco_bt_constraint *c, int32_t index, float stiffness, uint32_t limit_if_needed);
void eco_bt_sixdof_set_damping(eco_bt_constraint *c, int32_t index, float damping, uint32_t limit_if_needed);
void eco_bt_sixdof_set_equilibrium(eco_bt_constraint *c);
float eco_bt_sixdof_angle(eco_bt_constraint *c, int32_t axis);
float eco_bt_sixdof_position(eco_bt_constraint *c, int32_t axis);

/* cone twist */

eco_bt_constraint *eco_bt_cone_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b
);
void eco_bt_cone_set_limit(
    eco_bt_constraint *cone,
    float swing1,
    float swing2,
    float twist,
    float softness,
    float bias,
    float relaxation
);
void eco_bt_cone_set_damping(eco_bt_constraint *cone, float damping);
void eco_bt_cone_set_motor(eco_bt_constraint *cone, uint32_t on, float max_impulse);
void eco_bt_cone_set_motor_target(eco_bt_constraint *cone, const eco_bt_quat *target);
void eco_bt_cone_set_frames(eco_bt_constraint *cone, const eco_bt_transform *frame_a, const eco_bt_transform *frame_b);
float eco_bt_cone_twist(eco_bt_constraint *cone);
float eco_bt_cone_swing1(eco_bt_constraint *cone);
float eco_bt_cone_swing2(eco_bt_constraint *cone);

/* fixed */

eco_bt_constraint *eco_bt_fixed_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_transform *frame_a,
    const eco_bt_transform *frame_b
);

/* gear: both bodies required */

eco_bt_constraint *eco_bt_gear_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *axis_a,
    const eco_bt_vec3 *axis_b,
    float ratio
);
float eco_bt_gear_get_ratio(eco_bt_constraint *gear);
void eco_bt_gear_set_ratio(eco_bt_constraint *gear, float ratio);

/* hinge2 and universal: world anchor and axes, both bodies required */

eco_bt_constraint *eco_bt_hinge2_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *anchor,
    const eco_bt_vec3 *axis1,
    const eco_bt_vec3 *axis2
);
void eco_bt_hinge2_set_steering(eco_bt_constraint *hinge2, float low, float high);

eco_bt_constraint *eco_bt_universal_create(
    eco_bt_body *a,
    eco_bt_body *b,
    const eco_bt_vec3 *anchor,
    const eco_bt_vec3 *axis1,
    const eco_bt_vec3 *axis2
);
void eco_bt_universal_set_limit(eco_bt_constraint *universal, float low1, float high1, float low2, float high2);
float eco_bt_universal_angle1(eco_bt_constraint *universal);
float eco_bt_universal_angle2(eco_bt_constraint *universal);

/* ghosts */

eco_bt_ghost *eco_bt_ghost_create(eco_bt_shape *shape, const eco_bt_transform *at);
void eco_bt_ghost_destroy(eco_bt_ghost *ghost);
uint64_t eco_bt_ghost_id(eco_bt_ghost *ghost);
void eco_bt_ghost_get_transform(eco_bt_ghost *ghost, eco_bt_transform *out);
void eco_bt_ghost_set_transform(eco_bt_ghost *ghost, const eco_bt_transform *at);
uint32_t eco_bt_ghost_in_world(eco_bt_ghost *ghost);

/*
 * Broadphase overlaps (bounding boxes), copied as ids. The ghost keeps
 * them, so copy before the next step.
 */
size_t eco_bt_ghost_overlaps(eco_bt_ghost *ghost);
void eco_bt_ghost_overlaps_copy(eco_bt_ghost *ghost, uint64_t *out, size_t count);

/*
 * Ids of the bodies with a narrowphase point at or below zero distance
 * inside the ghost, each once. Needs the world it is in for the
 * dispatcher. Copy with ghost_overlaps_copy.
 */
size_t eco_bt_ghost_touching(eco_bt_ghost *ghost, eco_bt_world *world);

/* kinematic character controller: shape must be convex */

eco_bt_character *eco_bt_character_create(
    eco_bt_shape *shape,
    const eco_bt_transform *at,
    float step_height,
    const eco_bt_vec3 *up
);
void eco_bt_character_destroy(eco_bt_character *character);
uint64_t eco_bt_character_id(eco_bt_character *character);
void eco_bt_character_get_transform(eco_bt_character *character, eco_bt_transform *out);
void eco_bt_character_warp(eco_bt_character *character, const eco_bt_vec3 *origin);
void eco_bt_character_walk(eco_bt_character *character, const eco_bt_vec3 *per_step);
void eco_bt_character_set_velocity(eco_bt_character *character, const eco_bt_vec3 *velocity, float seconds);
void eco_bt_character_get_velocity(eco_bt_character *character, eco_bt_vec3 *out);
void eco_bt_character_jump(eco_bt_character *character, const eco_bt_vec3 *velocity);
uint32_t eco_bt_character_can_jump(eco_bt_character *character);
uint32_t eco_bt_character_grounded(eco_bt_character *character);
void eco_bt_character_set_jump_speed(eco_bt_character *character, float speed);
void eco_bt_character_set_fall_speed(eco_bt_character *character, float speed);
void eco_bt_character_set_max_jump_height(eco_bt_character *character, float height);
void eco_bt_character_set_max_slope(eco_bt_character *character, float radians);
float eco_bt_character_get_max_slope(eco_bt_character *character);
void eco_bt_character_set_gravity(eco_bt_character *character, const eco_bt_vec3 *gravity);
void eco_bt_character_get_gravity(eco_bt_character *character, eco_bt_vec3 *out);
void eco_bt_character_set_step_height(eco_bt_character *character, float height);
float eco_bt_character_get_step_height(eco_bt_character *character);
void eco_bt_character_set_up(eco_bt_character *character, const eco_bt_vec3 *up);
void eco_bt_character_set_max_penetration(eco_bt_character *character, float depth);
uint32_t eco_bt_character_in_world(eco_bt_character *character);

/* raycast vehicle */

eco_bt_vehicle *eco_bt_vehicle_create(eco_bt_body *chassis);
void eco_bt_vehicle_destroy(eco_bt_vehicle *vehicle);
int32_t eco_bt_vehicle_add_wheel(eco_bt_vehicle *vehicle, const eco_bt_wheel *wheel);
int32_t eco_bt_vehicle_wheel_count(eco_bt_vehicle *vehicle);
void eco_bt_vehicle_wheel_state(eco_bt_vehicle *vehicle, int32_t wheel, eco_bt_wheel_state *out);
void eco_bt_vehicle_set_steering(eco_bt_vehicle *vehicle, int32_t wheel, float radians);
void eco_bt_vehicle_set_engine(eco_bt_vehicle *vehicle, int32_t wheel, float force);
void eco_bt_vehicle_set_brake(eco_bt_vehicle *vehicle, int32_t wheel, float force);
float eco_bt_vehicle_speed(eco_bt_vehicle *vehicle);
void eco_bt_vehicle_forward(eco_bt_vehicle *vehicle, eco_bt_vec3 *out);
void eco_bt_vehicle_set_axes(eco_bt_vehicle *vehicle, int32_t right, int32_t up, int32_t forward);
void eco_bt_vehicle_reset(eco_bt_vehicle *vehicle);
uint32_t eco_bt_vehicle_in_world(eco_bt_vehicle *vehicle);

/* soft bodies */

eco_bt_soft *eco_bt_soft_rope(const eco_bt_vec3 *from, const eco_bt_vec3 *to, int32_t segments);
eco_bt_soft *eco_bt_soft_cloth(
    const eco_bt_vec3 *c00,
    const eco_bt_vec3 *c10,
    const eco_bt_vec3 *c01,
    const eco_bt_vec3 *c11,
    int32_t rx,
    int32_t ry,
    uint32_t diagonals
);
eco_bt_soft *eco_bt_soft_ellipsoid(const eco_bt_vec3 *centre, const eco_bt_vec3 *radius, int32_t resolution);
eco_bt_soft *eco_bt_soft_mesh(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
);
eco_bt_soft *eco_bt_soft_hull(const eco_bt_vec3 *points, int32_t count);
void eco_bt_soft_destroy(eco_bt_soft *soft);
uint64_t eco_bt_soft_id(eco_bt_soft *soft);
uint32_t eco_bt_soft_in_world(eco_bt_soft *soft);
int32_t eco_bt_soft_node_count(eco_bt_soft *soft);
void eco_bt_soft_nodes(eco_bt_soft *soft, eco_bt_vec3 *out, size_t count);
void eco_bt_soft_normals(eco_bt_soft *soft, eco_bt_vec3 *out, size_t count);
int32_t eco_bt_soft_face_count(eco_bt_soft *soft);
void eco_bt_soft_faces(eco_bt_soft *soft, int32_t *out, size_t count);
int32_t eco_bt_soft_link_count(eco_bt_soft *soft);
void eco_bt_soft_links(eco_bt_soft *soft, int32_t *out, size_t count);
float eco_bt_soft_get_mass(eco_bt_soft *soft);
void eco_bt_soft_set_mass(eco_bt_soft *soft, float mass, uint32_t from_faces);
float eco_bt_soft_get_node_mass(eco_bt_soft *soft, int32_t node);
void eco_bt_soft_set_node_mass(eco_bt_soft *soft, int32_t node, float mass);
void eco_bt_soft_anchor(eco_bt_soft *soft, int32_t node, eco_bt_body *body, uint32_t collide, float influence);
void eco_bt_soft_set_stiffness(eco_bt_soft *soft, float linear, float angular, float volume);
void eco_bt_soft_get_config(eco_bt_soft *soft, eco_bt_soft_config *out);
void eco_bt_soft_set_config(eco_bt_soft *soft, const eco_bt_soft_config *config);

/*
 * mode: 0 vertex/face against rigid (SDF), 1 clusters. soft_soft adds
 * collisions with other soft bodies, self adds self collision (clusters).
 */
void eco_bt_soft_set_collision(eco_bt_soft *soft, int32_t mode, uint32_t soft_soft, uint32_t self);
void eco_bt_soft_bend(eco_bt_soft *soft, int32_t distance);
void eco_bt_soft_randomize(eco_bt_soft *soft);
void eco_bt_soft_clusters(eco_bt_soft *soft, int32_t count);
void eco_bt_soft_set_pose(eco_bt_soft *soft, uint32_t volume, uint32_t frame);
void eco_bt_soft_set_wind(eco_bt_soft *soft, const eco_bt_vec3 *velocity);
void eco_bt_soft_apply_force(eco_bt_soft *soft, const eco_bt_vec3 *force);
void eco_bt_soft_apply_node_force(eco_bt_soft *soft, int32_t node, const eco_bt_vec3 *force);
void eco_bt_soft_set_velocity(eco_bt_soft *soft, const eco_bt_vec3 *velocity);
void eco_bt_soft_translate(eco_bt_soft *soft, const eco_bt_vec3 *offset);
void eco_bt_soft_rotate(eco_bt_soft *soft, const eco_bt_quat *rotation);
void eco_bt_soft_scale(eco_bt_soft *soft, const eco_bt_vec3 *scale);
void eco_bt_soft_aabb(eco_bt_soft *soft, eco_bt_aabb *out);
float eco_bt_soft_volume(eco_bt_soft *soft);
void eco_bt_soft_activate(eco_bt_soft *soft);

#ifdef __cplusplus
}
#endif

#endif
