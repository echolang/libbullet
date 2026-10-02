#include "internal.h"

/*
 * Soft bodies. Bullet binds a soft body to a btSoftBodyWorldInfo when it
 * is built, and reads it on every step. Each one here starts out bound to
 * its own copy, and World::add rebinds it to the world's, so building one
 * needs no world and a world dying first leaves nothing dangling.
 */

/*
 * A fresh soft body with its own world info, built by `build` from it.
 */
template <typename Build>
static eco_bt_soft *make(Build build)
{
    eco_bt_soft *s = new (std::nothrow) eco_bt_soft();
    if (s == nullptr) {
        return nullptr;
    }

    s->info.m_sparsesdf.Initialize();
    s->soft = build(s->info);
    tag(s->soft, &s->obj, ECO_BT_SOFT);
    return s;
}

/*
 * `segments` links, so segments + 1 nodes, node 0 at `from`.
 */
eco_bt_soft *eco_bt_soft_rope(const eco_bt_vec3 *from, const eco_bt_vec3 *to, int32_t segments)
{
    return make([&](btSoftBodyWorldInfo &info) {
        return btSoftBodyHelpers::CreateRope(info, to_bt(from), to_bt(to), segments - 1, 0);
    });
}

/*
 * rx by ry nodes, row by row from c00 towards c10, rows stepping towards
 * c01.
 */
eco_bt_soft *eco_bt_soft_cloth(
    const eco_bt_vec3 *c00,
    const eco_bt_vec3 *c10,
    const eco_bt_vec3 *c01,
    const eco_bt_vec3 *c11,
    int32_t rx,
    int32_t ry,
    uint32_t diagonals
)
{
    return make([&](btSoftBodyWorldInfo &info) {
        return btSoftBodyHelpers::CreatePatch(
            info, to_bt(c00), to_bt(c10), to_bt(c01), to_bt(c11), rx, ry, 0, diagonals != 0);
    });
}

eco_bt_soft *eco_bt_soft_ellipsoid(const eco_bt_vec3 *centre, const eco_bt_vec3 *radius, int32_t resolution)
{
    return make([&](btSoftBodyWorldInfo &info) {
        return btSoftBodyHelpers::CreateEllipsoid(info, to_bt(centre), to_bt(radius), resolution);
    });
}

eco_bt_soft *eco_bt_soft_mesh(
    const eco_bt_vec3 *vertices,
    int32_t vertex_count,
    const int32_t *indices,
    int32_t index_count
)
{
    (void)vertex_count;

    return make([&](btSoftBodyWorldInfo &info) {
        return btSoftBodyHelpers::CreateFromTriMesh(
            info, reinterpret_cast<const btScalar *>(vertices), indices, index_count / 3, false);
    });
}

eco_bt_soft *eco_bt_soft_hull(const eco_bt_vec3 *points, int32_t count)
{
    return make([&](btSoftBodyWorldInfo &info) {
        std::vector<btVector3> at = to_bt(points, count);
        return btSoftBodyHelpers::CreateFromConvexHull(info, at.data(), count, false);
    });
}

void eco_bt_soft_destroy(eco_bt_soft *s)
{
    if (s == nullptr) {
        return;
    }

    delete s->soft;
    delete s;
}

uint64_t eco_bt_soft_id(eco_bt_soft *s)
{
    return s->obj.id;
}

uint32_t eco_bt_soft_in_world(eco_bt_soft *s)
{
    return s->soft->getBroadphaseHandle() != nullptr ? 1u : 0u;
}

int32_t eco_bt_soft_node_count(eco_bt_soft *s)
{
    return s->soft->m_nodes.size();
}

void eco_bt_soft_nodes(eco_bt_soft *s, eco_bt_vec3 *out, size_t count)
{
    size_t n = (size_t)s->soft->m_nodes.size();
    for (size_t i = 0; i < n && i < count; i++) {
        store(&out[i], s->soft->m_nodes[(int)i].m_x);
    }
}

void eco_bt_soft_normals(eco_bt_soft *s, eco_bt_vec3 *out, size_t count)
{
    size_t n = (size_t)s->soft->m_nodes.size();
    for (size_t i = 0; i < n && i < count; i++) {
        store(&out[i], s->soft->m_nodes[(int)i].m_n);
    }
}

static int32_t index_of(btSoftBody *soft, const btSoftBody::Node *node)
{
    return (int32_t)(node - &soft->m_nodes[0]);
}

int32_t eco_bt_soft_face_count(eco_bt_soft *s)
{
    return s->soft->m_faces.size();
}

void eco_bt_soft_faces(eco_bt_soft *s, int32_t *out, size_t count)
{
    size_t k = 0;
    for (int i = 0; i < s->soft->m_faces.size(); i++) {
        for (int j = 0; j < 3 && k < count; j++) {
            out[k++] = index_of(s->soft, s->soft->m_faces[i].m_n[j]);
        }
    }
}

int32_t eco_bt_soft_link_count(eco_bt_soft *s)
{
    return s->soft->m_links.size();
}

void eco_bt_soft_links(eco_bt_soft *s, int32_t *out, size_t count)
{
    size_t k = 0;
    for (int i = 0; i < s->soft->m_links.size(); i++) {
        for (int j = 0; j < 2 && k < count; j++) {
            out[k++] = index_of(s->soft, s->soft->m_links[i].m_n[j]);
        }
    }
}

float eco_bt_soft_get_mass(eco_bt_soft *s)
{
    return s->soft->getTotalMass();
}

void eco_bt_soft_set_mass(eco_bt_soft *s, float mass, uint32_t from_faces)
{
    s->soft->setTotalMass(mass, from_faces != 0);
}

float eco_bt_soft_get_node_mass(eco_bt_soft *s, int32_t node)
{
    return s->soft->getMass(node);
}

void eco_bt_soft_set_node_mass(eco_bt_soft *s, int32_t node, float mass)
{
    s->soft->setMass(node, mass);
}

void eco_bt_soft_anchor(eco_bt_soft *s, int32_t node, eco_bt_body *body, uint32_t collide, float influence)
{
    s->soft->appendAnchor(node, body->body, collide == 0, influence);
}

/*
 * Material 0 is the one every link and face starts out with.
 */
void eco_bt_soft_set_stiffness(eco_bt_soft *s, float linear, float angular, float volume)
{
    btSoftBody::Material *m = s->soft->m_materials[0];
    m->m_kLST = linear;
    m->m_kAST = angular;
    m->m_kVST = volume;
}

void eco_bt_soft_get_config(eco_bt_soft *s, eco_bt_soft_config *out)
{
    const btSoftBody::Config &c = s->soft->m_cfg;
    out->damping = c.kDP;
    out->drag = c.kDG;
    out->lift = c.kLF;
    out->pressure = c.kPR;
    out->volume = c.kVC;
    out->friction = c.kDF;
    out->pose = c.kMT;
    out->rigid_hardness = c.kCHR;
    out->kinetic_hardness = c.kKHR;
    out->soft_hardness = c.kSHR;
    out->anchor_hardness = c.kAHR;
    out->position_iterations = c.piterations;
    out->velocity_iterations = c.viterations;
    out->drift_iterations = c.diterations;
    out->cluster_iterations = c.citerations;
}

void eco_bt_soft_set_config(eco_bt_soft *s, const eco_bt_soft_config *in)
{
    btSoftBody::Config &c = s->soft->m_cfg;
    c.kDP = in->damping;
    c.kDG = in->drag;
    c.kLF = in->lift;
    c.kPR = in->pressure;
    c.kVC = in->volume;
    c.kDF = in->friction;
    c.kMT = in->pose;
    c.kCHR = in->rigid_hardness;
    c.kKHR = in->kinetic_hardness;
    c.kSHR = in->soft_hardness;
    c.kAHR = in->anchor_hardness;
    c.piterations = in->position_iterations;
    c.viterations = in->velocity_iterations;
    c.diterations = in->drift_iterations;
    c.citerations = in->cluster_iterations;
}

void eco_bt_soft_set_collision(eco_bt_soft *s, int32_t mode, uint32_t soft_soft, uint32_t self)
{
    int flags = mode == 1 ? btSoftBody::fCollision::CL_RS : btSoftBody::fCollision::SDF_RS;

    if (soft_soft) {
        flags |= mode == 1 ? btSoftBody::fCollision::CL_SS : btSoftBody::fCollision::VF_SS;
    }

    if (self) {
        flags |= btSoftBody::fCollision::CL_SELF;
    }

    s->soft->m_cfg.collisions = flags;
}

void eco_bt_soft_bend(eco_bt_soft *s, int32_t distance)
{
    s->soft->generateBendingConstraints(distance, s->soft->m_materials[0]);
}

void eco_bt_soft_randomize(eco_bt_soft *s)
{
    s->soft->randomizeConstraints();
}

void eco_bt_soft_clusters(eco_bt_soft *s, int32_t count)
{
    s->soft->generateClusters(count);
}

void eco_bt_soft_set_pose(eco_bt_soft *s, uint32_t volume, uint32_t frame)
{
    s->soft->setPose(volume != 0, frame != 0);
}

void eco_bt_soft_set_wind(eco_bt_soft *s, const eco_bt_vec3 *velocity)
{
    s->soft->setWindVelocity(to_bt(velocity));
}

void eco_bt_soft_apply_force(eco_bt_soft *s, const eco_bt_vec3 *force)
{
    s->soft->activate(true);
    s->soft->addForce(to_bt(force));
}

void eco_bt_soft_apply_node_force(eco_bt_soft *s, int32_t node, const eco_bt_vec3 *force)
{
    s->soft->activate(true);
    s->soft->addForce(to_bt(force), node);
}

void eco_bt_soft_set_velocity(eco_bt_soft *s, const eco_bt_vec3 *velocity)
{
    s->soft->activate(true);
    s->soft->setVelocity(to_bt(velocity));
}

void eco_bt_soft_translate(eco_bt_soft *s, const eco_bt_vec3 *offset)
{
    s->soft->translate(to_bt(offset));
}

void eco_bt_soft_rotate(eco_bt_soft *s, const eco_bt_quat *rotation)
{
    s->soft->rotate(to_bt(rotation));
}

void eco_bt_soft_scale(eco_bt_soft *s, const eco_bt_vec3 *scale)
{
    s->soft->scale(to_bt(scale));
}

void eco_bt_soft_aabb(eco_bt_soft *s, eco_bt_aabb *out)
{
    btVector3 min;
    btVector3 max;
    s->soft->getAabb(min, max);
    store(out, min, max);
}

float eco_bt_soft_volume(eco_bt_soft *s)
{
    return s->soft->getVolume();
}

void eco_bt_soft_activate(eco_bt_soft *s)
{
    s->soft->activate(true);
}
