#include "phys/world.h"
#include <cmath>

namespace phys
{
    static inline uint64_t pairKey(int a, int b)
    {
        return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
    }

    RigidBody *World::createBox(const Vec3 &position, const Vec3 &halfExtents, float mass)
    {
        auto body = std::make_unique<RigidBody>();
        body->id = nextId_++;
        body->position = position;
        body->shape = Shape::box(halfExtents);
        body->setMass(mass);
        RigidBody *raw = body.get();
        bodies_.push_back(std::move(body));
        return raw;
    }

    RigidBody *World::createSphere(const Vec3 &position, float radius, float mass)
    {
        auto body = std::make_unique<RigidBody>();
        body->id = nextId_++;
        body->position = position;
        body->shape = Shape::sphere(radius);
        body->setMass(mass);
        RigidBody *raw = body.get();
        bodies_.push_back(std::move(body));
        return raw;
    }

    void World::clear()
    {
        bodies_.clear();
        manifolds_.clear();
        prevManifolds_.clear();
        pairs_.clear();
        aabbs_.clear();
        nextId_ = 0;
    }

    void World::integrateForces(float dt)
    {
        for (auto &b : bodies_)
        {
            b->updateInertiaWorld();
            if (b->isStatic())
            {
                b->velocity = Vec3{0, 0, 0};
                b->angularVelocity = Vec3{0, 0, 0};
                b->force = Vec3(0, 0, 0);
                b->torque = Vec3(0, 0, 0);
                continue;
            }
            b->velocity += (gravity + b->force * b->invMass) * dt;
            b->angularVelocity += b->invInertiaWorld * b->torque * dt;

            b->velocity *= 1.0f / (1.0f + dt * b->linearDamping);
            b->angularVelocity *= 1.0f / (1.0f + dt * b->angularDamping);
        }
    }

    void World::broadphase()
    {
        const int n = (int)bodies_.size();
        aabbs_.resize(n);
        for (int i = 0; i < n; i++)
        {
            aabbs_[i] = bodies_[i]->computeAABB();
            aabbs_[i].expand(0.02f);
        }

        pairs_.clear();
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (bodies_[i]->isStatic() && bodies_[j]->isStatic())
                    continue;
                if (!aabbs_[i].overlaps(aabbs_[j]))
                    continue;
                pairs_.emplace_back(i, j);
            }
        }
    }

    void World::narrowphase()
    {
        manifolds_.clear();
        Manifold m;
        for (const auto& p : pairs_)
        {
            m = Manifold{};
            if (collide(bodies_[p.first].get(), bodies_[p.second].get(), m))
            {
                manifolds_.push_back(m);
            }
        }
    }

    static inline float effectiveMass(const RigidBody *A, const RigidBody *B, const Vec3 &rA, const Vec3 &rB, const Vec3 &d)
    {
        Vec3 rAxd = cross(rA, d);
        Vec3 rBxd = cross(rB, d);
        float k = A->invMass + B->invMass + dot(A->invInertiaWorld * rAxd, rAxd) + dot(B->invInertiaWorld * rBxd, rBxd);
        return (k > 1e-9f) ? 1.0f / k : 0.0f;
    }

    void World::prepareConstraints(float dt)
    {
        for (auto &m : manifolds_)
        {
            RigidBody *A = m.bodyA;
            RigidBody *B = m.bodyB;

            for (int i = 0; i < m.count; ++i)
            {
                Contact &c = m.contacts[i];
                c.rA = c.position - A->position;
                c.rB = c.position - B->position;

                c.normalMass = effectiveMass(A, B, c.rA, c.rB, m.normal);
                c.tangentMass[0] = effectiveMass(A, B, c.rA, c.rB, m.tangent[0]);
                c.tangentMass[1] = effectiveMass(A, B, c.rA, c.rB, m.tangent[1]);

                Vec3 relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                float vn = dot(relVel, m.normal);
                c.velocityBias = (vn < -restitutionThreshold) ? m.restitution * vn : 0.0f;
            }
        }
    }

    void World::solveVelocities()
    {
        for (int iter = 0; iter < velocityIterations; iter++)
        {
            for (auto &m : manifolds_)
            {
                RigidBody *A = m.bodyA;
                RigidBody *B = m.bodyB;

                for (int i = 0; i < m.count; i++)
                {
                    Contact &c = m.contacts[i];

                    // 摩擦
                    float maxFriction = m.friction * c.normalImpulse;
                    for (int k = 0; k < 2; k++)
                    {
                        Vec3 relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                        float vt = dot(relVel, m.tangent[k]);
                        float lambda = -vt * c.tangentMass[k];

                        float oldImpulse = c.tangentImpulse[k];
                        float newImpulse = clampf(oldImpulse + lambda, -maxFriction, maxFriction);
                        lambda = newImpulse - oldImpulse;
                        c.tangentImpulse[k] = newImpulse;

                        Vec3 P = m.tangent[k] * lambda;
                        A->applyImpulse(-1.0f * P, c.rA);
                        B->applyImpulse(P, c.rB);
                    }

                    // 法線方向
                    Vec3 relVel = B->velocityAt(c.rB) - A->velocityAt(c.rA);
                    float vn = dot(relVel, m.normal);
                    float lambda = -(vn + c.velocityBias) * c.normalMass;

                    float oldImpulse = c.normalImpulse;
                    float newImpulse = std::max(oldImpulse + lambda, 0.0f);
                    lambda = newImpulse - oldImpulse;
                    c.normalImpulse = newImpulse;

                    Vec3 P = m.normal * lambda;
                    A->applyImpulse(-1.0f * P, c.rA);
                    B->applyImpulse(P, c.rB);
                }

                // 転がり摩擦
                if (m.rollingFriction > 0.0f)
                {
                    float totalPn = 0.0f;
                    for (int i = 0; i < m.count; i++)
                    {
                        totalPn += m.contacts[i].normalImpulse;
                    }
                    float maxRoll = m.rollingFriction * totalPn;
                    if (maxRoll > 0.0f)
                    {
                        Mat3 invISum = A->invInertiaWorld + B->invInertiaWorld;
                        for (int k = 0; k < 3; k++)
                        {
                            Vec3 axis = (k == 0) ? m.normal : m.tangent[k - 1];

                            float kAng = dot(axis, invISum * axis);
                            if (kAng < 1e-9f)
                                continue;

                            Vec3 relW = B->angularVelocity - A->angularVelocity;
                            float lambda = -dot(relW, axis) / kAng;

                            float oldImpulse = m.rollingFrictionImpulse[k];
                            float newImpulse = clampf(oldImpulse + lambda, -maxRoll, maxRoll);
                            lambda = newImpulse - oldImpulse;
                            m.rollingFrictionImpulse[k] = newImpulse;

                            Vec3 L = axis * lambda;
                            A->angularVelocity -= A->invInertiaWorld * L;
                            B->angularVelocity += B->invInertiaWorld * L;
                        }
                    }
                }
            }
        }
    }

    void World::integratePositions(float dt)
    {
        for (auto &b : bodies_)
        {
            if (b->isStatic())
                continue;

            b->position += b->velocity * dt;
            b->orientation.integrate(b->angularVelocity, dt);
            b->force = Vec3{0, 0, 0};
            b->torque = Vec3{0, 0, 0};
        }
    }

    void World::correctPositions()
    {
        const float maxCorrection = 0.2f;

        for (int iter = 0; iter < positionIterations; iter++)
        {
            for (auto &m : manifolds_)
            {
                RigidBody *A = m.bodyA;
                RigidBody *B = m.bodyB;
                float invMassSum = A->invMass + B->invMass;
                if (invMassSum <= 0.0f)
                    continue;

                Mat3 RA = A->orientation.toMat3();
                Mat3 RB = B->orientation.toMat3();

                for (int i = 0; i < m.count; i++)
                {
                    const Contact &c = m.contacts[i];

                    Vec3 pA = A->position + RA * c.localA;
                    Vec3 pB = B->position + RB * c.localB;
                    float pen = c.penetration - dot(pB - pA, m.normal);

                    float depth = pen - penetrationSlop;
                    if (depth <= 0.0f)
                        continue;

                    float amount = clampf(depth * positionCorrection, 0.0f, maxCorrection) / invMassSum;
                    Vec3 corr = m.normal * amount;
                    A->position -= corr * A->invMass;
                    B->position += corr * B->invMass;
                }
            }
        }
    }

    void World::warmStart()
    {
        if (!warmStarting)
        {
            prevManifolds_.clear();
            return;
        }

        const float matchDistSq = 0.01f * 0.01f;

        for (auto &m : manifolds_)
        {
            auto it = prevManifolds_.find(pairKey(m.bodyA->id, m.bodyB->id));
            if (it == prevManifolds_.end())
                continue;

            const Manifold &old = it->second;

            for (int i = 0; i < m.count; i++)
            {
                for (int j = 0; j < old.count; j++)
                {
                    if (lengthSq(m.contacts[i].localA - old.contacts[j].localA) < matchDistSq &&
                        lengthSq(m.contacts[i].localB - old.contacts[j].localB) < matchDistSq)
                    {
                        m.contacts[i].normalImpulse = old.contacts[j].normalImpulse;
                        m.contacts[i].tangentImpulse[0] = old.contacts[j].tangentImpulse[0];
                        m.contacts[i].tangentImpulse[1] = old.contacts[j].tangentImpulse[1];

                        Contact &c = m.contacts[i];
                        Vec3 impulse = m.normal * c.normalImpulse +
                                       m.tangent[0] * c.tangentImpulse[0] +
                                       m.tangent[1] * c.tangentImpulse[1];
                        m.bodyA->applyImpulse(-1.0f * impulse, c.rA);
                        m.bodyB->applyImpulse(impulse, c.rB);
                        break;
                    }
                }
            }
        }
    }

    void World::step(float dt)
    {
        if (dt <= 0.0)
        {
            return;
        }

        integrateForces(dt);
        broadphase();
        narrowphase();
        prepareConstraints(dt);
        warmStart();
        solveVelocities();
        integratePositions(dt);
        correctPositions();

        prevManifolds_.clear();
        for (const auto &m : manifolds_)
        {
            prevManifolds_[pairKey(m.bodyA->id, m.bodyB->id)] = m;
        }
    }

}