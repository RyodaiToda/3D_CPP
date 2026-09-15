#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include "math3d.h"
#include "collision.h"
#include <unordered_map>

namespace phys
{

    class World
    {
    public:
        Vec3 gravity{0.0f, -9.81f, 0.0f};
        float restitutionThreshold = 1.0f; // これ未満の接近速度では反発させない
        int velocityIterations = 10;
        int positionIterations = 3;      // めり込み補正の反復回数
        float penetrationSlop = 0.005f;  // 許容するめり込み量（ジッタ防止）
        float positionCorrection = 0.4f; // 1 反復で解消する割合

        bool warmStarting = true;

        RigidBody *createBox(const Vec3 &position, const Vec3 &halfExtents, float mass);
        RigidBody *createSphere(const Vec3 &position, float radius, float mass);

        void clear();
        void step(float dt);

        const std::vector<std::unique_ptr<RigidBody>> &getBodies() const { return bodies_; }
        const std::vector<Manifold> &getManifolds() const { return manifolds_; }

        int bodyCount() const { return (int)bodies_.size(); }
        int contactCount() const;

    private:
        void integrateForces(float dt);
        void integratePositions(float dt);
        void solveVelocities();
        void correctPositions();
        void prepareConstraints(float dt);
        void broadphase();
        void narrowphase();
        void warmStart();

        std::vector<std::unique_ptr<RigidBody>> bodies_;
        std::vector<std::pair<int, int>> pairs_;
        std::vector<AABB> aabbs_;
        std::vector<Manifold> manifolds_;
        std::unordered_map<uint64_t, Manifold> prevManifolds_;

        int nextId_ = 0;
    };
}