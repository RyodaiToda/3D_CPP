#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include "math3d.h"
#include "collision.h"
#include <unordered_map>

namespace phys {

    class World {
    public:
        Vec3 gravity{0.0f, -9.81f, 0.0f};

        RigidBody* createBox(const Vec3& position, const Vec3& halfExtents, float mass);
        RigidBody* createSphere(const Vec3& position, float radius, float mass);

        void clear();
        void step(float dt);

    private:
        void integrateForces(float dt);
        void integratePositions(float dt);
        void solveVelocities();
        void correctPositions();


        std::vector<std::unique_ptr<RigidBody>> bodies_;
        std::vector<std::pair<int,int>> pairs_;
        std::vector<AABB> aabbs_;
        std::vector<Manifold> manifolds_;
        std::unordered_map<uint64_t, Manifold> prevManifolds_;

    };
}