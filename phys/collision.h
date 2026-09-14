#pragma once

#include "phys/body.h"

namespace phys
{

    struct Contact{
        Vec3 position{0.0f, 0.0f, 0.0f};
        float penetration = 0.0f;

        Vec3 localA{0.0f, 0.0f, 0.0f};
        Vec3 localB{0.0f, 0.0f, 0.0f};

        float normalImpulse = 0.0f;
        float tangentImpulse[2] = {0.0f, 0.0f};

        Vec3 rA{0.0f, 0.0f, 0.0f}, rB{0.0f, 0.0f, 0.0f};
        float normalMass = 0.0f;
        float tangentMass[2] = {0.0f, 0.0f};
        float velocityBias = 0.0f;
    };

    struct Manifold
    {
        RigidBody *bodyA = nullptr;
        RigidBody *bodyB = nullptr;

        Vec3 normal{0.0f, 1.0f, 0.0f};
        Vec3 tangent[2];

        Contact contacts[4];

        int count=0;

        float restitution = 0.0f;
        float friction = 0.0f;

        float rollingFriction = 0.0f;
        float rollingFrictionImpulse[3]= {0.0f, 0.0f, 0.0f};


    };


    bool collide(RigidBody* a, RigidBody* b, Manifold& manifold);

}