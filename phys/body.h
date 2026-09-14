#pragma once

#include "phys/math3d.h"

namespace phys
{

    enum class ShapeType
    {
        Box,
        Sphere,
    };

    struct Shape
    {
        ShapeType type = ShapeType::Box;
        float radius = 0.5f;                // Sphere用
        Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box用

        static Shape sphere(float r)
        {
            Shape s;
            s.type = ShapeType::Sphere;
            s.radius = r;
            return s;
        }

        static Shape box(const Vec3 &halfExtents)
        {
            Shape s;
            s.type = ShapeType::Box;
            s.halfExtents = halfExtents;
            return s;
        }
    };

    struct RigidBody
    {
        int id = 0;

        Vec3 position{0.0f, 0.0f, 0.0f};
        Quat orientation{};
        Vec3 velocity{0.0f, 0.0f, 0.0f};
        Vec3 angularVelocity{0.0f, 0.0f, 0.0f};

        Vec3 force{0.0f, 0.0f, 0.0f};
        Vec3 torque{0.0f, 0.0f, 0.0f};

        float invMass = 1.0f;
        Vec3 invInertiaLocal{1.0f, 1.0f, 1.0f};  // ローカル座標系での逆慣性テンソルの対角成分
        Mat3 invInertiaWorld = Mat3::identity(); // ワールド座標系での逆慣性テンソル

        float restitution = 0.2f;
        float friction = 0.5f;
        float rollingFriction = 0.02f;
        float linearDamping = 0.01f;
        float angularDamping = 0.01f;

        Shape shape;

        bool isStatic() const { return invMass == 0.0f; }

        void setMass(float mass)
        {
            if (mass <= 0.0f)
            {
                invMass = 0.0f;
                invInertiaLocal = Vec3{0.0f, 0.0f, 0.0f};
                invInertiaWorld = Mat3::zero();
                return;
            }

            invMass = 1.0f / mass;

            // 慣性モーメントの計算
            Vec3 I{1, 1, 1};
            if (shape.type == ShapeType::Sphere)
            {
                float v = 0.4f * mass * shape.radius * shape.radius;
                I = Vec3{v, v, v};
            }
            else if (shape.type == ShapeType::Box)
            {
                Vec3 s = shape.halfExtents * 2.0f;
                float k = mass / 12.0f;
                I = Vec3{k * (s.y * s.y + s.z * s.z), k * (s.x * s.x + s.z * s.z), k * (s.x * s.x + s.y * s.y)};
            }

            invInertiaLocal = Vec3{1.0f / I.x, 1.0f / I.y, 1.0f / I.z};
            updateInertiaWorld();
        }

        void updateInertiaWorld()
        {
            if (isStatic())
            {
                invInertiaWorld = Mat3::zero();
                return;
            }

            Mat3 R = orientation.toMat3();
            Mat3 RD{R.c0 * invInertiaLocal.x, R.c1 * invInertiaLocal.y, R.c2 * invInertiaLocal.z};
            invInertiaWorld = RD * R.transpose();
        }

        void applyForce(const Vec3 &f)
        {
            force += f;
        }

        void applyForceAtPoint(const Vec3 &f, const Vec3 &point)
        {
            force += f;
            torque += cross(point - position, f);
        }

        void applyImpulse(const Vec3 &impulse, const Vec3 &r)
        {
            if (isStatic())
                return;
            velocity += impulse * invMass;
            angularVelocity += invInertiaWorld * cross(r, impulse);
        }

        void applyImpulseAtcenter(const Vec3 &impulse, const Vec3 &point)
        {
            if (isStatic())
                return;
            velocity += impulse * invMass;
        }

        Vec3 velocityAt(const Vec3 &r) const
        {
            return velocity + cross(angularVelocity, r);
        }

        AABB computeAABB() const
        {
            AABB box;
            if (shape.type == ShapeType::Sphere)
            {

                Vec3 r{shape.radius, shape.radius, shape.radius};
                box.min = position - r;
                box.max = position + r;
            }else if(shape.type == ShapeType::Box)
            {
                 // 回転した OBB を包む AABB：|R| * halfExtents
                Mat3 R = orientation.toMat3();
                Vec3 r{std::fabs(R.c0.x) * shape.halfExtents.x + std::fabs(R.c1.x) * shape.halfExtents.y + std::fabs(R.c2.x) * shape.halfExtents.z,
                       std::fabs(R.c0.y) * shape.halfExtents.x + std::fabs(R.c1.y) * shape.halfExtents.y + std::fabs(R.c2.y) * shape.halfExtents.z,
                       std::fabs(R.c0.z) * shape.halfExtents.x + std::fabs(R.c1.z) * shape.halfExtents.y + std::fabs(R.c2.z) * shape.halfExtents.z};
                box.min = position - r;
                box.max = position + r;
            }

            return box;
        }
    };
}