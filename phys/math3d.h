#pragma once

#include <cmath>
#include <algorithm>

namespace phys
{

    constexpr float PHYS_PI = 3.14159265358979323846f;

    struct Vec3
    {
        float x = 0.0f, y = 0.0f, z = 0.0f;

        Vec3() = default;
        Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

        Vec3 operator+(const Vec3 &rhs) const { return Vec3{x + rhs.x, y + rhs.y, z + rhs.z}; }
        Vec3 operator-(const Vec3 &rhs) const { return Vec3{x - rhs.x, y - rhs.y, z - rhs.z}; }
        Vec3 operator*(float s) const { return Vec3{x * s, y * s, z * s}; }
        Vec3 operator/(float s) const { return Vec3{x / s, y / s, z / s}; }

        Vec3 &operator+=(const Vec3 &rhs)
        {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }
        Vec3 &operator-=(const Vec3 &rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }
        Vec3 &operator*=(float s)
        {
            x *= s;
            y *= s;
            z *= s;
            return *this;
        }

        float operator[](int i) const { return (&x)[i]; }
        float &operator[](int i) { return (&x)[i]; }
    };

    inline Vec3 operator*(float s, const Vec3 &v) { return v * s; }
    inline float dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3 cross(const Vec3 &a, const Vec3 &b)
    {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    inline float lengthSq(const Vec3 &v) { return dot(v, v); }
    inline float length(const Vec3 &v) { return std::sqrt(dot(v, v)); }

    inline Vec3 normalize(const Vec3 &v)
    {
        float len = length(v);
        return (len > 1e-8f) ? v / len : Vec3{0, 0, 0};
    }

    inline Vec3 absVec(const Vec3 &v)
    {
        return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)};
    }

    // n に直交する正規直交な接線 2 本を作る（摩擦の 2 方向に使う）
    inline void buildTangents(const Vec3 &n, Vec3 &t1, Vec3 &t2)
    {
        // n の成分のうち最も小さい軸を選ぶと数値的に安定
        if (std::fabs(n.x) >= 0.57735f)
            t1 = normalize(Vec3{n.y, -n.x, 0.0f});
        else
            t1 = normalize(Vec3{0.0f, n.z, -n.y});
        t2 = cross(n, t1);
    }

    struct Mat3
    {
        Vec3 c0{1, 0, 0}, c1{0, 1, 0}, c2{0, 0, 1};

        Mat3() = default;
        Mat3(const Vec3 &a, const Vec3 &b, const Vec3 &c) : c0(a), c1(b), c2(c) {}

        const Vec3 &col(int i) const { return (i == 0) ? c0 : (i == 1) ? c1
                                                                       : c2; }

        Vec3 operator*(const Vec3 &v) const
        {
            return c0 * v.x + c1 * v.y + c2 * v.z;
        }

        Mat3 operator*(const Mat3 &o) const
        {
            return {(*this) * o.c0, (*this) * o.c1, (*this) * o.c2};
        }

        Mat3 transpose() const
        {
            return {Vec3{c0.x, c1.x, c2.x},
                    Vec3{c0.y, c1.y, c2.y},
                    Vec3{c0.z, c1.z, c2.z}};
        }

        Mat3 operator+(const Mat3 &o) const
        {
            return {c0 + o.c0, c1 + o.c1, c2 + o.c2};
        }
        Mat3 operator-(const Mat3 &o) const
        {
            return {c0 - o.c0, c1 - o.c1, c2 - o.c2};
        }

        Mat3 operator*(float s) const
        {
            return {c0 * s, c1 * s, c2 * s};
        }

        Mat3 &operator*(float s)
        {
            c0 *= s;
            c1 *= s;
            c2 *= s;
            return *this;
        }

        Mat3 inverse() const
        {

            // 逆行列の計算（Cramer's rule）
            float det = dot(c0, cross(c1, c2));
            if (std::fabs(det) < 1e-8f)
                return Mat3{}; // 非正則行列の場合は単位行列を返す

            float invDet = 1.0f / det;
            Vec3 r0 = cross(c1, c2) * invDet;
            Vec3 r1 = cross(c2, c0) * invDet;
            Vec3 r2 = cross(c0, c1) * invDet;

            return Mat3{r0, r1, r2}.transpose();
        }

        static Mat3 zero() { return Mat3{Vec3{0, 0, 0}, Vec3{0, 0, 0}, Vec3{0, 0, 0}}; }
        static Mat3 identity() { return Mat3{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}; }
        static Mat3 diagonal(float x, float y, float z) { return Mat3{Vec3{x, 0, 0}, Vec3{0, y, 0}, Vec3{0, 0, z}}; }
        static Mat3 skew(const Vec3 &v) { return Mat3{Vec3{0, -v.z, v.y}, Vec3{v.z, 0, -v.x}, Vec3{-v.y, v.x, 0}}; }
    };

    struct Quat
    {
        float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

        Quat() = default;
        Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

        static Quat fromAxisAngle(const Vec3 &axis, float angle)
        {
            Vec3 normAxis = normalize(axis);
            float halfAngle = angle * 0.5f;
            float s = std::sin(halfAngle);
            return Quat{normAxis.x * s, normAxis.y * s, normAxis.z * s, std::cos(halfAngle)};
        }

        Quat operator*(const Quat &rhs) const
        {
            return Quat{
                w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
                w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
                w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
                w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z};
        }

        Quat operator*(float s) const
        {
            return Quat{x * s, y * s, z * s, w * s};
        }

        Quat operator+(const Quat &rhs) const
        {
            return Quat{x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
        }

        void normalizeInPlace()
        {
            float len = std::sqrt(x * x + y * y + z * z + w * w);
            if (len > 1e-8f)
            {
                float invLen = 1.0f / len;
                x *= invLen;
                y *= invLen;
                z *= invLen;
                w *= invLen;
            }
            else
            {
                x = y = z = 0.0f;
                w = 1.0f; // デフォルトの単位クォータニオンに戻す
            }
        }

        Vec3 rotate(const Vec3 &v) const
        {
            Vec3 qv{x, y, z};
            Vec3 t = 2.0f * cross(qv, v);
            return v + w * t + cross(qv, t);
        }

        Mat3 toMat3() const
        {
            float xx = x * x;
            float yy = y * y;
            float zz = z * z;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;
            float wx = w * x;
            float wy = w * y;
            float wz = w * z;

            return Mat3{
                Vec3{1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
                Vec3{2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
                Vec3{2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}};
        }

        void integrate(const Vec3 &omega, float dt)
        {
            Quat wq{omega.x, omega.y, omega.z, 0.0f};
            Quat dq = wq * (*this) * 0.5f;
            *this = *this + dq * dt;
            normalizeInPlace();
        }
    };

    struct AABB
    {
        Vec3 min{0, 0, 0};
        Vec3 max{0, 0, 0};

        bool overlaps(const AABB &other) const
        {
            return (min.x <= other.max.x && max.x >= other.min.x) &&
                   (min.y <= other.max.y && max.y >= other.min.y) &&
                   (min.z <= other.max.z && max.z >= other.min.z);
        }

        void expand(float m)
        {

            min -= Vec3{m, m, m};
            max += Vec3{m, m, m};
        }
    };

    inline float clampf(float v, float lo, float hi)
    {
        return std::max(lo, std::min(v, hi));
    }

}