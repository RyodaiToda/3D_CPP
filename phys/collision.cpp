#include "phys/collision.h"

#include <cfloat>
#include <cmath>

namespace phys
{

    static inline Vec3 toLocal(const RigidBody *b, const Vec3 &world)
    {
        Mat3 Rt = b->orientation.toMat3().transpose();
        return Rt * (world - b->position);
    }

    static void finalizeContacts(Manifold &m)
    {
        m.restitution = std::max(m.bodyA->restitution, m.bodyB->restitution);
        m.friction = std::sqrt(m.bodyA->friction * m.bodyB->friction);
        m.rollingFriction = std::sqrt(m.bodyA->rollingFriction * m.bodyB->rollingFriction);
        buildTangents(m.normal, m.tangent[0], m.tangent[1]);
        for (int i = 0; i < m.count; i++)
        {
            m.contacts[i].localA = toLocal(m.bodyA, m.contacts[i].position);
            m.contacts[i].localB = toLocal(m.bodyB, m.contacts[i].position);
        }
    }

    static bool collideSphereSphere(RigidBody *a, RigidBody *b, Manifold &m)
    {
        Vec3 d = b->position - a->position;
        float rr = a->shape.radius + b->shape.radius;
        float d2 = lengthSq(d);
        if (d2 > rr * rr)
            return false;

        float dist = std::sqrt(d2);
        Vec3 n = (dist > 1e-6f) ? d / dist : Vec3(0, 1, 0);

        m.bodyA = a;
        m.bodyB = b;
        m.normal = n;
        m.count = 1;
        m.contacts[0] = Contact{};
        m.contacts[0].penetration = rr - dist;
        m.contacts[0].position = a->position + n * (a->shape.radius - (rr - dist) * 0.5f);
        return true;
    }

    static bool collideSphereBox(RigidBody *sph, RigidBody *box, Manifold &m, bool sphereIsA)
    {
        Mat3 R = box->orientation.toMat3();
        Mat3 Rt = R.transpose();
        Vec3 he = box->shape.halfExtents;
        float r = sph->shape.radius;

        Vec3 local = Rt * (sph->position - box->position);

        Vec3 clamped{clampf(local.x, -he.x, he.x), clampf(local.y, -he.y, he.y), clampf(local.z, -he.z, he.z)};

        bool centerInside = (clamped.x == local.x && clamped.y == local.y && clamped.z == local.z);

        Vec3 nBoxToSphere;
        float penetration;
        Vec3 contactPoint;

        if (!centerInside)
        {
            Vec3 closestWorld = box->position + R * clamped;
            Vec3 delta = sph->position - closestWorld;
            float d2 = lengthSq(delta);
            if (d2 > r * r)
                return false;

            float dist = std::sqrt(d2);
            nBoxToSphere = (dist > 1e-6f) ? delta / dist : Vec3{0, 1, 0};
            penetration = r - dist;
            contactPoint = closestWorld;
        }
        else
        {
            int bestAxis = 0;
            float bestDepth = FLT_MAX;
            for (int k = 0; k < 3; k++)
            {
                float depth = he[k] - std::fabs(local[k]);
                if (depth < bestDepth)
                {
                    bestDepth = depth;
                    bestAxis = k;
                }
                float sign = (local[bestAxis] >= 0.0f) ? 1.0f : -1.0f;
                nBoxToSphere = R.col(bestAxis) * sign;

                Vec3 surf = local;
                surf[bestAxis] = sign * he[bestAxis];
                contactPoint = box->position + R * surf;
                penetration = r + bestDepth;
            }
        }
        if (sphereIsA)
        {
            m.bodyA = sph;
            m.bodyB = box;
            m.normal = -1.0f * nBoxToSphere;
        }
        else
        {
            m.bodyA = box;
            m.bodyB = sph;
            m.normal = nBoxToSphere;
        }

        m.count = 1;
        m.contacts[0] = Contact{};
        m.contacts[0].penetration = penetration;
        m.contacts[0].position = contactPoint;

        return true;
    }

    static bool collideBoxBox(RigidBody *A, RigidBody *B, Manifold &m)
    {
        Mat3 RA = A->orientation.toMat3();
        Mat3 RB = B->orientation.toMat3();
        Vec3 ea = A->shape.halfExtents;
        Vec3 eb = B->shape.halfExtents;

        Vec3 dWorld = B->position - A->position;

        float R[3][3], AbsR[3][3];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R[i][j] = dot(RA.col(i), RB.col(j));
                AbsR[i][j] = std::fabs(R[i][j]) + 1e-6f;
            }
        }

        Vec3 t{dot(RA.col(0), dWorld), dot(RA.col(1), dWorld), dot(RA.col(2), dWorld)};

        // SAT
        int bestType = -1, bestI = 0, bestJ = 0;
        float bestOverlap = FLT_MAX;
        float bestScore = FLT_MAX;

        // A
        for (int i = 0; i < 3; i++)
        {
            float ra = ea[i];
            float rb = eb[0] * AbsR[i][0] + eb[1] * AbsR[i][1] + eb[2] * AbsR[i][2];
            float overlap = ra + rb - std::fabs(t[i]);
            if (overlap < 0.0f)
                return false;
            if (overlap < bestScore)
            {
                bestScore = overlap;
                bestOverlap = overlap;
                bestType = 0;
                bestI = i;
            }
        }

        // B
        for (int j = 0; j < 3; j++)
        {
            float ra = ea[0] * AbsR[0][j] + ea[1] * AbsR[1][j] + ea[2] * AbsR[2][j];
            float rb = eb[j];
            float proj = std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
            float overlap = ra + rb - proj;
            if (overlap < 0.0f)
                return false;
            float score = overlap * 1.005f + 1e-4f;
            if (score < bestScore)
            {
                bestScore = score;
                bestOverlap = overlap;
                bestType = 1;
                bestJ = j;
            }
        }

        // edge
        for (int i = 0; i < 3; i++)
        {
            int i1 = (i + 1) % 3;
            int i2 = (i + 2) % 3;
            for (int j = 0; j < 3; j++)
            {
                int j1 = (j + 1) % 3;
                int j2 = (j + 2) % 3;

                float axisLen2 = 1.0f - R[i][j] * R[i][j];
                if (axisLen2 < 1e-6)
                    continue;

                float ra = ea[i1] * AbsR[i2][j] + ea[i2] * AbsR[i1][j];
                float rb = eb[j1] * AbsR[i][j2] + eb[j2] * AbsR[i][j1];
                float proj = std::fabs(t[i2] * R[i1][j] - t[i1] * R[i2][j]);
                float overlap = ra + rb - proj;
                if (overlap < 0.0f)
                    return false;

                float invLen = 1.0f / std::sqrt(axisLen2);
                float realOverlap = overlap * invLen;
                float score = realOverlap * 1.05f + 1e-3f;
                if (score < bestScore)
                {
                    bestScore = score;
                    bestOverlap = realOverlap;
                    bestType = 2;
                    bestI = i;
                    bestJ = j;
                }
            }
        }

        if (bestType == -1)
            return false;

        Vec3 normal;
        if (bestType == 0)
        {
            normal = RA.col(bestI) * ((t[bestI] >= 0.0f) ? 1.0f : -1.0f);
        }
        else if (bestType == 1)
        {
            float proj = dot(dWorld, RB.col(bestJ));
            normal = RB.col(bestJ) * ((proj >= 0.0f) ? 1.0f : -1.0f);
        }
        else
        {
            normal = normalize(cross(RA.col(bestI), RB.col(bestJ)));
            if (dot(normal, dWorld) < 0.0f)
                normal = -1.0f * normal;
        }

        m.bodyA = A;
        m.bodyB = B;
        m.normal = normal;
        m.count = 0;

        // edge x edge
        if (bestType == 2)
        {
            Vec3 eAxis = RA.col(bestI);
            Vec3 fAxis = RB.col(bestJ);

            Vec3 pA = A->position;
            for (int k = 0; k < 3; k++)
            {
                if (k != bestI)
                {
                    pA += RA.col(k) * ((dot(RA.col(k), normal) > 0.0f) ? ea[k] : -ea[k]);
                }
            }
            Vec3 pB = B->position;
            for (int k = 0; k < 3; k++)
            {
                if (k != bestJ)
                {
                    pB += RB.col(k) * ((dot(RB.col(k), normal) < 0.0f) ? eb[k] : -eb[k]);
                }
            }

            Vec3 r = pA - pB;
            float b = dot(eAxis, fAxis);
            float c = dot(eAxis, r);
            float f = dot(fAxis, r);
            float denom = 1.0f - b * b;

            Vec3 point;
            if (std::fabs(denom) < 1e-6f)
            {
                point = (pA + pB) * 0.5f;
            }
            else
            {
                float s = (b * f - c) / denom;
                float u = (f - b * c) / denom;
                point = ((pA + eAxis * s) + (pB + fAxis * u)) * 0.5f;
            }

            m.count = 1;
            m.contacts[0] = Contact{};
            m.contacts[0].position = point;
            m.contacts[0].penetration = bestOverlap;
            finalizeContacts(m);
            return true;
        }

        RigidBody *ref;
        RigidBody *inc;
        Vec3 refNormal;
        int refAxis;

        if (bestType == 0)
        {
            ref = A;
            inc = B;
            refNormal = normal;
            refAxis = bestI;
        }
        else
        {
            ref = B;
            inc = A;
            refNormal = -1.0f * normal;
            refAxis = bestJ;
        }

        Mat3 Rref = ref->orientation.toMat3();
        Mat3 Rinc = inc->orientation.toMat3();
        Vec3 heRef = ref->shape.halfExtents;

        float refSign = (dot(Rref.col(refAxis), refNormal) >= 0.0f) ? 1.0f : -1.0f;

        // 入射面
        int incAxis = 0;
        float incSign = 1.0f;
        float minDot = FLT_MAX;
        for (int k = 0; k < 3; k++)
        {
            for (float s : {1.0f, -1.0f})
            {
                float d = dot(Rinc.col(k) * s, refNormal);
                if (d < minDot)
                {
                    minDot = d;
                    incAxis = k;
                    incSign = s;
                }
            }
        }

        Vec3 poly[16], buf[16];
        boxFaceVertices(inc, incAxis, incSign, poly);
        int polyCount = 4;

        int u = (refAxis + 1) % 3;
        int v = (refAxis + 2) % 3;
        for (int axis : {u, v})
        {
            Vec3 an = Rref.col(axis);
            float ac = dot(an, ref->position);

            polyCount = clipPolygonByPlane(poly, polyCount, an, ac + heRef[axis], buf);
            if (polyCount == 0)
                return false;
            for (int i = 0; i < polyCount; i++)
                poly[i] = buf[i];

            polyCount = clipPolygonByPlane(poly, polyCount, -1.0f * an, -ac + heRef[axis], buf);
            if (polyCount == 0)
                return false;
            for (int i = 0; i < polyCount; i++)
                poly[i] = buf[i];
        }

        Vec3 refFaceCenter = ref->position + Rref.col(refAxis) * (refSign * heRef[refAxis]);
        float refPlaneD = dot(refNormal, refFaceCenter);
        Vec3 keptPts[16];
        float keptDepth[16];
        int keptCount = 0;
        for (int i = 0; i < polyCount; ++i)
        {
            float sep = dot(refNormal, poly[i]) - refPlaneD;
            if (sep <= 0.0f)
            {
                // 2 面の中間に接触点を置く（描画上も自然になる）
                keptPts[keptCount] = poly[i] - refNormal * (sep * 0.5f);
                keptDepth[keptCount] = -sep;
                ++keptCount;
            }
        }
        if (keptCount == 0)
            return false;

        reduceContacts(keptPts, keptDepth, keptCount);

        m.count = keptCount;
        for (int i = 0; i < keptCount; ++i)
        {
            m.contacts[i] = Contact{};
            m.contacts[i].position = keptPts[i];
            m.contacts[i].penetration = keptDepth[i];
        }
        finalizeContacts(m);
        return true;
    }

    static void boxFaceVertices(const RigidBody *b, int axis, float sign, Vec3 out[4])
    {
        Mat3 R = b->orientation.toMat3();
        Vec3 he = b->shape.halfExtents;

        int u = (axis + 1) % 3;
        int v = (axis + 2) % 3;

        Vec3 center = b->position + R.col(axis) * (sign * he[axis]);
        Vec3 du = R.col(u) * he[u];
        Vec3 dv = R.col(v) * he[v];

        out[0] = center - du - dv;
        out[1] = center + du - dv;
        out[2] = center + du + dv;
        out[3] = center - du + dv;
    }

    // 平面 dot(n, p) <= d の内側で多角形をクリップ（Sutherland–Hodgman）
    static int clipPolygonByPlane(const Vec3 *in, int n, const Vec3 &planeN, float planeD, Vec3 *out)
    {
        int outCount = 0;
        for (int i = 0; i < n; i++)
        {
            const Vec3 &cur = in[i];
            const Vec3 &next = in[(i + 1) % n];

            float dCur = dot(planeN, cur) - planeD;
            float dnext = dot(planeN, next) - planeD;

            if (dCur <= 0.0f)
                out[outCount++] = cur;

            if ((dCur < 0.0f && dnext > 0.0f) || (dCur > 0.0f && dnext < 0.0f))
            {
                float t = dCur / (dCur - dnext);
                out[outCount++] = cur + (next - cur) * t;
            }
        }
        return outCount;
    }

    static void reduceContacts(Vec3 *pts, float *depths, int &count)
    {
        if (count <= MAX_CONTACT_POINTS)
        {
            return;
        }

        int chosen[MAX_CONTACT_POINTS];
        bool used[32] = {false};

        int best = 0;
        for (int i = 1; i < count; i++)
        {
            if (depths[i] > depths[best])
                best = i;
        }
        chosen[0] = best;
        used[best] = true;

        for (int k = 1; k < MAX_CONTACT_POINTS; ++k)
        {
            int pick = -1;
            float pickDist = -1.0f;
            for (int i = 0; i < count; i++)
            {
                if (used[i])
                    continue;
                float minD = FLT_MAX;
                for (int j = 0; j < k; j++)
                {
                    minD = std::min(minD, lengthSq(pts[i] - pts[chosen[j]]));
                }
                if (minD > pickDist)
                {
                    pickDist = minD;
                    pick = i;
                }
            }
            chosen[k] = pick;
            used[pick] = true;
        }

        Vec3 tempP[MAX_CONTACT_POINTS];
        float tempD[MAX_CONTACT_POINTS];

        for (int k = 0; k < MAX_CONTACT_POINTS; k++)
        {
            tempP[k] = pts[chosen[k]];
            tempD[k] = depths[chosen[k]];
        }
        for (int k = 0; k < MAX_CONTACT_POINTS; k++)
        {
            pts[k] = tempP[k];
            depths[k] = tempD[k];
        }
        count = MAX_CONTACT_POINTS;
    }
    // static inline Vec3 toLocal

    bool collide(RigidBody *a, RigidBody *b, Manifold &m)
    {
        ShapeType ta = a->shape.type;
        ShapeType tb = b->shape.type;

        if (ta == ShapeType::Sphere && tb == ShapeType::Sphere)
        {
            if (!collideSphereSphere(a, b, m))
                return false;
            finalizeContacts(m);
            return true;
        }
        if (ta == ShapeType::Sphere && tb == ShapeType::Box)
        {
            if (!collideSphereBox(a, b, m, /*sphereIsA=*/true))
                return false;
            finalizeContacts(m);
            return true;
        }
        if (ta == ShapeType::Box && tb == ShapeType::Sphere)
        {
            if (!collideSphereBox(b, a, m, /*sphereIsA=*/false))
                return false;
            finalizeContacts(m);
            return true;
        }
        return collideBoxBox(a, b, m); // 内部で finalizeContacts 済み
    }
}