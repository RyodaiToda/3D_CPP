#include "raylib.h"

#include "raymath.h"
#include "rlgl.h"
#include "phys/world.h"
#include <cmath>

#include <iostream>
#include <cstdio>
#include <vector>
using namespace phys;
using namespace std;

// ---------------------------------------------------------------------------
// 変換ヘルパ
// ---------------------------------------------------------------------------
static inline Vector3 toRay(const Vec3 &v) { return Vector3{v.x, v.y, v.z}; }
static inline Vec3 toPhys(const Vector3 &v) { return Vec3{v.x, v.y, v.z}; }

struct Demo
{
    World world;
    std::vector<Color> colors;

    Model cubeModel{};
    Model sphereModel{};

    float camTargetY = 3.0f; // シーンごとの推奨カメラ設定
    float camDistance = 24.0f;

    RigidBody *addBox(const Vec3 &pos, const Vec3 &he, float mass, Color c)
    {
        RigidBody *b = world.createBox(pos, he, mass);
        if (static_cast<int>(colors.size()) <= b->id)
            colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }
    RigidBody *addSphere(const Vec3 &pos, float r, float mass, Color c)
    {
        RigidBody *b = world.createSphere(pos, r, mass);
        if (static_cast<int>(colors.size()) <= b->id)
            colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }
    void addGround()
    {
        RigidBody *g = addBox({0, -1.0f, 0}, {15.0f, 1.0f, 15.0f}, 0.0f, Color{70, 75, 85, 255});
        g->friction = 0.7f;
        g->restitution = 0.1f;
    }

    void buildScene()
    {
        world.clear();
        colors.clear();

        // 地面
        addGround();

        // 左右の壁で横に流れすぎないようにする
        auto *wallL = addBox({-8.5f, 2.5f, 0.0f}, {0.5f, 2.5f, 8.0f}, 0.0f, Color{50, 55, 65, 255});
        auto *wallR = addBox({8.5f, 2.5f, 0.0f}, {0.5f, 2.5f, 8.0f}, 0.0f, Color{50, 55, 65, 255});
        wallL->restitution = 0.05f;
        wallR->restitution = 0.05f;

        // ドミノ列
        const int dominoCount = 12;
        const float spacing = 1.2f;
        const float startX = -5.2f;
        const float y = 1.15f;

        for (int i = 0; i < dominoCount; ++i)
        {
            float x = startX + i * spacing;
            float z = (i % 2 == 0) ? 0.0f : 0.1f;
            auto *d = addBox({x, y, z}, {0.18f, 0.9f, 0.32f}, 1.0f, (i % 2 == 0) ? SKYBLUE : ORANGE);
            d->friction = 0.9f;
            d->restitution = 0.08f;
            d->angularDamping = 0.06f;
        }

        // 最初に押す丸いボール
        auto *ball = addSphere({-7.5f, 2.4f, 0.0f}, 0.55f, 2.0f, GOLD);
        ball->velocity = {7.8f, 0.0f, 0.0f};
        ball->friction = 0.15f;
    }

    void draw()
    {
        for (const auto &body : world.getBodies())
        {
            Quaternion q{body->orientation.x, body->orientation.y, body->orientation.z, body->orientation.w};
            Vector3 axis;
            float angle;
            QuaternionToAxisAngle(q, &axis, &angle);
            if (Vector3Length(axis) < 1e-6f)
                axis = Vector3{0, 1, 0};
            Vector3 pos = toRay(body->position);

            if (body->shape.type == ShapeType::Box)
            {
                Vector3 scale = toRay(body->shape.halfExtents * 2.0f);
                DrawModelEx(cubeModel, pos, axis, angle * RAD2DEG, scale, colors[body->id]);
            }
            else if (body->shape.type == ShapeType::Sphere)
            {
                float r = body->shape.radius;
                Vector3 scale{r, r, r};
                DrawModelEx(sphereModel, pos, axis, angle * RAD2DEG, scale, colors[body->id]);
            }
        }
    }
};

int main()
{
    InitWindow(1280, 720, "New Title");
    SetTargetFPS(60);

    const float FIXED_DT = 1.0f / 60.0f;

    Demo demo;
    demo.cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    demo.sphereModel = LoadModelFromMesh(GenMeshSphere(1.0f, 14, 20));
    demo.buildScene();
    // cout << demo.world.bodyCount() << endl;

    // --- カメラ（球面座標で手動制御） ---
    float camYaw = 0.9f, camPitch = 0.28f;
    float camDist = 24.0f;
    Vector3 camTarget{0, 3.0, 0};

    Camera3D camera{};
    camera.up = Vector3{0, 1, 0};
    camera.fovy = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose())
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 d = GetMouseDelta();
            camYaw -= d.x * 0.005f;
            camPitch += d.y * 0.005f;
            camPitch = Clamp(camPitch, -1.45f, 1.45f);
        }
        camDist = Clamp(camDist - GetMouseWheelMove() * 1.8f, 4.0f, 90.0f);

        if (IsKeyDown(KEY_A))
        {
            camTarget.x += 0.1;
        }
        if (IsKeyDown(KEY_D))
        {
            camTarget.x -= 0.1;
        }

        camera.target = camTarget;

        camera.position = Vector3{
            camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
            camTarget.y + camDist * sinf(camPitch),
            camTarget.z + camDist * cosf(camPitch) * cosf(camYaw)};

        if (IsKeyPressed(KEY_R))
        {
            demo.buildScene();
            camTarget.y = demo.camTargetY;
            camDist = demo.camDistance;
        }

        //    --- 更新（ここが後で world.step() になる）---
        demo.world.step(FIXED_DT);

        // --- 描画 ---
        BeginDrawing();
        ClearBackground(Color{28, 30, 38, 255});

        BeginMode3D(camera);
        demo.draw();
        DrawGrid(20, 1.0f);
        EndMode3D();

        // DrawFPS(10, 10);
        // DrawText(TextFormat("box y = %.2f", demo.world.getBodies()[1]->position.y), 20, 20, 20, RAYWHITE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
