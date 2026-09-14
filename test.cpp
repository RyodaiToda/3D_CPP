#include "raylib.h"

int main() {
    InitWindow(1280, 720,"New Title");
    SetTargetFPS(60);

    Camera3D cam{};
    cam.position   = Vector3{10, 8, 10};
    cam.target     = Vector3{0, 2, 0};
    cam.up         = Vector3{0, 1, 0};
    cam.fovy       = 50.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    float y = 8.0f, vy = 0.0f;

    while (!WindowShouldClose()) {
        // --- 更新（ここが後で world.step() になる）---
        const float dt = 1.0f / 60.0f;
        vy += -9.81f * dt;          // 速度を先に
        y  += vy * dt;              // その速度で位置を
        if (y < 0.5f) { y = 0.5f; vy = -vy * 0.6f; }   // 床で反発

        // --- 描画 ---
        BeginDrawing();
        ClearBackground(Color{28, 30, 38, 255});

        BeginMode3D(cam);
            DrawSphere(Vector3{0, y, 0}, 0.5f, GREEN);
            DrawGrid(20, 1.0f);
        EndMode3D();

        // DrawFPS(10, 10);
        DrawText(TextFormat("y = %.2f  vy = %.2f", y, vy), 20, 20, 20, RAYWHITE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
