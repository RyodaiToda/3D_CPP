// ---------------------------------------------------------------------------
// Physics Range - FPS 視点で当たり判定を検証するためのミニゲーム
//
//   目的: 自作の phys エンジンの接触判定が正しく動いているかを、実際に歩いて
//         ぶつかって撃って確かめられるようにする。
//
//   ビルド: cmake --build build --config Debug --target game
//   実行  : .\build\Debug\game.exe
//   自己診断: .\build\Debug\game.exe --selftest   (描画なしで物理だけ検証)
// ---------------------------------------------------------------------------
#include "raylib.h"
#include "raymath.h"
#include "phys/world.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace phys;

static inline Vector3 toRay(const Vec3 &v) { return Vector3{v.x, v.y, v.z}; }

// 見た目の色
static const Color C_GROUND{58, 62, 72, 255};
static const Color C_WALL{44, 48, 58, 255};
static const Color C_BRICK{176, 94, 72, 255};
static const Color C_STONE{132, 138, 150, 255};
static const Color C_PIN{236, 240, 246, 255};
static const Color C_TARGET{228, 74, 74, 255};
static const Color C_TARGET_DOWN{80, 150, 90, 255};
static const Color C_AMMO{240, 196, 64, 255};
static const Color C_SKY{26, 28, 36, 255};

// ---------------------------------------------------------------------------

struct Game
{
    World world;
    std::vector<Color> colors;

    Model cube{};
    Model ball{};
    bool hasModels = false;

    RigidBody *player = nullptr;

    struct Target
    {
        RigidBody *body = nullptr;
        float startY = 0.0f;
        bool down = false;
    };
    std::vector<Target> targets;

    std::vector<RigidBody *> ammo;
    int ammoNext = 0;
    bool ammoIsBox = false;

    // カメラ（FPS）
    float yaw = 0.0f;   // 0 で -Z 方向を向く
    float pitch = 0.0f;
    bool grounded = false;

    // ゲーム状態
    float clock = 0.0f;
    bool cleared = false;
    int shots = 0;
    bool debugDraw = false;

    static constexpr float EYE_HEIGHT = 0.62f;
    static constexpr int AMMO_POOL = 24;
    static constexpr float PARK_Y = -600.0f; // 未使用の弾を待機させる高さ

    // -- 生成ヘルパ ---------------------------------------------------------

    RigidBody *addBox(const Vec3 &pos, const Vec3 &he, float mass, Color c)
    {
        RigidBody *b = world.createBox(pos, he, mass);
        if ((int)colors.size() <= b->id) colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }

    RigidBody *addSphere(const Vec3 &pos, float r, float mass, Color c)
    {
        RigidBody *b = world.createSphere(pos, r, mass);
        if ((int)colors.size() <= b->id) colors.resize(b->id + 1, WHITE);
        colors[b->id] = c;
        return b;
    }

    void addTarget(RigidBody *b)
    {
        targets.push_back(Target{b, b->position.y, false});
    }

    // -- シーン -------------------------------------------------------------

    void buildScene()
    {
        world.clear();
        colors.clear();
        targets.clear();
        ammo.clear();
        ammoNext = 0;
        clock = 0.0f;
        cleared = false;
        shots = 0;

        // 地面（上面が y = 0）
        RigidBody *g = addBox({0.0f, -1.0f, 0.0f}, {17.0f, 1.0f, 17.0f}, 0.0f, C_GROUND);
        g->friction = 0.8f;
        g->restitution = 0.05f;

        // 外周の壁
        const Vec3 wallHX{0.5f, 3.0f, 16.0f};
        const Vec3 wallHZ{16.0f, 3.0f, 0.5f};
        addBox({-16.0f, 3.0f, 0.0f}, wallHX, 0.0f, C_WALL);
        addBox({16.0f, 3.0f, 0.0f}, wallHX, 0.0f, C_WALL);
        addBox({0.0f, 3.0f, -16.0f}, wallHZ, 0.0f, C_WALL);
        addBox({0.0f, 3.0f, 16.0f}, wallHZ, 0.0f, C_WALL);

        // --- レンガの壁（箱同士の積み重ねと崩壊を見る）---
        const float bhx = 0.5f, bhy = 0.35f, bhz = 0.5f;
        const float rowStep = bhy * 2.0f + 0.01f;
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 6; ++col)
            {
                float x = -2.75f + col * 1.1f + ((row % 2) ? 0.55f : 0.0f);
                float y = bhy + row * rowStep;
                RigidBody *b = addBox({x, y, -4.0f}, {bhx, bhy, bhz}, 1.2f, C_BRICK);
                b->friction = 0.75f;
                b->restitution = 0.05f;
            }
        }
        // レンガの上の的 3 つ
        {
            float topY = bhy + 3 * rowStep + bhy; // 最上段の上面
            for (float x : {-2.2f, 0.3f, 2.8f})
            {
                RigidBody *t = addBox({x, topY + 0.3f, -4.0f}, {0.3f, 0.3f, 0.3f}, 0.8f, C_TARGET);
                t->friction = 0.6f;
                addTarget(t);
            }
        }

        // --- ピラミッド（左）---
        const int levels = 4;
        for (int L = 0; L < levels; ++L)
        {
            int n = levels - L;
            for (int i = 0; i < n; ++i)
            {
                float x = -9.0f + (i - (n - 1) * 0.5f) * 1.05f;
                float y = 0.5f + L * 1.0f;
                RigidBody *b = addBox({x, y, -8.0f}, {0.5f, 0.5f, 0.5f}, 1.0f, C_STONE);
                b->friction = 0.7f;
                b->restitution = 0.05f;
            }
        }
        {
            RigidBody *t = addBox({-9.0f, 4.3f, -8.0f}, {0.3f, 0.3f, 0.3f}, 0.8f, C_TARGET);
            addTarget(t);
        }

        // --- 塔（右）---
        for (int i = 0; i < 6; ++i)
        {
            RigidBody *b = addBox({9.0f, 0.45f + i * 0.9f, -8.0f}, {0.45f, 0.45f, 0.45f}, 1.0f, C_STONE);
            b->friction = 0.7f;
            b->restitution = 0.05f;
        }
        {
            RigidBody *t = addBox({9.0f, 5.7f, -8.0f}, {0.3f, 0.3f, 0.3f}, 0.8f, C_TARGET);
            addTarget(t);
        }

        // --- ピン列（倒れたことを姿勢で判定する的）---
        for (int i = 0; i < 7; ++i)
        {
            float x = -6.0f + i * 2.0f;
            RigidBody *p = addBox({x, 0.9f, 2.0f}, {0.18f, 0.9f, 0.18f}, 0.9f, C_PIN);
            p->friction = 0.9f;
            p->restitution = 0.05f;
            p->angularDamping = 0.05f;
            addTarget(p);
        }

        // --- プレイヤー ---
        player = addBox({0.0f, 0.9f, 11.0f}, {0.3f, 0.85f, 0.3f}, 5.0f, BLANK);
        player->friction = 0.2f;
        player->restitution = 0.0f;
        player->linearDamping = 0.0f;
        // 転倒しないように回転を殺す（慣性テンソルの逆行列をゼロに）
        player->invInertiaLocal = Vec3{0.0f, 0.0f, 0.0f};
        player->updateInertiaWorld();

        // --- 弾のプール（World に削除 API が無いので使い回す）---
        for (int i = 0; i < AMMO_POOL; ++i)
        {
            RigidBody *b = addSphere({0.0f, PARK_Y - i * 2.0f, 0.0f}, 0.22f, 0.0f, C_AMMO);
            ammo.push_back(b);
        }

        yaw = 0.0f;
        pitch = -0.05f;
        grounded = false;
    }

    // -- 視線 ---------------------------------------------------------------

    Vec3 lookDir() const
    {
        return Vec3{std::cos(pitch) * std::sin(yaw),
                    std::sin(pitch),
                    -std::cos(pitch) * std::cos(yaw)};
    }
    Vec3 eye() const
    {
        return player->position + Vec3{0.0f, EYE_HEIGHT, 0.0f};
    }

    // -- 発射 ---------------------------------------------------------------

    void fire()
    {
        RigidBody *b = ammo[ammoNext];
        ammoNext = (ammoNext + 1) % (int)ammo.size();

        Vec3 dir = lookDir();
        b->shape = ammoIsBox ? Shape::box(Vec3{0.22f, 0.22f, 0.22f}) : Shape::sphere(0.22f);
        b->position = eye() + dir * 1.0f;
        b->orientation = Quat{};
        b->angularVelocity = Vec3{0.0f, 0.0f, 0.0f};
        b->setMass(ammoIsBox ? 1.3f : 1.6f); // 形状を決めてから慣性を再計算させる
        b->velocity = dir * 26.0f;
        b->restitution = 0.35f;
        b->friction = 0.5f;
        b->linearDamping = 0.01f;
        ++shots;
    }

    void parkStrayAmmo()
    {
        for (RigidBody *b : ammo)
        {
            if (!b->isStatic() && b->position.y < -30.0f)
            {
                b->setMass(0.0f); // 静的に戻して待機させる
                b->position = Vec3{0.0f, PARK_Y, 0.0f};
                b->velocity = Vec3{0.0f, 0.0f, 0.0f};
                b->angularVelocity = Vec3{0.0f, 0.0f, 0.0f};
            }
        }
    }

    // -- 接地判定（接触マニフォールドの法線から求める）-----------------------
    //    normal は bodyA -> bodyB 向き。プレイヤーが B 側なら上向きが接地。
    void updateGrounded()
    {
        grounded = false;
        for (const Manifold &m : world.getManifolds())
        {
            if (m.bodyA == player && m.normal.y < -0.5f) { grounded = true; return; }
            if (m.bodyB == player && m.normal.y > 0.5f)  { grounded = true; return; }
        }
    }

    // -- 的の判定 -----------------------------------------------------------

    int updateTargets()
    {
        int down = 0;
        for (Target &t : targets)
        {
            if (!t.down)
            {
                // 落下した / 倒れた（ローカル +Y がワールド +Y から 45 度以上傾いた）
                Vec3 up = t.body->orientation.toMat3().col(1);
                bool fell = t.body->position.y < t.startY - 0.7f;
                bool toppled = up.y < 0.70f;
                if (fell || toppled) t.down = true;
            }
            if (t.down) ++down;
        }
        return down;
    }

    // -- 描画 ---------------------------------------------------------------

    void drawBodies()
    {
        for (const auto &b : world.getBodies())
        {
            if (b.get() == player) continue;              // 一人称なので自分は描かない
            if (b->position.y < -50.0f) continue;         // 待機中の弾

            Color c = colors[b->id];
            if (c.a == 0) continue;

            for (const Target &t : targets)
                if (t.body == b.get() && t.down) c = C_TARGET_DOWN;

            Quaternion q{b->orientation.x, b->orientation.y, b->orientation.z, b->orientation.w};
            Vector3 axis; float angle;
            QuaternionToAxisAngle(q, &axis, &angle);
            if (Vector3Length(axis) < 1e-6f) axis = Vector3{0, 1, 0};
            Vector3 pos = toRay(b->position);
            float deg = angle * RAD2DEG;

            if (b->shape.type == ShapeType::Box)
            {
                Vector3 scale = toRay(b->shape.halfExtents * 2.0f);
                DrawModelEx(cube, pos, axis, deg, scale, c);
                DrawModelWiresEx(cube, pos, axis, deg, scale, Fade(BLACK, 0.28f));
            }
            else
            {
                float r = b->shape.radius;
                DrawModelEx(ball, pos, axis, deg, Vector3{r, r, r}, c);
            }
        }
    }

    // 当たり判定の可視化：接触点・法線・AABB
    void drawCollisionDebug()
    {
        for (const auto &b : world.getBodies())
        {
            if (b->position.y < -50.0f) continue;
            AABB box = b->computeAABB();
            DrawBoundingBox(BoundingBox{toRay(box.min), toRay(box.max)},
                            b->isStatic() ? Fade(GRAY, 0.35f) : Fade(GREEN, 0.55f));
        }

        for (const Manifold &m : world.getManifolds())
        {
            for (int i = 0; i < m.count; ++i)
            {
                const Contact &c = m.contacts[i];
                Vector3 p = toRay(c.position);
                DrawSphere(p, 0.055f, RED);
                // 法線（bodyA -> bodyB）
                DrawLine3D(p, toRay(c.position + m.normal * 0.45f), YELLOW);
                // 貫入量を法線の逆向きに（めり込みの深さ）
                if (c.penetration > 0.0f)
                    DrawLine3D(p, toRay(c.position - m.normal * c.penetration), MAGENTA);
            }
        }
    }

    int contactCount() const
    {
        int n = 0;
        for (const Manifold &m : world.getManifolds()) n += m.count;
        return n;
    }
};

// ---------------------------------------------------------------------------
// 自己診断: 描画なしで物理だけ回し、当たり判定が破綻していないか数値で確認する
// ---------------------------------------------------------------------------
static int runSelfTest()
{
    Game g;
    g.buildScene();

    const float dt = 1.0f / 60.0f;
    int failures = 0;

    auto lowestDynamicY = [&]() {
        float lo = 1e9f;
        for (const auto &b : g.world.getBodies())
        {
            if (b->isStatic() || b->position.y < -50.0f) continue;
            lo = std::min(lo, b->position.y);
        }
        return lo;
    };
    auto maxPenetration = [&]() {
        float mx = 0.0f;
        for (const Manifold &m : g.world.getManifolds())
            for (int i = 0; i < m.count; ++i)
                mx = std::max(mx, m.contacts[i].penetration);
        return mx;
    };

    std::printf("== phys self test ==\n");
    std::printf("bodies: %d, targets: %d\n", g.world.bodyCount(), (int)g.targets.size());

    // --- 1. 静止安定性: 何も撃たずに 3 秒回す ---------------------------------
    std::vector<Vec3> before;
    for (const auto &b : g.world.getBodies()) before.push_back(b->position);

    for (int i = 0; i < 180; ++i) g.world.step(dt);

    float maxDrift = 0.0f;
    int idx = 0;
    for (const auto &b : g.world.getBodies())
    {
        if (!b->isStatic() && b->position.y > -50.0f)
            maxDrift = std::max(maxDrift, length(b->position - before[idx]));
        ++idx;
    }
    float maxTilt = 0.0f;
    for (const Game::Target &t : g.targets)
    {
        Vec3 up = t.body->orientation.toMat3().col(1);
        maxTilt = std::max(maxTilt, std::acos(clampf(up.y, -1.0f, 1.0f)) * 180.0f / PHYS_PI);
    }

    std::printf("\n[1] settle 3s (no input)\n");
    std::printf("    max drift of a dynamic body : %.4f m\n", maxDrift);
    std::printf("    max penetration at rest     : %.4f m\n", maxPenetration());
    std::printf("    lowest dynamic body y       : %.4f m\n", lowestDynamicY());
    std::printf("    max target tilt at rest     : %.4f deg (expected ~0)\n", maxTilt);
    std::printf("    targets knocked down        : %d (expected 0)\n", g.updateTargets());
    if (maxTilt > 5.0f)        { std::printf("    FAIL: upright bodies are leaning at rest\n"); ++failures; }
    if (maxDrift > 0.35f)      { std::printf("    FAIL: stacks are not settling\n"); ++failures; }
    if (maxPenetration() > 0.06f) { std::printf("    FAIL: resting penetration too deep\n"); ++failures; }
    if (lowestDynamicY() < -1.0f) { std::printf("    FAIL: something fell through the floor\n"); ++failures; }
    if (g.updateTargets() != 0)   { std::printf("    FAIL: targets fell without being hit\n"); ++failures; }

    // --- 2. 高速な弾が薄い板を貫通しないか（トンネリング）---------------------
    std::printf("\n[2] fast projectile vs pin row (tunnelling check)\n");
    g.player->position = Vec3{-6.0f, 0.9f, 9.0f};
    g.yaw = 0.0f; g.pitch = 0.0f;
    g.fire();
    RigidBody *shot = g.ammo[0];
    RigidBody *pin = g.targets[5].body; // x = -6 のピン（狙っている的）
    bool hitDynamic = false;            // 地面との接触では通らないようにする
    bool hitPin = false;
    for (int i = 0; i < 120; ++i)
    {
        g.world.step(dt);
        for (const Manifold &m : g.world.getManifolds())
        {
            RigidBody *other = (m.bodyA == shot) ? m.bodyB : (m.bodyB == shot) ? m.bodyA : nullptr;
            if (!other) continue;
            if (!other->isStatic()) hitDynamic = true;
            if (other == pin) hitPin = true;
        }
    }
    std::printf("    hit a dynamic body        : %s\n", hitDynamic ? "yes" : "NO");
    std::printf("    hit the aimed pin         : %s\n", hitPin ? "yes" : "no");
    std::printf("    aimed pin tilt afterwards : %.1f deg\n",
                std::acos(clampf(pin->orientation.toMat3().col(1).y, -1.0f, 1.0f)) * 180.0f / PHYS_PI);
    std::printf("    projectile final position : (%.2f, %.2f, %.2f)\n",
                shot->position.x, shot->position.y, shot->position.z);
    if (!hitDynamic) { std::printf("    FAIL: projectile passed through everything (tunnelled)\n"); ++failures; }

    // --- 3. 的を撃ち落とせるか -----------------------------------------------
    std::printf("\n[3] shooting the structures\n");
    struct Aim { float px, pz, yaw, pitch; };
    const Aim aims[] = {
        {  0.0f, 9.0f,  0.00f,  0.10f},   // brick wall
        {  0.0f, 9.0f,  0.00f,  0.16f},
        {  0.0f, 9.0f,  0.02f,  0.18f},
        { -9.0f, 6.0f,  0.00f,  0.22f},   // pyramid
        { -9.0f, 6.0f,  0.00f,  0.26f},
        {  9.0f, 6.0f,  0.00f,  0.30f},   // tower
        {  9.0f, 6.0f,  0.00f,  0.34f},
        { -6.0f, 9.0f,  0.00f, -0.02f},   // pins
        { -2.0f, 9.0f,  0.00f, -0.02f},
        {  2.0f, 9.0f,  0.00f, -0.02f},
        {  6.0f, 9.0f,  0.00f, -0.02f},
    };
    for (const Aim &a : aims)
    {
        g.player->position = Vec3{a.px, 0.9f, a.pz};
        g.player->velocity = Vec3{0, 0, 0};
        g.yaw = a.yaw;
        g.pitch = a.pitch;
        g.fire();
        for (int i = 0; i < 60; ++i) g.world.step(dt);
    }
    for (int i = 0; i < 240; ++i) g.world.step(dt);

    int down = g.updateTargets();
    std::printf("    shots fired          : %d\n", g.shots);
    std::printf("    targets down         : %d / %d\n", down, (int)g.targets.size());
    std::printf("    lowest dynamic y     : %.4f m\n", lowestDynamicY());
    std::printf("    max penetration      : %.4f m\n", maxPenetration());
    if (down == 0)                { std::printf("    FAIL: nothing reacted to the projectiles\n"); ++failures; }
    if (lowestDynamicY() < -1.0f) { std::printf("    FAIL: a body fell through the floor\n"); ++failures; }

    std::printf("\n== %s (%d failure%s) ==\n",
                failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

// ---------------------------------------------------------------------------
// スクリーンショット: ウィンドウを出さずに 1 枚だけ描画して PNG に保存する
//   game.exe --shot out.png [warmupFrames] [debug]
// ---------------------------------------------------------------------------
static int runShot(const char *outPath, int warmupFrames, bool debugOverlay)
{
    SetConfigFlags(FLAG_WINDOW_HIDDEN | FLAG_MSAA_4X_HINT);
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(1280, 720, "shot");
    if (!IsWindowReady()) { std::printf("ERROR: window not ready\n"); return 1; }

    RenderTexture2D rt = LoadRenderTexture(1280, 720);

    Game g;
    g.cube = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    g.ball = LoadModelFromMesh(GenMeshSphere(1.0f, 12, 18));
    g.buildScene();

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < warmupFrames; ++i) g.world.step(dt);
    g.debugDraw = debugOverlay;
    g.updateTargets();

    Camera3D cam{};
    cam.up = Vector3{0, 1, 0};
    cam.fovy = 72.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    Vec3 e = g.eye(), d = g.lookDir();
    cam.position = toRay(e);
    cam.target = toRay(e + d);

    BeginTextureMode(rt);
    ClearBackground(C_SKY);
    BeginMode3D(cam);
    g.drawBodies();
    DrawGrid(34, 1.0f);
    if (g.debugDraw) g.drawCollisionDebug();
    EndMode3D();
    EndTextureMode();

    Image img = LoadImageFromTexture(rt.texture);
    ImageFlipVertical(&img); // RenderTexture は上下反転で読み出される
    ImageFormat(&img, PIXELFORMAT_UNCOMPRESSED_R8G8B8);
    ExportImage(img, outPath);
    UnloadImage(img);

    std::printf("wrote %s (warmup %d frames, debug %s)\n",
                outPath, warmupFrames, debugOverlay ? "on" : "off");

    UnloadRenderTexture(rt);
    UnloadModel(g.cube);
    UnloadModel(g.ball);
    CloseWindow();
    return 0;
}

// ---------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--selftest") == 0)
        return runSelfTest();
    if (argc > 2 && std::strcmp(argv[1], "--shot") == 0)
        return runShot(argv[2],
                       (argc > 3) ? std::atoi(argv[3]) : 60,
                       (argc > 4) && std::strcmp(argv[4], "debug") == 0);

    const int SCREEN_W = 1280, SCREEN_H = 720;
    InitWindow(SCREEN_W, SCREEN_H, "Physics Range - FPS collision test");
    SetTargetFPS(60);

    Game g;
    g.cube = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    g.ball = LoadModelFromMesh(GenMeshSphere(1.0f, 12, 18));
    g.hasModels = true;
    g.buildScene();

    Camera3D camera{};
    camera.up = Vector3{0, 1, 0};
    camera.fovy = 72.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();

    const float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;
    bool paused = false;

    while (!WindowShouldClose())
    {
        // ---------------- 入力 ----------------
        if (IsKeyPressed(KEY_P))
        {
            paused = !paused;
            if (paused) EnableCursor(); else DisableCursor();
        }
        if (IsKeyPressed(KEY_TAB)) g.debugDraw = !g.debugDraw;
        if (IsKeyPressed(KEY_Q)) g.ammoIsBox = !g.ammoIsBox;
        if (IsKeyPressed(KEY_R)) g.buildScene();

        if (!paused)
        {
            Vector2 md = GetMouseDelta();
            g.yaw += md.x * 0.0032f;
            g.pitch -= md.y * 0.0032f;
            g.pitch = Clamp(g.pitch, -1.45f, 1.45f);

            Vec3 dir = g.lookDir();
            Vec3 fwd = normalize(Vec3{dir.x, 0.0f, dir.z});
            Vec3 right{std::cos(g.yaw), 0.0f, std::sin(g.yaw)};

            Vec3 wish{0, 0, 0};
            if (IsKeyDown(KEY_W)) wish += fwd;
            if (IsKeyDown(KEY_S)) wish -= fwd;
            if (IsKeyDown(KEY_D)) wish += right;
            if (IsKeyDown(KEY_A)) wish -= right;
            if (lengthSq(wish) > 1e-6f) wish = normalize(wish);

            float speed = IsKeyDown(KEY_LEFT_SHIFT) ? 9.0f : 5.5f;
            g.player->velocity.x = wish.x * speed;
            g.player->velocity.z = wish.z * speed;

            if (g.grounded && IsKeyPressed(KEY_SPACE)) g.player->velocity.y = 6.4f;

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !g.cleared) g.fire();

            // ---------------- 物理 ----------------
            accumulator += GetFrameTime();
            if (accumulator > 0.25f) accumulator = 0.25f;
            while (accumulator >= FIXED_DT)
            {
                g.world.step(FIXED_DT);
                accumulator -= FIXED_DT;
                if (!g.cleared) g.clock += FIXED_DT;
            }

            g.updateGrounded();
            g.parkStrayAmmo();

            // 落ちたら復帰
            if (g.player->position.y < -20.0f)
            {
                g.player->position = Vec3{0.0f, 1.2f, 11.0f};
                g.player->velocity = Vec3{0, 0, 0};
            }
        }

        int down = g.updateTargets();
        if (down == (int)g.targets.size()) g.cleared = true;

        // ---------------- 描画 ----------------
        Vec3 e = g.eye();
        Vec3 d = g.lookDir();
        camera.position = toRay(e);
        camera.target = toRay(e + d);

        BeginDrawing();
        ClearBackground(C_SKY);

        BeginMode3D(camera);
        g.drawBodies();
        DrawGrid(34, 1.0f);
        if (g.debugDraw) g.drawCollisionDebug();
        EndMode3D();

        // --- HUD ---
        const int cx = GetScreenWidth() / 2, cy = GetScreenHeight() / 2;
        Color cross = g.cleared ? GREEN : RAYWHITE;
        DrawRectangle(cx - 9, cy - 1, 18, 2, cross);
        DrawRectangle(cx - 1, cy - 9, 2, 18, cross);

        DrawRectangle(12, 12, 292, g.debugDraw ? 176 : 114, Fade(BLACK, 0.55f));
        DrawText(TextFormat("Targets  %d / %d", down, (int)g.targets.size()), 24, 24, 22,
                 g.cleared ? GREEN : RAYWHITE);
        DrawText(TextFormat("Time     %.1f s", g.clock), 24, 52, 20, RAYWHITE);
        DrawText(TextFormat("Shots    %d", g.shots), 24, 76, 20, RAYWHITE);
        DrawText(TextFormat("Ammo     %s   [Q]", g.ammoIsBox ? "box" : "sphere"), 24, 98, 20, C_AMMO);

        if (g.debugDraw)
        {
            DrawText(TextFormat("bodies   %d", g.world.bodyCount()), 24, 126, 18, LIME);
            DrawText(TextFormat("manifolds %d", (int)g.world.getManifolds().size()), 24, 146, 18, LIME);
            DrawText(TextFormat("contacts %d", g.contactCount()), 24, 166, 18, LIME);
        }

        DrawText("WASD move   SPACE jump   SHIFT run   LMB shoot   TAB collision debug   R reset   P pause",
                 16, GetScreenHeight() - 28, 18, Fade(RAYWHITE, 0.75f));
        DrawText(g.grounded ? "grounded" : "airborne",
                 GetScreenWidth() - 120, GetScreenHeight() - 28, 18,
                 g.grounded ? Fade(LIME, 0.85f) : Fade(ORANGE, 0.85f));
        DrawFPS(GetScreenWidth() - 96, 16);

        if (g.cleared)
        {
            const char *msg = TextFormat("CLEARED  %.1f s  /  %d shots", g.clock, g.shots);
            int w = MeasureText(msg, 40);
            DrawRectangle(cx - w / 2 - 20, cy - 74, w + 40, 60, Fade(BLACK, 0.65f));
            DrawText(msg, cx - w / 2, cy - 60, 40, GREEN);
            const char *sub = "press R to reset";
            int w2 = MeasureText(sub, 20);
            DrawText(sub, cx - w2 / 2, cy - 8, 20, Fade(RAYWHITE, 0.85f));
        }
        if (paused)
        {
            const char *msg = "PAUSED  -  press P to resume";
            int w = MeasureText(msg, 28);
            DrawText(msg, cx - w / 2, cy + 40, 28, RAYWHITE);
        }

        EndDrawing();
    }

    UnloadModel(g.cube);
    UnloadModel(g.ball);
    CloseWindow();
    return 0;
}
