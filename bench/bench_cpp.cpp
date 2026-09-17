// C++ 物理エンジン（phys/）のヘッドレスベンチマーク。
//
// 兄弟プロジェクト 3D_claude_code の Python 物理エンジンと同一シーン・同一
// パラメータで走らせ、実行時間と最終状態を JSON に吐く。bench_py.py と 1:1 で
// 対応させること（シーン定義・LCG の系列・出力フォーマットを変えたら両方直す）。
//
//   bench --out results.json --config matched --pile-n 256 --pile-steps 900
//
// レンダリングには一切依存しないので raylib は不要。

#include "phys/world.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace phys;

// --------------------------------------------------------------------- RNG
// bench_py.py の LCG と同一の系列を出すこと（初期配置を一致させるため）。
// 戻り値は 2^-24 刻みなので float でも double でも厳密に同じ値になる。
struct LCG {
    uint32_t s;
    explicit LCG(uint32_t seed) : s(seed) {}
    double next() {
        s = s * 1664525u + 1013904223u;
        return (double)(s >> 8) / 16777216.0;
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// ---------------------------------------------------------------- world 設定
//   matched      : Python 版に無い機能をすべて切り、アルゴリズムを揃えた設定
//   warmonly     : matched + ウォームスタートのみ
//   positiononly : matched + 位置補正の強化のみ
//   default      : この C++ エンジンの既定設定
static void applyConfig(World &w, const std::string &cfg) {
    w.velocityIterations = 10;
    w.restitutionThreshold = 1.0f;
    w.penetrationSlop = 0.005f;
    if (cfg == "matched") {
        w.positionIterations = 1;
        w.positionCorrection = 0.2f;
        w.warmStarting = false;
    } else if (cfg == "warmonly") {
        w.positionIterations = 1;
        w.positionCorrection = 0.2f;
        w.warmStarting = true;
    } else if (cfg == "positiononly") {
        w.positionIterations = 3;
        w.positionCorrection = 0.4f;
        w.warmStarting = false;
    } else {
        w.positionIterations = 3;
        w.positionCorrection = 0.4f;
        w.warmStarting = true;
    }
}

static void setMat(RigidBody *b, float rest, float fric, const std::string &cfg) {
    b->restitution = rest;
    b->friction = fric;
    if (cfg != "default") {
        // Python 版に無い機能を無効化
        b->rollingFriction = 0.0f;
        b->linearDamping = 0.0f;
        b->angularDamping = 0.0f;
    } else {
        b->rollingFriction = 0.02f;
        b->linearDamping = 0.01f;
        b->angularDamping = 0.01f;
    }
}

// ------------------------------------------------------------------- 計測
struct Stats {
    double ms = 0.0;
    long long contactPointsTotal = 0;
    double maxPenetration = 0.0;
    std::vector<double> apexes;
};

static double stepPenetration(const World &w) {
    double mx = 0.0;
    for (const auto &m : w.getManifolds())
        for (int i = 0; i < m.count; ++i)
            if (m.contacts[i].penetration > mx) mx = m.contacts[i].penetration;
    return mx;
}

static long long stepContactPoints(const World &w) {
    long long n = 0;
    for (const auto &m : w.getManifolds()) n += m.count;
    return n;
}

// --------------------------------------------------------------------- 出力
static void dumpBodies(FILE *f, const World &w) {
    fprintf(f, "  \"bodies\": [\n");
    const auto &bodies = w.getBodies();
    for (size_t i = 0; i < bodies.size(); ++i) {
        const RigidBody *b = bodies[i].get();
        fprintf(f,
                "    [%.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g, %.9g]%s\n",
                (double)b->position.x, (double)b->position.y, (double)b->position.z,
                (double)b->orientation.w, (double)b->orientation.x, (double)b->orientation.y,
                (double)b->orientation.z,
                (double)b->velocity.x, (double)b->velocity.y, (double)b->velocity.z,
                (double)b->angularVelocity.x, (double)b->angularVelocity.y,
                (double)b->angularVelocity.z,
                (i + 1 < bodies.size()) ? "," : "");
    }
    fprintf(f, "  ]");
}

// 形状一覧（GIF レンダラ用）
static void dumpShapes(FILE *f, const World &w) {
    fprintf(f, "\"shapes\": [");
    const auto &bodies = w.getBodies();
    for (size_t i = 0; i < bodies.size(); ++i) {
        const RigidBody *b = bodies[i].get();
        if (i) fprintf(f, ", ");
        if (b->shape.type == ShapeType::Sphere)
            fprintf(f, "{\"type\": \"sphere\", \"r\": %.6g, \"static\": %s}",
                    (double)b->shape.radius, b->isStatic() ? "true" : "false");
        else
            fprintf(f, "{\"type\": \"box\", \"half\": [%.6g, %.6g, %.6g], \"static\": %s}",
                    (double)b->shape.halfExtents.x, (double)b->shape.halfExtents.y,
                    (double)b->shape.halfExtents.z, b->isStatic() ? "true" : "false");
    }
    fprintf(f, "]");
}

static void dumpFrame(FILE *f, const World &w, bool first) {
    if (!first) fprintf(f, ",\n");
    fprintf(f, "[");
    const auto &bodies = w.getBodies();
    for (size_t i = 0; i < bodies.size(); ++i) {
        const RigidBody *b = bodies[i].get();
        if (i) fprintf(f, ",");
        fprintf(f, "[%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g]",
                (double)b->position.x, (double)b->position.y, (double)b->position.z,
                (double)b->orientation.w, (double)b->orientation.x,
                (double)b->orientation.y, (double)b->orientation.z);
    }
    fprintf(f, "]");
}

// --------------------------------------------------------------------- シーン
static void sceneFreefall(World &w, const std::string &cfg) {
    RigidBody *s = w.createSphere(Vec3{0.0f, 100.0f, 0.0f}, 0.5f, 1.0f);
    setMat(s, 0.0f, 0.5f, cfg);
}

static void sceneBounce(World &w, const std::string &cfg) {
    RigidBody *g = w.createBox(Vec3{0.0f, -1.0f, 0.0f}, Vec3{50.0f, 1.0f, 50.0f}, 0.0f);
    setMat(g, 0.8f, 0.5f, cfg);
    RigidBody *s = w.createSphere(Vec3{0.0f, 5.0f, 0.0f}, 0.5f, 1.0f);
    setMat(s, 0.8f, 0.5f, cfg);
}

static void sceneStack(World &w, const std::string &cfg) {
    RigidBody *g = w.createBox(Vec3{0.0f, -1.0f, 0.0f}, Vec3{50.0f, 1.0f, 50.0f}, 0.0f);
    setMat(g, 0.0f, 0.6f, cfg);
    for (int i = 0; i < 10; ++i) {
        RigidBody *b = w.createBox(Vec3{0.0f, (float)(0.5 + i * 1.01), 0.0f},
                                   Vec3{0.5f, 0.5f, 0.5f}, 1.0f);
        setMat(b, 0.0f, 0.6f, cfg);
    }
}

static void scenePile(World &w, const std::string &cfg, int n) {
    RigidBody *g = w.createBox(Vec3{0.0f, -1.0f, 0.0f}, Vec3{50.0f, 1.0f, 50.0f}, 0.0f);
    setMat(g, 0.0f, 0.5f, cfg);
    LCG rng(12345u);
    for (int i = 0; i < n; ++i) {
        int layer = i / 16;
        int idx = i % 16;
        int gx = idx % 4;
        int gz = idx / 4;
        double jx = rng.range(-0.03, 0.03);
        double jz = rng.range(-0.03, 0.03);
        double jy = rng.range(0.0, 0.02);
        double x = (gx - 1.5) * 1.15 + jx;
        double z = (gz - 1.5) * 1.15 + jz;
        double y = 0.8 + layer * 1.3 + jy;
        RigidBody *b;
        if (i % 2 == 0)
            b = w.createBox(Vec3{(float)x, (float)y, (float)z}, Vec3{0.5f, 0.5f, 0.5f}, 1.0f);
        else
            b = w.createSphere(Vec3{(float)x, (float)y, (float)z}, 0.5f, 1.0f);
        setMat(b, 0.0f, 0.5f, cfg);
    }
}

// --------------------------------------------------------------------- 実行
static Stats run(World &w, double dt, int steps, bool trackApex, FILE *trace, int traceStride) {
    Stats st;
    double prevVy = 0.0;
    bool haveContacted = false;
    bool firstFrame = true;

    auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < steps; ++k) {
        w.step((float)dt);

        st.contactPointsTotal += stepContactPoints(w);
        double p = stepPenetration(w);
        if (p > st.maxPenetration) st.maxPenetration = p;

        if (trackApex) {
            const RigidBody *s = w.getBodies().back().get();
            double vy = s->velocity.y;
            if (!w.getManifolds().empty()) haveContacted = true;
            if (haveContacted && prevVy > 0.0 && vy <= 0.0 && st.apexes.size() < 6)
                st.apexes.push_back((double)s->position.y);
            prevVy = vy;
        }

        if (trace && k % traceStride == 0) {
            dumpFrame(trace, w, firstFrame);
            firstFrame = false;
        }
    }
    auto t1 = std::chrono::steady_clock::now();
    st.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return st;
    // 注: トレース出力はタイマーの内側にある。GIF 用の実行と計測用の実行は
    //     分けること（run_all.ps1 はそうしている）。
}

struct SceneRun {
    std::string name;
    double dt;
    int steps;
};

static const char *argValue(int argc, char **argv, const char *flag, const char *fallback) {
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
    return fallback;
}

int main(int argc, char **argv) {
    std::string out = argValue(argc, argv, "--out", "results_cpp.json");
    std::string cfg = argValue(argc, argv, "--config", "matched");
    int pileN = atoi(argValue(argc, argv, "--pile-n", "256"));
    int pileSteps = atoi(argValue(argc, argv, "--pile-steps", "900"));
    std::string only = argValue(argc, argv, "--scene", "");
    std::string tracePath = argValue(argc, argv, "--trace", "");
    int traceStride = atoi(argValue(argc, argv, "--trace-stride", "10"));

    std::vector<SceneRun> scenes = {
        {"freefall", 1.0 / 240.0, 20000},
        {"bounce", 1.0 / 240.0, 4800},
        {"stack", 1.0 / 240.0, 2400},
        {"pile", 1.0 / 120.0, pileSteps},
    };

    FILE *f = fopen(out.c_str(), "w");
    if (!f) {
        fprintf(stderr, "cannot open %s\n", out.c_str());
        return 1;
    }
    fprintf(f, "{\n\"engine\": \"cpp\",\n\"config\": \"%s\",\n\"pile_n\": %d,\n\"scenes\": {\n",
            cfg.c_str(), pileN);

    bool first = true;
    for (const auto &sc : scenes) {
        if (!only.empty() && only != sc.name) continue;

        World w;
        applyConfig(w, cfg);
        if (sc.name == "freefall") sceneFreefall(w, cfg);
        else if (sc.name == "bounce") sceneBounce(w, cfg);
        else if (sc.name == "stack") sceneStack(w, cfg);
        else scenePile(w, cfg, pileN);

        FILE *tf = nullptr;
        if (!tracePath.empty()) {
            tf = fopen(tracePath.c_str(), "w");
            fprintf(tf, "{\n\"engine\": \"cpp\",\n\"config\": \"%s\",\n\"scene\": \"%s\",\n",
                    cfg.c_str(), sc.name.c_str());
            fprintf(tf, "\"dt\": %.17g,\n\"stride\": %d,\n", sc.dt, traceStride);
            dumpShapes(tf, w);
            fprintf(tf, ",\n\"frames\": [\n");
        }

        Stats st = run(w, sc.dt, sc.steps, sc.name == "bounce", tf, traceStride);

        if (tf) {
            fprintf(tf, "\n]\n}\n");
            fclose(tf);
        }

        if (!first) fprintf(f, ",\n");
        first = false;
        fprintf(f, "  \"%s\": {\n", sc.name.c_str());
        fprintf(f, "  \"dt\": %.17g,\n  \"steps\": %d,\n", sc.dt, sc.steps);
        fprintf(f, "  \"ms\": %.6f,\n", st.ms);
        fprintf(f, "  \"contact_points_total\": %lld,\n", st.contactPointsTotal);
        fprintf(f, "  \"max_penetration\": %.9g,\n", st.maxPenetration);
        fprintf(f, "  \"apexes\": [");
        for (size_t i = 0; i < st.apexes.size(); ++i)
            fprintf(f, "%s%.9g", i ? ", " : "", st.apexes[i]);
        fprintf(f, "],\n");
        dumpBodies(f, w);
        fprintf(f, "\n  }");
        fflush(f);

        fprintf(stderr, "[cpp/%-12s] %-9s %6d steps  %10.2f ms  (%.4f ms/step)\n",
                cfg.c_str(), sc.name.c_str(), sc.steps, st.ms, st.ms / sc.steps);
    }
    fprintf(f, "\n}\n}\n");
    fclose(f);
    return 0;
}
