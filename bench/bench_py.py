"""Python 物理エンジン（3D_claude_code/physics）のヘッドレスベンチマーク。

bench_cpp.cpp と同一シーン・同一パラメータ・同一初期値を構築し、実行時間と
最終状態を同じ形式の JSON に吐く。片方を変えたらもう片方も必ず直すこと。

    python bench_py.py --out results_py.json --pile-n 256 --pile-steps 900

Python エンジンの場所は既定で ../../../3D_claude_code。--engine か環境変数
PHYSICS_PY_ROOT で差し替えられる。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

DEFAULT_ENGINE_ROOT = Path(__file__).resolve().parents[3] / "3D_claude_code"


def load_engine(root: Path):
    """Python 物理エンジンを import して必要なシンボルを返す。"""
    if not (root / "physics" / "world.py").exists():
        raise SystemExit(
            f"Python 物理エンジンが見つかりません: {root}\n"
            f"--engine <path> か環境変数 PHYSICS_PY_ROOT で指定してください。"
        )
    sys.path.insert(0, str(root))
    from physics.body import RigidBody
    from physics.math3d import Vector3
    from physics.shapes import Box, Sphere
    from physics.world import PhysicsWorld

    return RigidBody, Vector3, Box, Sphere, PhysicsWorld


# --------------------------------------------------------------------- RNG
class LCG:
    """bench_cpp.cpp の LCG と同一系列。"""

    def __init__(self, seed: int) -> None:
        self.s = seed & 0xFFFFFFFF

    def next(self) -> float:
        self.s = (self.s * 1664525 + 1013904223) & 0xFFFFFFFF
        return (self.s >> 8) / 16777216.0

    def range(self, lo: float, hi: float) -> float:
        return lo + (hi - lo) * self.next()


# ------------------------------------------------------------------- シーン
class Scenes:
    def __init__(self, env) -> None:
        self.RigidBody, self.Vector3, self.Box, self.Sphere, self.PhysicsWorld = env

    def world(self):
        return self.PhysicsWorld(gravity=self.Vector3(0.0, -9.81, 0.0))

    def ground(self, world, rest: float, fric: float) -> None:
        world.add_body(
            self.RigidBody.create_static(
                self.Box(self.Vector3(50.0, 1.0, 50.0)),
                self.Vector3(0.0, -1.0, 0.0),
                restitution=rest,
                friction=fric,
            )
        )

    def freefall(self, world) -> None:
        world.add_body(
            self.RigidBody.create_dynamic(
                self.Sphere(0.5), self.Vector3(0.0, 100.0, 0.0), 1.0,
                restitution=0.0, friction=0.5,
            )
        )

    def bounce(self, world) -> None:
        self.ground(world, 0.8, 0.5)
        world.add_body(
            self.RigidBody.create_dynamic(
                self.Sphere(0.5), self.Vector3(0.0, 5.0, 0.0), 1.0,
                restitution=0.8, friction=0.5,
            )
        )

    def stack(self, world) -> None:
        self.ground(world, 0.0, 0.6)
        for i in range(10):
            world.add_body(
                self.RigidBody.create_dynamic(
                    self.Box(self.Vector3(0.5, 0.5, 0.5)),
                    self.Vector3(0.0, 0.5 + i * 1.01, 0.0),
                    1.0, restitution=0.0, friction=0.6,
                )
            )

    def pile(self, world, n: int) -> None:
        self.ground(world, 0.0, 0.5)
        rng = LCG(12345)
        for i in range(n):
            layer = i // 16
            idx = i % 16
            gx = idx % 4
            gz = idx // 4
            jx = rng.range(-0.03, 0.03)
            jz = rng.range(-0.03, 0.03)
            jy = rng.range(0.0, 0.02)
            x = (gx - 1.5) * 1.15 + jx
            z = (gz - 1.5) * 1.15 + jz
            y = 0.8 + layer * 1.3 + jy
            shape = (
                self.Box(self.Vector3(0.5, 0.5, 0.5)) if i % 2 == 0 else self.Sphere(0.5)
            )
            world.add_body(
                self.RigidBody.create_dynamic(
                    shape, self.Vector3(x, y, z), 1.0, restitution=0.0, friction=0.5
                )
            )


# --------------------------------------------------------------------- 実行
def shapes_of(world) -> list[dict]:
    out = []
    for b in world.bodies:
        s = b.shape
        if hasattr(s, "radius"):
            out.append({"type": "sphere", "r": s.radius, "static": b.is_static})
        else:
            h = s.half_extents
            out.append({"type": "box", "half": [h.x, h.y, h.z], "static": b.is_static})
    return out


def frame_of(world) -> list[list[float]]:
    return [
        [b.position.x, b.position.y, b.position.z,
         b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z]
        for b in world.bodies
    ]


def run(world, dt: float, steps: int, track_apex: bool,
        frames: list | None, stride: int) -> dict:
    contact_points_total = 0
    max_penetration = 0.0
    apexes: list[float] = []
    prev_vy = 0.0
    have_contacted = False

    t0 = time.perf_counter()
    for k in range(steps):
        world.step(dt)

        for manifold in world.last_contacts:
            contact_points_total += len(manifold.points)
            for point in manifold.points:
                if point.depth > max_penetration:
                    max_penetration = point.depth

        if track_apex:
            body = world.bodies[-1]
            vy = body.linear_velocity.y
            if world.last_contacts:
                have_contacted = True
            if have_contacted and prev_vy > 0.0 and vy <= 0.0 and len(apexes) < 6:
                apexes.append(body.position.y)
            prev_vy = vy

        if frames is not None and k % stride == 0:
            frames.append(frame_of(world))
    t1 = time.perf_counter()

    return {
        "ms": (t1 - t0) * 1000.0,
        "contact_points_total": contact_points_total,
        "max_penetration": max_penetration,
        "apexes": apexes,
    }


def dump_bodies(world) -> list[list[float]]:
    out = []
    for b in world.bodies:
        q = b.orientation
        out.append([
            b.position.x, b.position.y, b.position.z,
            q.w, q.x, q.y, q.z,
            b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z,
            b.angular_velocity.x, b.angular_velocity.y, b.angular_velocity.z,
        ])
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results_py.json")
    ap.add_argument("--pile-n", type=int, default=256)
    ap.add_argument("--pile-steps", type=int, default=900)
    ap.add_argument("--scene", default="")
    ap.add_argument("--trace", default="")
    ap.add_argument("--trace-stride", type=int, default=10)
    ap.add_argument(
        "--engine",
        default=os.environ.get("PHYSICS_PY_ROOT", str(DEFAULT_ENGINE_ROOT)),
        help="3D_claude_code のパス",
    )
    args = ap.parse_args()

    scenes_api = Scenes(load_engine(Path(args.engine)))

    scenes = [
        ("freefall", 1.0 / 240.0, 20000),
        ("bounce", 1.0 / 240.0, 4800),
        ("stack", 1.0 / 240.0, 2400),
        ("pile", 1.0 / 120.0, args.pile_steps),
    ]

    results = {
        "engine": "python",
        "config": "native",
        "pile_n": args.pile_n,
        "scenes": {},
    }

    for name, dt, steps in scenes:
        if args.scene and args.scene != name:
            continue

        world = scenes_api.world()
        if name == "pile":
            scenes_api.pile(world, args.pile_n)
        else:
            getattr(scenes_api, name)(world)

        frames: list | None = [] if args.trace else None
        stats = run(world, dt, steps, name == "bounce", frames, args.trace_stride)
        stats["dt"] = dt
        stats["steps"] = steps
        stats["bodies"] = dump_bodies(world)
        results["scenes"][name] = stats

        if args.trace:
            Path(args.trace).write_text(
                json.dumps({
                    "engine": "python",
                    "config": "native",
                    "scene": name,
                    "dt": dt,
                    "stride": args.trace_stride,
                    "shapes": shapes_of(world),
                    "frames": frames,
                }),
                encoding="utf-8",
            )

        print(
            f"[py/native      ] {name:<9} {steps:6d} steps  {stats['ms']:10.2f} ms  "
            f"({stats['ms'] / steps:.4f} ms/step)",
            file=sys.stderr,
            flush=True,
        )

    Path(args.out).write_text(json.dumps(results), encoding="utf-8")


if __name__ == "__main__":
    main()
