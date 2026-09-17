"""何段まで積めるかを調べる補助スクリプト（崩壊の過程を時系列で出す）。

    python stack_depth.py --engine-kind py  --boxes 5
    python stack_depth.py --engine-kind py  --boxes 10 --seconds 6

README の「Python 版でも 5 段までなら安定する」の裏取りに使ったもの。
C++ 側は bench_cpp.cpp の stack シーン（10段固定）と run_all.ps1 で見る。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

DEFAULT_ENGINE_ROOT = Path(__file__).resolve().parents[3] / "3D_claude_code"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--boxes", type=int, default=10)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--engine",
                    default=os.environ.get("PHYSICS_PY_ROOT", str(DEFAULT_ENGINE_ROOT)))
    args = ap.parse_args()

    sys.path.insert(0, args.engine)
    from physics.body import RigidBody
    from physics.math3d import Vector3
    from physics.shapes import Box
    from physics.world import PhysicsWorld

    world = PhysicsWorld(gravity=Vector3(0.0, -9.81, 0.0))
    world.add_body(
        RigidBody.create_static(
            Box(Vector3(50.0, 1.0, 50.0)), Vector3(0.0, -1.0, 0.0),
            restitution=0.0, friction=0.6,
        )
    )
    for i in range(args.boxes):
        world.add_body(
            RigidBody.create_dynamic(
                Box(Vector3(0.5, 0.5, 0.5)), Vector3(0.0, 0.5 + i * 1.01, 0.0),
                1.0, restitution=0.0, friction=0.6,
            )
        )

    dt = 1.0 / 240.0
    steps = int(args.seconds / dt)
    print(f"{args.boxes} 段 / {args.seconds} 秒 @ {1/dt:.0f} Hz "
          f"(理想の最上段 y = {args.boxes - 0.5})")
    print(f"{'t[s]':>6} {'最上段 y':>9} {'最大|x,z|':>10} {'最大めり込み':>12}  各段の y")
    for k in range(steps):
        world.step(dt)
        if k % 120 == 0 or k == steps - 1:
            bodies = world.bodies[1:]
            ys = [b.position.y for b in bodies]
            drift = max(max(abs(b.position.x), abs(b.position.z)) for b in bodies)
            pen = max((p.depth for m in world.last_contacts for p in m.points),
                      default=0.0)
            print(f"{k*dt:6.2f} {max(ys):9.4f} {drift:10.4f} {pen:12.5f}  "
                  + " ".join(f"{y:6.3f}" for y in ys))


if __name__ == "__main__":
    main()
