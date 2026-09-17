"""bench_cpp / bench_py の結果 JSON を突き合わせてテキストレポートを出す。

    python compare.py python=results/py.json cpp-matched=results/cpp_matched.json ...

ラベル=パス を並べた順に列が並ぶ。ラベル python は「一致度」の基準に使われる。
"""

from __future__ import annotations

import io
import json
import math
import sys
from pathlib import Path

G = 9.81

# Windows のコンソールは既定で CP932。UTF-8 で出す。
if hasattr(sys.stdout, "buffer"):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")


def load(path: str) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def analytic_freefall(y0: float, dt: float, n: int) -> tuple[float, float]:
    """半陰的オイラーの厳密解: v_k = -g k dt,  y_k = y0 - g dt^2 k(k+1)/2"""
    return y0 - G * dt * dt * n * (n + 1) / 2.0, -G * n * dt


def section(title: str) -> None:
    print()
    print("=" * 90)
    print(title)
    print("=" * 90)


def scene_order(runs: dict[str, dict]) -> list[str]:
    order: list[str] = []
    for r in runs.values():
        for s in r["scenes"]:
            if s not in order:
                order.append(s)
    return order


def perf_table(runs: dict[str, dict]) -> None:
    section("1. 速度 (wall-clock)")
    names = list(runs)
    print(f"{'scene':<10}{'steps':>7}" + "".join(f"{n:>18}" for n in names))
    print("-" * (17 + 18 * len(names)))
    for sc in scene_order(runs):
        steps = next(
            (runs[n]["scenes"][sc]["steps"] for n in names if sc in runs[n]["scenes"]), 0
        )
        line = f"{sc:<10}{steps:>7}"
        for n in names:
            d = runs[n]["scenes"].get(sc)
            line += f"{'n/a':>18}" if d is None else f"{d['ms']:>13.1f} ms"
        print(line)

    print()
    print("1 ステップあたり [μs]")
    print(f"{'scene':<17}" + "".join(f"{n:>18}" for n in names))
    for sc in scene_order(runs):
        line = f"{sc:<17}"
        for n in names:
            d = runs[n]["scenes"].get(sc)
            line += f"{'n/a':>18}" if d is None else f"{d['ms'] * 1000 / d['steps']:>18.1f}"
        print(line)

    if "python" in runs:
        print()
        print("倍率 (python の所要時間 ÷ 各実装の所要時間 = 何倍速いか)")
        print(f"{'scene':<17}" + "".join(f"{n:>18}" for n in names))
        for sc in scene_order(runs):
            p = runs["python"]["scenes"].get(sc)
            if p is None:
                continue
            line = f"{sc:<17}"
            for n in names:
                d = runs[n]["scenes"].get(sc)
                line += f"{'n/a':>18}" if d is None else f"{p['ms'] / d['ms']:>17.1f}x"
            print(line)


def accuracy_freefall(runs: dict[str, dict]) -> None:
    section("2. 精度: 自由落下 (接触なし / 解析解との誤差)")
    print("半陰的オイラーの厳密解と最終 y を比較。差は丸め誤差の蓄積そのもの。")
    print("※ config=default は線形減衰 (linearDamping=0.01) が効くので、ここでの")
    print("   ずれは誤差ではなく仕様の差。")
    print()
    print(f"{'engine':<18}{'final y':>20}{'analytic y':>20}{'abs err':>14}{'rel err':>12}")
    print("-" * 84)
    for n, r in runs.items():
        d = r["scenes"].get("freefall")
        if d is None:
            continue
        y = d["bodies"][0][1]
        ya, _ = analytic_freefall(100.0, d["dt"], d["steps"])
        err = abs(y - ya)
        print(f"{n:<18}{y:>20.9f}{ya:>20.9f}{err:>14.3e}{err / abs(ya):>12.3e}")


def accuracy_bounce(runs: dict[str, dict]) -> None:
    section("3. 精度: 反発 (e=0.8 の球を y=5.0 から落下)")
    print("接触面は y=0、球半径 0.5 なので静止時の中心高さは 0.5。")
    print("理論頂点: h_n = 0.5 + (5.0 - 0.5) * e^(2n)")
    print()
    theo = [0.5 + 4.5 * (0.8 ** (2 * (n + 1))) for n in range(5)]
    print(f"{'bounce #':<10}" + "".join(f"{n:>16}" for n in runs) + f"{'theory':>16}")
    print("-" * (10 + 16 * (len(runs) + 1)))
    for i in range(5):
        line = f"{i + 1:<10}"
        for r in runs.values():
            ap = (r["scenes"].get("bounce") or {}).get("apexes", [])
            line += f"{ap[i]:>16.5f}" if i < len(ap) else f"{'-':>16}"
        print(line + f"{theo[i]:>16.5f}")
    print()
    print("理論値との相対誤差 (1〜3 バウンド目の平均):")
    for n, r in runs.items():
        ap = (r["scenes"].get("bounce") or {}).get("apexes", [])
        if not ap:
            continue
        errs = [abs(ap[i] - theo[i]) / (theo[i] - 0.5) for i in range(min(3, len(ap)))]
        print(f"  {n:<18}{sum(errs) / len(errs):>10.2%}")


def accuracy_stack(runs: dict[str, dict]) -> None:
    section("4. 精度: 10段ボックススタック (10秒後)")
    print("理想: i 段目の中心 y = 0.5 + i、水平ドリフト 0、残留速度 0。")
    print()
    print(f"{'engine':<18}{'max |Δy|':>12}{'max drift':>12}{'max gap err':>13}"
          f"{'max |v|':>12}{'max |ω|':>12}{'max pen':>12}")
    print("-" * 91)
    for n, r in runs.items():
        d = r["scenes"].get("stack")
        if d is None:
            continue
        bodies = d["bodies"][1:]  # 0 番は地面
        dy = max(abs(b[1] - (0.5 + i)) for i, b in enumerate(bodies))
        drift = max(math.hypot(b[0], b[2]) for b in bodies)
        gaps = [bodies[i + 1][1] - bodies[i][1] - 1.0 for i in range(len(bodies) - 1)]
        vmax = max(math.dist(b[7:10], (0, 0, 0)) for b in bodies)
        wmax = max(math.dist(b[10:13], (0, 0, 0)) for b in bodies)
        print(f"{n:<18}{dy:>12.5f}{drift:>12.5f}{max(abs(g) for g in gaps):>13.5f}"
              f"{vmax:>12.5f}{wmax:>12.5f}{d['max_penetration']:>12.5f}")
    print()
    print("各段の中心 y (理想 = 0.5, 1.5, ... 9.5):")
    for n, r in runs.items():
        d = r["scenes"].get("stack")
        if d is None:
            continue
        print(f"  {n:<18}" + " ".join(f"{b[1]:7.3f}" for b in d["bodies"][1:]))


def divergence(runs: dict[str, dict], ref: str = "python") -> None:
    section("5. 両エンジンの一致度 (同一シーン・同一ステップ後の状態差)")
    if ref not in runs:
        return
    print(f"基準 = {ref}。位置差の最大値と RMS [m]。")
    print()
    print(f"{'scene':<10}{'engine':<18}{'max |Δpos|':>14}{'rms |Δpos|':>14}{'max |Δv|':>14}")
    print("-" * 70)
    for sc in runs[ref]["scenes"]:
        rb = runs[ref]["scenes"][sc]["bodies"]
        for n, r in runs.items():
            if n == ref:
                continue
            d = r["scenes"].get(sc)
            if d is None or len(d["bodies"]) != len(rb):
                continue
            dp = [math.dist(b[0:3], a[0:3]) for a, b in zip(rb, d["bodies"])]
            dv = [math.dist(b[7:10], a[7:10]) for a, b in zip(rb, d["bodies"])]
            rms = math.sqrt(sum(x * x for x in dp) / len(dp))
            print(f"{sc:<10}{n:<18}{max(dp):>14.6g}{rms:>14.6g}{max(dv):>14.6g}")


def contacts(runs: dict[str, dict]) -> None:
    section("6. 検出した接触点の総数 (両者が同じ量の仕事をしているかの確認)")
    print(f"{'scene':<17}" + "".join(f"{n:>18}" for n in runs))
    print("-" * (17 + 18 * len(runs)))
    for sc in scene_order(runs):
        line = f"{sc:<17}"
        for r in runs.values():
            d = r["scenes"].get(sc)
            line += f"{'n/a':>18}" if d is None else f"{d['contact_points_total']:>18,}"
        print(line)


def main() -> None:
    runs: dict[str, dict] = {}
    for arg in sys.argv[1:]:
        if "=" not in arg:
            raise SystemExit(f"引数は ラベル=パス の形で渡してください（{arg!r} は不正）\n{__doc__}")
        label, path = arg.split("=", 1)
        runs[label] = load(path)
    if not runs:
        raise SystemExit(__doc__)
    perf_table(runs)
    accuracy_freefall(runs)
    accuracy_bounce(runs)
    accuracy_stack(runs)
    divergence(runs)
    contacts(runs)


if __name__ == "__main__":
    main()
