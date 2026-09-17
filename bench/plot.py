"""ベンチ結果 JSON からグラフ（PNG）を生成する。

    python plot.py --results results --out results

生成物:
    speed.png    シーン別の 1 ステップ所要時間
    scaling.png  体数 vs 1 ステップ所要時間（60FPS 予算線つき）
    stack.png    10段スタックの最終形（設定ごとの小分割）
    bounce.png   反発の頂点高さと理論値
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ---------------------------------------------------------------- 配色・体裁
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"

SERIES = {
    "python": "#2a78d6",        # slot 1 blue
    "cpp-matched": "#eb6834",   # slot 2 orange
    "cpp-warmonly": "#1baf7a",  # slot 3 aqua
    "cpp-default": "#eda100",   # slot 4 yellow
}
STATUS_GOOD = "#0ca30c"
STATUS_CRITICAL = "#d03b3b"

plt.rcParams.update({
    "font.family": ["Yu Gothic", "Meiryo", "MS Gothic", "DejaVu Sans"],
    "font.size": 10,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": AXIS,
    "axes.labelcolor": INK_2,
    "text.color": INK,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "grid.color": GRID,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "savefig.facecolor": SURFACE,
    "savefig.dpi": 160,
})

SCENE_LABEL = {
    "freefall": "自由落下\n(1体・20000步)",
    "bounce": "反発\n(2体・4800步)",
    "stack": "10段スタック\n(11体・2400步)",
    "pile": "山積み\n(257体・900步)",
}


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def us_per_step(run: dict, scene: str) -> float | None:
    d = run["scenes"].get(scene)
    return None if d is None else d["ms"] * 1000.0 / d["steps"]


def headline(ax, title: str, sub: str) -> None:
    """タイトルと補足を重ならない高さに置く。"""
    ax.set_title(title, fontsize=13, fontweight="bold", loc="left", pad=32)
    ax.text(0, 1.015, sub, transform=ax.transAxes, fontsize=9, color=MUTED,
            ha="left", va="bottom")


# ------------------------------------------------------------------- speed
def plot_speed(runs: dict[str, dict], out: Path) -> None:
    scenes = ["freefall", "bounce", "stack", "pile"]
    labels = [SCENE_LABEL[s].replace("步", "ステップ") for s in scenes]
    py = [us_per_step(runs["python"], s) for s in scenes]
    cpp = [us_per_step(runs["cpp-matched"], s) for s in scenes]

    fig, ax = plt.subplots(figsize=(10.0, 4.8))
    y = range(len(scenes))
    h = 0.34
    gap = 0.02  # 隣接バーの間に 2px 相当のサーフェス隙間

    def fmt(v: float) -> str:
        return f"{v:,.0f} μs" if v >= 10 else f"{v:.3g} μs"

    for i, (a, b) in enumerate(zip(py, cpp)):
        ax.barh(i + h / 2 + gap, a, height=h, color=SERIES["python"],
                label="Python" if i == 0 else None, zorder=3)
        ax.barh(i - h / 2 - gap, b, height=h, color=SERIES["cpp-matched"],
                label="C++ (matched)" if i == 0 else None, zorder=3)
        ax.text(a * 1.15, i + h / 2 + gap, fmt(a), va="center", fontsize=9, color=INK_2)
        ax.text(b * 1.15, i - h / 2 - gap, fmt(b), va="center", fontsize=9, color=INK_2)
        ax.text(0.995, i, f"{a / b:.0f}×", transform=ax.get_yaxis_transform(),
                ha="right", va="center", fontsize=14, fontweight="bold", color=INK)

    ax.set_xscale("log")
    ax.set_yticks(list(y))
    ax.set_yticklabels(labels, fontsize=9, color=INK_2)
    ax.set_xlabel("1 ステップあたりの所要時間 [μs] — 対数軸（短いほど速い）")
    ax.set_xlim(0.02, 2e6)
    ax.grid(axis="x", lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, ncol=2,
              loc="upper center", bbox_to_anchor=(0.5, -0.16))
    headline(ax, "同一シーンでの 1 ステップ所要時間",
             "右端の太字 = Python ÷ C++（C++ が何倍速いか）")
    fig.tight_layout()
    fig.savefig(out / "speed.png", bbox_inches="tight")
    plt.close(fig)


# ----------------------------------------------------------------- scaling
def plot_scaling(results_dir: Path, out: Path) -> None:
    def sweep(prefix: str) -> tuple[list[int], list[float]]:
        pts = []
        for f in results_dir.glob(f"sweep_{prefix}_*.json"):
            n = int(re.findall(r"_(\d+)\.json$", f.name)[0])
            d = load(f)["scenes"]["pile"]
            pts.append((n, d["ms"] * 1000.0 / d["steps"]))
        pts.sort()
        return [p[0] for p in pts], [p[1] for p in pts]

    nx_py, ty_py = sweep("py")
    nx_cpp, ty_cpp = sweep("cpp")

    fig, ax = plt.subplots(figsize=(8.2, 5.2))
    ax.axhline(16667, color=MUTED, lw=1.4, ls=(0, (5, 4)), zorder=2)
    ax.text(3400, 16667 * 1.3, "60 FPS の予算 (16.7 ms/step)",
            ha="right", fontsize=9, color=MUTED)

    ax.plot(nx_py, ty_py, color=SERIES["python"], lw=2, marker="o", ms=8,
            mec=SURFACE, mew=2, label="Python", zorder=4)
    ax.plot(nx_cpp, ty_cpp, color=SERIES["cpp-matched"], lw=2, marker="o", ms=8,
            mec=SURFACE, mew=2, label="C++ (matched)", zorder=4)

    if nx_py:
        ax.annotate("Python", (nx_py[-1], ty_py[-1]), textcoords="offset points",
                    xytext=(10, 2), color=SERIES["python"], fontsize=10,
                    fontweight="bold")
    if nx_cpp:
        ax.annotate("C++", (nx_cpp[-1], ty_cpp[-1]), textcoords="offset points",
                    xytext=(10, 2), color=SERIES["cpp-matched"], fontsize=10,
                    fontweight="bold")

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(12, 3600)
    ax.set_ylim(60, 2.5e5)
    ax.set_xlabel("剛体の数")
    ax.set_ylabel("1 ステップあたりの所要時間 [μs]")
    ax.grid(lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=9, loc="lower left", labelcolor=INK_2)
    headline(ax, "体数に対するスケーリング（山積みシーン・各300ステップ）",
             "両者とも O(n²) 総当たりブロードフェーズ。予算線より下が実時間で回せる規模")
    fig.tight_layout()
    fig.savefig(out / "scaling.png", bbox_inches="tight")
    plt.close(fig)


# ------------------------------------------------------------------- stack
def plot_stack(runs: dict[str, dict], out: Path) -> None:
    order = [n for n in ("python", "cpp-matched", "cpp-warmonly", "cpp-default")
             if n in runs and "stack" in runs[n]["scenes"]]
    fig, axes = plt.subplots(1, len(order), figsize=(3.0 * len(order), 5.2),
                             sharey=True)
    if len(order) == 1:
        axes = [axes]

    for ax, name in zip(axes, order):
        d = runs[name]["scenes"]["stack"]
        bodies = d["bodies"][1:]
        held = sum(1 for i, b in enumerate(bodies) if abs(b[1] - (0.5 + i)) < 0.1)
        ok = held == len(bodies)

        ax.axhspan(-0.6, 0.0, color=GRID, zorder=1)
        for i in range(10):  # 理想位置（各段の中心高さ）
            ax.plot([-7, 7], [0.5 + i, 0.5 + i], color=AXIS, lw=0.9,
                    ls=(0, (3, 4)), zorder=2)
        for b in bodies:
            ax.add_patch(Rectangle((b[0] - 0.5, b[1] - 0.5), 1.0, 1.0,
                                   facecolor=SERIES.get(name, MUTED), alpha=0.9,
                                   edgecolor=SURFACE, lw=2, zorder=3))

        status = "維持" if ok else "崩壊"
        mark = "✔" if ok else "✖"
        ax.set_title(name, fontsize=11.5, fontweight="bold", loc="left", pad=22,
                     color=INK)
        ax.text(0, 1.015, f"{mark} {status}   めり込み {d['max_penetration'] * 100:.1f} cm",
                transform=ax.transAxes, fontsize=9.5, va="bottom",
                color=STATUS_GOOD if ok else STATUS_CRITICAL)
        ax.set_xlim(-7, 7)
        ax.set_ylim(-0.6, 11.0)
        ax.set_xlabel("x [m]")
        ax.grid(False)
        ax.tick_params(labelsize=8)

    axes[0].set_ylabel("高さ y [m]   （横の破線 = 理想位置）")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.text(0.004, 0.975, "10段スタック 10 秒後の最終形（真横から）",
             fontsize=13, fontweight="bold", ha="left", va="top")
    fig.text(0.004, 0.925,
             "matched = C++ から Python に無い機能を全部外した設定。"
             "Python と同じように崩れる ＝ 言語ではなくアルゴリズムの差",
             fontsize=9, color=MUTED, ha="left", va="top")
    fig.savefig(out / "stack.png", bbox_inches="tight")
    plt.close(fig)


# ------------------------------------------------------------------ bounce
def plot_bounce(runs: dict[str, dict], out: Path) -> None:
    """頂点高さそのものは 3 本が重なって読めないので、理論値との差を描く。"""
    theo = [0.5 + 4.5 * (0.8 ** (2 * (n + 1))) for n in range(5)]
    x = list(range(1, 6))
    labels = {"python": "Python", "cpp-matched": "C++ (matched)",
              "cpp-default": "C++ (default)"}

    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    ax.axhline(0, color=AXIS, lw=1.4, zorder=2)
    ax.text(5.08, 0, "理論値", fontsize=9, color=MUTED, va="center")

    for name in ("python", "cpp-matched", "cpp-default"):
        d = runs.get(name, {}).get("scenes", {}).get("bounce")
        if not d or not d["apexes"]:
            continue
        ap = d["apexes"][:5]
        err = [(a - t) * 1000.0 for a, t in zip(ap, theo)]
        ax.plot(x[:len(err)], err, color=SERIES[name], lw=2, marker="o", ms=8,
                mec=SURFACE, mew=2, label=labels[name], zorder=4)
        ax.annotate(labels[name], (x[len(err) - 1], err[-1]),
                    textcoords="offset points", xytext=(10, -3),
                    color=SERIES[name], fontsize=9, fontweight="bold")

    ax.set_xticks(x)
    ax.set_xlim(0.7, 6.2)
    ax.set_xlabel("何回目のバウンド")
    ax.set_ylabel("理論値との差 [mm]（＋は跳ねすぎ）")
    ax.grid(axis="y", lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, ncol=3,
              loc="upper center", bbox_to_anchor=(0.5, -0.15))
    headline(ax, "反発係数 e=0.8 の球：頂点高さの理論値からのずれ",
             "跳ね上がり 4.5 m に対する差なので、40 mm でも 1% 弱")
    fig.tight_layout()
    fig.savefig(out / "bounce.png", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="results")
    ap.add_argument("--out", default="results")
    args = ap.parse_args()

    rd = Path(args.results)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    runs = {
        "python": load(rd / "py.json"),
        "cpp-matched": load(rd / "cpp_matched.json"),
        "cpp-warmonly": load(rd / "cpp_warmonly.json"),
        "cpp-default": load(rd / "cpp_default.json"),
    }

    plot_speed(runs, out)
    plot_scaling(rd, out)
    plot_stack(runs, out)
    plot_bounce(runs, out)
    print(f"wrote speed.png scaling.png stack.png bounce.png -> {out}")


if __name__ == "__main__":
    main()
