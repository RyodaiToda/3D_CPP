"""トレースと計測結果を突き合わせて GIF を作る（run_all.ps1 から呼ばれる）。

各パネルのラベルに「実測 ms/step」と「実時間の何倍で回せるか」を載せる。
その数字は計測用の実行（トレースなし）から取る — トレース付きの実行は
ファイル I/O でタイマーが汚れるため。

    python gif_labels.py --results results
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import make_gif

# scene -> (出力名, dt, パネル定義, max_frames, fps, 見出し)
#   パネル定義 = [(表示名, トレースのファイル名, 計測結果のファイル名), ...]
JOBS = {
    "stack": (
        "stack.gif", 1 / 240,
        [("Python", "trace_py_stack.json", "py.json"),
         ("C++ (matched)", "trace_cpp_matched_stack.json", "cpp_matched.json"),
         ("C++ (default)", "trace_cpp_default_stack.json", "cpp_default.json")],
        100, 11,
        "matched = Python に無い機能を全部外した C++",
    ),
    "pile": (
        "pile.gif", 1 / 120,
        [("Python", "trace_py_pile.json", "py.json"),
         ("C++ (matched)", "trace_cpp_matched_pile.json", "cpp_matched.json")],
        80, 14,
        "",
    ),
}


def rate_label(results: Path, engine_file: str, scene: str, dt: float) -> str:
    path = results / engine_file
    if not path.exists():
        return ""
    d = json.loads(path.read_text(encoding="utf-8"))["scenes"].get(scene)
    if d is None:
        return ""
    ms = d["ms"] / d["steps"]
    realtime = (dt * 1000.0) / ms  # 物理 1 秒ぶんを何倍速で計算できるか
    speed = f"{realtime:,.2f}" if realtime < 10 else f"{realtime:,.0f}"
    return f"{ms:.3f} ms/step   実時間の {speed} 倍速"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default="results")
    ap.add_argument("--scene", default="", choices=["", "stack", "pile"])
    args = ap.parse_args()
    rd = Path(args.results)

    for scene, (out, dt, specs, max_frames, fps, subtitle) in JOBS.items():
        if args.scene and args.scene != scene:
            continue
        panels = []
        for label, trace_name, result_name in specs:
            tp = rd / trace_name
            if not tp.exists():
                print(f"skip panel {label}: {tp.name} がありません")
                continue
            panels.append({
                "label": label,
                "trace": json.loads(tp.read_text(encoding="utf-8")),
                "rate": rate_label(rd, result_name, scene, dt),
            })
        if len(panels) < 2:
            print(f"skip {out}: パネルが足りません")
            continue
        make_gif.build(panels, rd / out, scene, max_frames=max_frames, fps=fps,
                       subtitle=subtitle)


if __name__ == "__main__":
    main()
