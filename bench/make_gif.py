"""トレース JSON を横に並べて GIF に焼く。

    python make_gif.py --preset stack --out results/stack.gif \
        --panel "Python=results/trace_py_stack.json" \
        --panel "C++ (matched)=results/trace_cpp_matched_stack.json"

--panel は左から順に並ぶ。計測値のラベル付きで作るなら gif_labels.py を使う。
依存は Pillow だけ。ソフトウェアラスタライザ（ペインタのアルゴリズム）なので
OpenGL も raylib も要らない。
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SURFACE = (252, 252, 251)
PANEL_BG = (247, 247, 244)
INK = (11, 11, 11)
MUTED = (137, 135, 129)
GRID_A = (231, 230, 223)
GRID_B = (222, 221, 213)

SLOT = [
    (42, 120, 214),   # slot 1 blue
    (235, 104, 52),   # slot 2 orange
    (27, 175, 122),   # slot 3 aqua
]
LIGHT_DIR = (-0.45, 0.82, 0.35)

PRESETS = {
    # scene: (eye, target, ground half-size, tile step, fovy)
    "stack": ((10.5, 9.0, 17.0), (0.0, 4.2, 0.0), 12.0, 1.5, 44.0),
    "pile": ((14.0, 11.5, 20.0), (0.0, 4.0, 0.0), 13.5, 1.5, 42.0),
}


# ------------------------------------------------------------------ 3D math
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(dot(a, a)) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


def qrot(q, v):
    """q = (w, x, y, z) でベクトルを回す。"""
    w, x, y, z = q
    u = (x, y, z)
    t = mul(cross(u, v), 2.0)
    return add(add(v, mul(t, w)), cross(u, t))


class Camera:
    def __init__(self, eye, target, w, h, fovy_deg):
        self.eye = eye
        f = norm(sub(target, eye))
        r = norm(cross(f, (0.0, 1.0, 0.0)))
        u = cross(r, f)
        self.f, self.r, self.u = f, r, u
        self.w, self.h = w, h
        self.scale = (h / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)

    def project(self, p):
        d = sub(p, self.eye)
        z = dot(d, self.f)
        if z <= 0.1:
            return None
        return (self.w / 2.0 + self.scale * dot(d, self.r) / z,
                self.h / 2.0 - self.scale * dot(d, self.u) / z,
                z)


def shade(base, normal, ambient=0.42):
    lam = max(0.0, dot(normal, LIGHT_DIR))
    k = ambient + (1.0 - ambient) * lam
    return tuple(min(255, int(c * k + 255 * 0.06 * lam)) for c in base)


# ------------------------------------------------------------------ 形状
BOX_FACES = (
    ((1, 0, 0), ((1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1))),
    ((-1, 0, 0), ((-1, -1, 1), (-1, 1, 1), (-1, 1, -1), (-1, -1, -1))),
    ((0, 1, 0), ((-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1))),
    ((0, -1, 0), ((-1, -1, 1), (-1, -1, -1), (1, -1, -1), (1, -1, 1))),
    ((0, 0, 1), ((-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))),
    ((0, 0, -1), ((1, -1, -1), (-1, -1, -1), (-1, 1, -1), (1, 1, -1))),
)


def collect_box(prims, cam, pos, quat, half, color):
    for n_local, corners in BOX_FACES:
        n_world = qrot(quat, n_local)
        pts3 = [add(pos, qrot(quat, (c[0] * half[0], c[1] * half[1], c[2] * half[2])))
                for c in corners]
        centroid = mul((sum(p[0] for p in pts3), sum(p[1] for p in pts3),
                        sum(p[2] for p in pts3)), 0.25)
        if dot(n_world, sub(centroid, cam.eye)) >= 0.0:
            continue  # 裏面
        pts2 = [cam.project(p) for p in pts3]
        if any(p is None for p in pts2):
            continue
        depth = math.dist(centroid, cam.eye)
        prims.append((depth, "poly", [(p[0], p[1]) for p in pts2],
                      shade(color, n_world)))


def collect_sphere(prims, cam, pos, radius, color):
    p = cam.project(pos)
    if p is None:
        return
    r = cam.scale * radius / p[2]
    prims.append((math.dist(pos, cam.eye), "sphere", (p[0], p[1], r), color))


def collect_ground(prims, cam, half, step):
    y = 0.0
    n = (0.0, 1.0, 0.0)
    i = 0
    x = -half
    while x < half - 1e-9:
        z = -half
        j = 0
        while z < half - 1e-9:
            quad = [(x, y, z), (x + step, y, z), (x + step, y, z + step), (x, y, z + step)]
            pts2 = [cam.project(q) for q in quad]
            if all(p is not None for p in pts2):
                c = GRID_A if (i + j) % 2 == 0 else GRID_B
                centroid = (x + step / 2, y, z + step / 2)
                prims.append((math.dist(centroid, cam.eye) + 1e3, "poly",
                              [(p[0], p[1]) for p in pts2], shade(c, n, 0.75)))
            z += step
            j += 1
        x += step
        i += 1


# ------------------------------------------------------------------ 描画
def render_panel(trace, frame_idx, size, preset, color, ss=2):
    w, h = size[0] * ss, size[1] * ss
    eye, target, half, step, fovy = PRESETS[preset]
    cam = Camera(eye, target, w, h, fovy)

    img = Image.new("RGB", (w, h), PANEL_BG)
    draw = ImageDraw.Draw(img)

    prims: list = []
    collect_ground(prims, cam, half, step)

    frame = trace["frames"][min(frame_idx, len(trace["frames"]) - 1)]
    for shape, state in zip(trace["shapes"], frame):
        if shape.get("static"):
            continue  # 地面はタイルで描いている
        pos = (state[0], state[1], state[2])
        quat = (state[3], state[4], state[5], state[6])
        if shape["type"] == "box":
            collect_box(prims, cam, pos, quat, shape["half"], color)
        else:
            collect_sphere(prims, cam, pos, shape["r"],
                           tuple(min(255, int(c * 0.78 + 60)) for c in color))

    prims.sort(key=lambda p: -p[0])
    for _, kind, geom, col in prims:
        if kind == "poly":
            edge = tuple(int(c * 0.72) for c in col)
            draw.polygon(geom, fill=col, outline=edge, width=2)
        else:
            cx, cy, r = geom
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=shade(col, (0, 0.3, 1)))
            hr = r * 0.42
            hx, hy = cx - r * 0.33, cy - r * 0.36
            draw.ellipse((hx - hr, hy - hr, hx + hr, hy + hr),
                         fill=tuple(min(255, int(c * 0.55 + 125)) for c in col))

    return img.resize(size, Image.LANCZOS)


def get_font(size):
    for name in ("meiryo.ttc", "YuGothM.ttc", "msgothic.ttc", "arial.ttf"):
        try:
            return ImageFont.truetype(f"C:/Windows/Fonts/{name}", size)
        except OSError:
            continue
    return ImageFont.load_default()


def build(panels: list[dict], out: Path, preset: str, panel_size=(360, 340),
          fps=14, max_frames=120, subtitle: str = "", colors: int = 64) -> None:
    """panels = [{"trace": dict, "label": str, "rate": str}, ...] を横に並べる。"""
    pad = 14
    header = 66
    n = len(panels)
    w = panel_size[0] * n + pad * (n + 1)
    h = panel_size[1] + header + pad

    f_title = get_font(18)
    f_sub = get_font(12)
    f_hud = get_font(14)

    available = min(len(p["trace"]["frames"]) for p in panels)
    n_frames = min(available, max_frames)
    stride = max(1, available // n_frames)
    base = panels[0]["trace"]
    sim_dt = base["dt"] * base["stride"]

    frames = []
    for k in range(n_frames):
        idx = k * stride
        canvas = Image.new("RGB", (w, h), SURFACE)
        d = ImageDraw.Draw(canvas)

        for i, spec in enumerate(panels):
            x0 = pad + i * (panel_size[0] + pad)
            color = SLOT[i % len(SLOT)]
            canvas.paste(render_panel(spec["trace"], idx, panel_size, preset, color),
                         (x0, header))
            d.rectangle((x0, header, x0 + panel_size[0] - 1,
                         header + panel_size[1] - 1), outline=(225, 224, 217))
            d.rectangle((x0, 26, x0 + 4, 44), fill=color)
            d.text((x0 + 12, 24), spec["label"], font=f_title, fill=INK)
            if spec.get("rate"):
                d.text((x0 + 12, 47), spec["rate"], font=f_sub, fill=MUTED)

        title = {"stack": "10段スタック", "pile": "山積み"}.get(preset, preset)
        head = f"{title}  —  同一シーン・同一パラメータ"
        if subtitle:
            head += f"   /   {subtitle}"
        d.text((pad, 4), head, font=f_sub, fill=MUTED)
        d.text((w - pad - 92, h - 22), f"t = {idx * sim_dt:5.2f} s",
               font=f_hud, fill=MUTED)

        frames.append(canvas.convert("P", palette=Image.ADAPTIVE, colors=colors))

    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True, disposal=2)
    print(f"wrote {out}  ({len(frames)} frames, {out.stat().st_size / 1e6:.1f} MB)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--panel", action="append", required=True,
                    metavar="LABEL=TRACE.json",
                    help="左から順に並べるパネル。複数指定可")
    ap.add_argument("--out", required=True)
    ap.add_argument("--preset", default="stack", choices=list(PRESETS))
    ap.add_argument("--max-frames", type=int, default=120)
    ap.add_argument("--fps", type=int, default=14)
    args = ap.parse_args()

    panels = []
    for spec in args.panel:
        label, path = spec.split("=", 1)
        panels.append({
            "label": label,
            "trace": json.loads(Path(path).read_text(encoding="utf-8")),
            "rate": "",
        })
    build(panels, Path(args.out), args.preset,
          max_frames=args.max_frames, fps=args.fps)


if __name__ == "__main__":
    main()
