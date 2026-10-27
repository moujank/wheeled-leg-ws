"""Render reproducible plots and a self-contained, locally viewable report."""
from pathlib import Path
import base64
import json
import numpy as np


def write_report(out, mapper, planned, trajectory, scans, poses, angles, progress):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image, ImageDraw

    out = Path(out)
    s = mapper.spec
    extent = [s.xmin, s.xmin+s.width*s.resolution, s.ymin, s.ymin+s.height*s.resolution]
    pixels = np.full(mapper.log_odds.shape, 0.55)
    pixels[mapper.free()] = 0.96
    pixels[mapper.occupied()] = 0.04
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(pixels, cmap="gray", vmin=0, vmax=1, origin="lower", extent=extent)
    xy = np.array(poses)
    ax.plot(xy[:, 0], xy[:, 1], color="#3975cc", alpha=0.7, lw=1.4, label="Preset mapping tour")
    for i, item in enumerate(planned):
        path = np.array(item["path"])
        ax.plot(path[:, 0], path[:, 1], lw=2, label=f"A* route {i+1}: {item['length_m']:.1f} m")
    t = np.array(trajectory)
    ax.plot(t[:, 0], t[:, 1], "--", color="#a72474", lw=1.2, label="Kinematic follower")
    ax.scatter(*planned[0]["start"], s=110, c="#15a363", marker="o", zorder=5)
    ax.scatter(*planned[0]["goal"], s=180, c="#d43f53", marker="*", zorder=5)
    ax.set(xlabel="x (m)", ylabel="y (m)", title="Isaac Sim lidar measurements → occupancy grid → goal paths")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out / "mapping_and_paths.png", dpi=160)
    plt.close(fig)

    # Animated evidence is reconstructed from the recorded real simulator scans.
    from scan_nav.core import OccupancyMapper
    replay = OccupancyMapper(s)
    frames = []
    every = max(1, len(scans)//45)
    for i, (scan, pose, angle) in enumerate(zip(scans, poses, angles)):
        replay.integrate(pose, angle, scan, 20)
        if i % every and i != len(scans)-1:
            continue
        rgb = np.full((*replay.log_odds.shape, 3), 135, np.uint8)
        rgb[replay.free()] = (239, 243, 247)
        rgb[replay.occupied()] = (28, 37, 49)
        img = Image.fromarray(np.flipud(rgb)).resize((900, 660), Image.Resampling.NEAREST)
        draw = ImageDraw.Draw(img)
        c, r = mapper.cell(pose[0], pose[1])
        x, y = (c+.5)*6, (s.height-r-.5)*6
        draw.ellipse((x-6,y-6,x+6,y+6), fill=(0,175,130))
        draw.rectangle((0, 0, 900, 28), fill=(20,30,45))
        draw.text((12, 7), f"PhysX lidar map | scan {i+1}/{len(scans)} | ideal odometry | preset tour", fill="white")
        frames.append(img)
    frames[0].save(out / "mapping_progress.gif", save_all=True, append_images=frames[1:], duration=110, loop=0)
    metrics = json.loads((out / "metrics.json").read_text(encoding="utf-8"))

    def embed(name):
        p = out / name
        if not p.exists():
            return ""
        mime = "image/gif" if name.endswith("gif") else "image/png"
        return f'<img alt="{name}" src="data:{mime};base64,{base64.b64encode(p.read_bytes()).decode()}">'

    cards = "".join(f'<div class="card"><b>{value}</b><span>{label}</span></div>' for label, value in [
        ("Laser scans", metrics["scan_count"]), ("Beams / scan", metrics["beams_per_scan"]),
        ("Goal error", f'{metrics["goal_error_m"]:.3f} m'),
        ("Minimum follower clearance", f'{metrics["trajectory_min_clearance_m"]:.2f} m')])
    html = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>Isaac Sim 建图与路径规划验证</title>
<style>body{{font:16px/1.7 system-ui;background:#101824;color:#e7edf5;max-width:1100px;margin:40px auto;padding:0 24px}}h1{{font-size:32px}}.cards{{display:flex;gap:14px;flex-wrap:wrap}}.card{{background:#1c2b3e;border-radius:12px;padding:18px;min-width:175px}}b{{display:block;font-size:27px;color:#61dcb6}}span{{color:#b7c5d6}}img{{max-width:100%;border-radius:12px;margin:14px 0}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#1c2b3e;padding:18px}}a{{color:#61dcb6}}</style>
<h1>Isaac Sim：激光建图与目标点路径规划</h1>
<p>实际运行结果 · Isaac Sim 4.5.0 · PhysX 360° LiDAR · 自定义双轮足机器人资产</p>
<div class="cards">{cards}</div>
<p>本次使用<strong>仿真真值里程计、预设采集路线和简化运动驱动</strong>。地图由真实运行的仿真雷达扫描生成，规划器只读取扫描地图。
未执行 RL 策略，未运行 SLAM Toolbox/Nav2，未验证回环优化、传感器里程计、自主探索或真实动力学导航。</p>
<h2>扫描生成的地图和三条目标路线</h2>{embed('mapping_and_paths.png')}
<h2>建图过程回放</h2>{embed('mapping_progress.gif')}
<h2>Isaac Sim 场景截图</h2>{embed('isaac_scene.png')}
<h2>验收记录</h2><pre>{json.dumps(metrics, ensure_ascii=False, indent=2).replace('<', '&lt;')}</pre>
</html>'''
    (out / "report.html").write_text(html, encoding="utf-8")
