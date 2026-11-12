import csv
import json
import math
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import numpy as np
from PIL import Image
from scipy.ndimage import maximum_filter
from scipy.optimize import minimize
import yaml

folder, world = map(Path, sys.argv[1:3])
name = sys.argv[3] if len(sys.argv) > 3 else "competition_slam"
meta = yaml.safe_load((folder / f"{name}.yaml").read_text())
grid = np.flipud(np.array(Image.open(folder / meta["image"])))
res = meta["resolution"]
origin = np.array(meta["origin"][:2])
walls = []
for model in ET.parse(world).findall("world/model"):
    size = model.findtext("link/collision/geometry/box/size")
    if size:
        center = np.array(model.findtext("pose").split()[:2], float)
        half = np.array(size.split()[:2], float) / 2
        walls.append((model.get("name"), center, half))

ij = np.argwhere(grid == 0)
points = origin + (ij[:, ::-1] + 0.5) * res

def transform(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + [x, y]

def distances(points):
    nearest = np.full(len(points), np.inf)
    for _, center, half in walls:
        delta = np.abs(points - center) - half
        dist = np.where(delta.max(axis=1) >= 0,
                        np.linalg.norm(np.maximum(delta, 0), axis=1),
                        -delta.max(axis=1))
        nearest = np.minimum(nearest, dist)
    return nearest

def score(pose):
    d = distances(transform(points, pose))
    return np.minimum(d, 0.18).mean()

fit = minimize(score, [0.15, 1.7, 0], method="Powell",
               bounds=[(-0.25, 0.55), (1.25, 2.15), (-0.16, 0.16)],
               options={"xtol": 1e-6, "ftol": 1e-6})
pose = fit.x

def sample(points):
    local = transform(np.asarray(points) - pose[:2], [0, 0, -pose[2]])
    ij = np.floor((local - origin) / res).astype(int)
    i, j = ij.T
    valid = (i >= 0) & (i < grid.shape[1]) & (j >= 0) & (j < grid.shape[0])
    out = np.full(len(points), 205, dtype=np.uint8)
    out[valid] = grid[j[valid], i[valid]]
    return out

xs = np.arange(-2.85, 2.85, res) + res / 2
ys = np.arange(-3.30, 2.13, res) + res / 2
x, y = np.meshgrid(xs, ys)
xy = np.c_[x.ravel(), y.ravel()]
mapped = sample(xy).reshape(x.shape)
inside = ((np.abs(x) < 2.65) & (y > -1.95) & (y < 1.95)) | ((np.abs(x) < .55) & (y > -3.2) & (y <= -1.95))
for _, center, half in walls:
    inside &= (abs(x-center[0]) > half[0]) | (abs(y-center[1]) > half[1])
widths = []
for label, axis, fixed, lo, hi in [
    ("G lane", 1, .20, 1.50, 1.95),
    ("Upper right lane", 0, 1.42, .55, 1.00),
    ("East lane", 0, 1.20, 2.20, 2.65),
    ("U pocket", 0, 1.17, 1.65, 2.10),
    ("West pocket", 0, 1.65, -.85, -.40),
    ("Divider lane", 1, .75, .40, .85),
]:
    widths_at_offsets = []
    values = np.arange(lo-.12, hi+.12, .002)
    for offset in [-.02, 0., .02]:
        line = np.full((len(values), 2), fixed + offset)
        line[:, axis] = values
        observed = sample(line)
        center = np.argmin(abs(values - (lo+hi)/2))
        lower = values[:center][observed[:center] == 0]
        upper = values[center:][observed[center:] == 0]
        if len(lower) and len(upper) and observed[center] == 254:
            widths_at_offsets.append(round(float(upper[0] - lower[-1]), 3))
    widths.append({"location": label, "world_m": round(hi-lo, 3),
                   "slam_m": float(np.median(widths_at_offsets)) if widths_at_offsets else None,
                   "valid_transects": len(widths_at_offsets)})

telemetry = np.loadtxt(folder/"telemetry.csv", delimiter=",", skiprows=1, usecols=(2,3))
d = distances(transform(points, pose))
summary = {"aligned_pose": pose.tolist(), "occupied_cell_wall_error_median_m": float(np.median(d)),
           "occupied_cell_wall_error_p95_m": float(np.percentile(d, 95)),
           "expected_free_area_known_fraction": float(np.mean(mapped[inside] != 205)),
           "expected_free_area_marked_free_fraction": float(np.mean(mapped[inside] == 254)),
           "passage_widths": widths}
def route_connected(margin):
    rx = math.ceil((.18 + margin) / res - 1e-8)
    ry = math.ceil((.16 + margin) / res - 1e-8)
    from scipy.ndimage import label
    safe = ~maximum_filter(mapped != 254, size=(2*ry+1, 2*rx+1),
                           mode="constant", cval=True)
    components, _ = label(safe)
    def component_at(px, py):
        return int(components[np.argmin(abs(ys-py)), np.argmin(abs(xs-px))])
    g, c = component_at(.15, 1.7), component_at(-1.8, 1.2)
    return {"G_clear": g != 0, "C_clear": c != 0, "G_to_C_connected": g != 0 and g == c}

summary["fixed_heading_grid_connectivity"] = {
    "bare_footprint": route_connected(0),
    "with_2p5cm_margin": route_connected(.025),
}
(folder / f"{name}_audit.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))

fig, axes = plt.subplots(1, 2, figsize=(12.0, 6.6), layout="constrained", sharex=True, sharey=True)
extent = [xs[0]-res/2, xs[-1]+res/2, ys[0]-res/2, ys[-1]+res/2]
classes = np.where(mapped == 0, 0, np.where(mapped == 254, 2, 1))
axes[1].imshow(classes, origin="lower", extent=extent, interpolation="nearest",
               cmap=ListedColormap(["#19222e", "#c6cbd2", "#fdfdfd"]), vmin=0, vmax=2)
for _, center, half in walls:
    for i, ax in enumerate(axes):
        ax.add_patch(Rectangle(center - half, *(2*half),
                               facecolor="#19222e" if i == 0 else "none",
                               edgecolor="#19222e" if i == 0 else "#d98135",
                               linewidth=.7, alpha=1 if i == 0 else .65))
for i, ax in enumerate(axes):
    ax.plot(*telemetry.T, color="#257694", alpha=.7, linewidth=1.15)
    ax.scatter([.15,-1.8], [1.70,1.20], s=25, color="#257694", zorder=4)
    ax.annotate("G", (.15,1.70), xytext=(5, 5), textcoords="offset points")
    ax.annotate("C", (-1.8,1.20), xytext=(5, 5), textcoords="offset points")
    ax.set_aspect("equal")
    ax.set_xlim(extent[:2])
    ax.set_ylim(extent[2:])
    ax.set_xlabel("x (m)")
axes[0].set_ylabel("y (m)")
axes[0].set_title("Gazebo geometry and survey route", loc="left")
axes[1].set_title("SLAM map with Gazebo wall outlines", loc="left")
fig.savefig(folder / f"{name}_comparison.png", dpi=160)
