#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import itertools
import textwrap
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Tuple, List, Any, Optional

import yaml
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

# ===========================
# Appearance / theme
# ===========================
ROOM_NODE_SIZE = 1467
OBJECT_NODE_SIZE = 240
DOOR_NODE_SIZE = 143

FONT_ROOM = 12
FONT_OBJECT = 9
FONT_DISTANCE = 10.5
FONT_TITLE = 20

PALETTE_ROOMS = [
    "#56c6c6", "#f28b82", "#a78bfa", "#7dc4ff", "#ffd166",
    "#f48fb1", "#90caf9", "#ffe082", "#b0bec5", "#80cbc4",
    "#c5e1a5", "#ce93d8"
]
COLOR_OBJECT = "#ff6b6b"
COLOR_DOOR = "#ffa43a"
COLOR_ROOM_EDGE = "#546e7a"
COLOR_OBJ_EDGE = "#9e9e9e"

ROOM_EDGE_WIDTH = 2.4
OBJ_EDGE_WIDTH = 1.25
ROOM_NODE_EDGE = "#455a64"
OBJECT_NODE_EDGE = "#c62828"
DOOR_NODE_EDGE = "#ef6c00"
DPI = 220


# ===========================
# Data structures
# ===========================
@dataclass
class ObjFeat:
    name: str
    world_xy: Optional[Tuple[float, float]]  # None if not provided

@dataclass
class Room:
    id: str
    name: str
    xy: Tuple[float, float]
    objects: List[ObjFeat]

@dataclass
class Edge:
    a: str
    b: str
    cost: Optional[float]


# ===========================
# YAML parsing for YOUR schema
# ===========================
def load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data

def parse_rooms(data: Dict[str, Any]) -> Dict[str, Room]:
    rooms: Dict[str, Room] = {}
    for n in data.get("nodes", []):
        name = n["name"]
        pose = n.get("pose", [0.0, 0.0, 0.0])
        xy = (float(pose[0]), float(pose[1]))

        feats: List[ObjFeat] = []
        for f in n.get("features", []):
            obj_name = f.get("object", "object")
            wf = f.get("Coordinate relative to the world frame")
            if isinstance(wf, list) and len(wf) >= 2:
                feats.append(ObjFeat(obj_name, (float(wf[0]), float(wf[1]))))
            else:
                feats.append(ObjFeat(obj_name, None))

        rooms[name] = Room(id=name, name=name, xy=xy, objects=feats)
    return rooms

def parse_edges(data: Dict[str, Any]) -> List[Edge]:
    raw = data.get("edges", [])
    # Deduplicate opposite directions; keep min cost if both present
    best: Dict[Tuple[str, str], float] = {}
    for e in raw:
        a, b = e["from"], e["to"]
        key = tuple(sorted((a, b)))
        cost = float(e["cost"]) if "cost" in e and e["cost"] is not None else None
        if key not in best:
            best[key] = cost if cost is not None else float("inf")
        else:
            if cost is not None:
                best[key] = min(best[key], cost)
    edges = []
    for (a, b), c in best.items():
        edges.append(Edge(a, b, None if c == float("inf") else c))
    return edges


# ===========================
# Geometry helpers
# ===========================
def interp(a, b, t):
    return (a[0]*(1-t) + b[0]*t, a[1]*(1-t) + b[1]*t)


# ===========================
# Drawing
# ===========================
def draw_graph(
    rooms: Dict[str, Room],
    edges: List[Edge],
    title: str = "Topological Graph",
    out_png: str = "semantic_graph.png",
    out_svg: str = "semantic_graph.svg",
):
    # Build simple graph for any optional layout ops later
    G = nx.Graph()
    for r in rooms.values():
        G.add_node(r.id)
    for e in edges:
        if e.a in rooms and e.b in rooms:
            G.add_edge(e.a, e.b)

    # Room positions are given by pose x,y
    pos = {rid: r.xy for rid, r in rooms.items()}

    # Colors for rooms
    palette = itertools.cycle(PALETTE_ROOMS)
    room_color = {rid: next(palette) for rid in rooms.keys()}

    fig, ax = plt.subplots(figsize=(16, 12))
    ax.axis("off")

    # 1) Room–room edges with distance labels + door nodes
    door_letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    door_idx = 0
    for e in edges:
        if e.a not in pos or e.b not in pos:
            continue
        (x1, y1), (x2, y2) = pos[e.a], pos[e.b]

        # main connection line
        ax.plot([x1, x2], [y1, y2], color=COLOR_ROOM_EDGE, linewidth=ROOM_EDGE_WIDTH, zorder=1)

        # distance label at midpoint
        if e.cost is not None:
            xm, ym = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            ax.text(
                xm, ym, f"{e.cost:.1f}m",
                fontsize=FONT_DISTANCE, ha="center", va="center", color="#1a1a1a",
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#b0bec5", alpha=0.95),
                zorder=6,
            )

        # door node on edge
        t = 0.5
        xd, yd = interp((x1, y1), (x2, y2), t)
        ax.scatter([xd], [yd], s=DOOR_NODE_SIZE, color=COLOR_DOOR,
                   edgecolors=DOOR_NODE_EDGE, linewidths=1.1, zorder=7)
        letter = door_letters[door_idx % len(door_letters)]
        door_idx += 1
        ax.text(xd, yd, letter, fontsize=FONT_OBJECT + 1, ha="center", va="center",
                color="white", fontweight="bold", zorder=8,
                bbox=dict(boxstyle="circle,pad=0.15", fc="black", ec="none", alpha=0.7))

    # 2) Rooms
    for rid, r in rooms.items():
        x, y = r.xy
        ax.scatter([x], [y], s=ROOM_NODE_SIZE, color=room_color[rid],
                   edgecolors=ROOM_NODE_EDGE, linewidths=1.4, zorder=10)
        ax.text(
            x, y, "\n".join(textwrap.wrap(r.name, width=10)),
            fontsize=FONT_ROOM, ha="center", va="center",
            color="white", fontweight="bold", zorder=11,
            bbox=dict(boxstyle="round,pad=0.25", fc=(0, 0, 0, 0.15), ec="none"),
        )

    # 3) Objects
    # If object has world XY -> draw there. Otherwise arrange radially around room.
    radial_cache: Dict[str, float] = defaultdict(lambda: 0.0)
    for rid, r in rooms.items():
        cx, cy = r.xy
        n_missing = sum(1 for o in r.objects if o.world_xy is None)
        # spacing angle for missing objects
        if n_missing:
            step = 2 * math.pi / n_missing

        for o in r.objects:
            if o.world_xy is not None:
                ox, oy = o.world_xy
            else:
                ang = radial_cache[rid]
                radial_cache[rid] += step
                radius = 1.2
                ox = cx + radius * math.cos(ang)
                oy = cy + radius * math.sin(ang)

            # dashed edge room->object
            ax.plot([cx, ox], [cy, oy], linestyle=(0, (3, 2)),
                    color=COLOR_OBJ_EDGE, linewidth=OBJ_EDGE_WIDTH, alpha=0.6, zorder=3)

            # object node + label
            ax.scatter([ox], [oy], s=OBJECT_NODE_SIZE, color=COLOR_OBJECT,
                       edgecolors=OBJECT_NODE_EDGE, linewidths=0.8, zorder=8)
            ax.text(
                ox, oy, "\n".join(textwrap.wrap(o.name, width=8)),
                fontsize=FONT_OBJECT, ha="center", va="center", color="white",
                fontweight="bold", zorder=9,
                bbox=dict(boxstyle="round,pad=0.22", fc=(0, 0, 0, 0.4), ec="none"),
            )

    # 4) Title
    ax.set_title("Topological Graph - Home Environment" if title is None else title,
                 fontsize=FONT_TITLE, fontweight="bold", pad=12)

    # 5) Legend
    legend_elems = [
        Patch(facecolor="#7dc4ff", edgecolor=ROOM_NODE_EDGE, label="Rooms"),
        Patch(facecolor=COLOR_OBJECT, edgecolor=OBJECT_NODE_EDGE, label="Objects/Features"),
        Patch(facecolor=COLOR_DOOR, edgecolor=DOOR_NODE_EDGE, label="Doors/Connections"),
        Line2D([0], [0], color=COLOR_ROOM_EDGE, lw=ROOM_EDGE_WIDTH, label="Room Connections"),
        Line2D([0], [0], color=COLOR_OBJ_EDGE, lw=OBJ_EDGE_WIDTH, linestyle=(0, (2, 3)),
               label="Object–Room Relations"),
    ]
    leg = ax.legend(handles=legend_elems, loc="lower right", frameon=True,
                    framealpha=0.9, fontsize=9, borderpad=0.6)
    leg.get_frame().set_edgecolor("#90a4ae")

    # 6) Environment stats
    n_rooms = len(rooms)
    n_objs = sum(len(r.objects) for r in rooms.values())
    n_conns = len(edges)
    stats = f"Environment Statistics:\n• {n_rooms} Rooms\n• {n_objs} Objects\n• {n_conns} Connections"
    ax.text(1.005, 0.03, stats, transform=ax.transAxes, ha="left", va="bottom",
            fontsize=9.5, bbox=dict(boxstyle="round,pad=0.32", fc="white", ec="#b0bec5", alpha=0.92))

    plt.tight_layout()
    plt.savefig(out_png, dpi=DPI, bbox_inches="tight")
    plt.savefig(out_svg, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {out_png}  |  {out_svg}")


# ===========================
# CLI
# ===========================
if __name__ == "__main__":
    import argparse, os
    parser = argparse.ArgumentParser(description="Render a semantic/topological graph from memory.yaml")
    parser.add_argument("yaml_path", help="Path to YAML with keys: nodes[], edges[]")
    parser.add_argument("--title", default="Topological Graph - Home Environment")
    parser.add_argument("--out", default=None, help="Output basename (no extension)")
    args = parser.parse_args()

    raw = load_yaml(args.yaml_path)
    rooms = parse_rooms(raw)
    edges = parse_edges(raw)
    base = args.out or (os.path.splitext(os.path.basename(args.yaml_path))[0] + "_graph")
    draw_graph(rooms, edges, title=args.title, out_png=base + ".png", out_svg=base + ".svg")
