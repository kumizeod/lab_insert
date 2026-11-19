"""Gate 0: is the geometry/contact right BEFORE any robot or RL?

For N sampled assemblies per family, three passive probes (no robot):
  A  centred + aligned, dropped from 1 mm     -> must SEAT        (clearance kept)
  B  offset by 0.5 x clearance                -> must SEAT        (no false walls)
  C  part 0.2 mm OVERSIZE vs hole, centred    -> must NOT seat    (rim is solid,
                                                 contact not too soft)
Plus a stability check on the seated part (tiny masses at 1 ms step).

Usage:  python -m lab_insert.drop_test --n 200
"""
from __future__ import annotations

import argparse
import numpy as np
import mujoco

import xml.etree.ElementTree as ET
from .labware_family import FAMILIES, sample, to_mjcf

def oversize_xml(p: dict, over=0.2e-3) -> str:
    """Hole built for a part shrunk by (clearance + over): the real part is
    then `over` LARGER than the hole on every side."""
    q = dict(p)
    s = p["clearance"] + over
    if p["family"] == "cuvette":
        q["outer"] = p["outer"] - 2 * s
        q["corner_r"] = max(p["corner_r"] - s, 0.05e-3)
    else:
        q["radius"] = p["radius"] - s
    hole = ET.fromstring(to_mjcf(q))
    part = ET.fromstring(to_mjcf(p))
    for parent_path, selector in (("asset", "mesh"), ("worldbody", "body")):
        dest = hole.find(parent_path)
        source = part.find(parent_path)
        def belongs(node):
            name = node.get("name", "")
            return name == "peg" if selector == "body" else name.startswith("peg_")
        for node in list(dest.findall(selector)):
            if belongs(node):
                dest.remove(node)
        for node in source.findall(selector):
            if belongs(node):
                dest.append(node)
    return ET.tostring(hole, encoding="unicode")

MM = 1e-3


def run_probe(xml: str, p: dict, dx: float, t_end=0.6):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    j = m.joint("peg_joint").qposadr[0]
    d.qpos[j] += dx                       # lateral offset along x
    z0 = d.qpos[j + 2]
    zs = []
    for _ in range(int(t_end / m.opt.timestep)):
        mujoco.mj_step(m, d)
        zs.append(d.qpos[j + 2])
    zs = np.array(zs)
    drop = z0 - zs[-1]                    # how far it went down
    # seated: dropped the full hole depth + spawn gap (minus 0.3 mm tolerance)
    position_ok = np.linalg.norm(d.qpos[j:j + 2]) <= p["clearance"]
    upright = d.body("peg").xmat.reshape(3, 3)[2, 2] > np.cos(np.deg2rad(3))
    seated = (abs(drop - p["depth"] - MM) < 0.3 * MM and position_ok and upright)
    tail = zs[-int(0.1 / m.opt.timestep):]
    jitter = tail.max() - tail.min()      # z motion in the last 100 ms
    blocked = abs(drop - MM) < 0.3 * MM and position_ok and upright and d.ncon > 0
    finite = np.isfinite(zs).all() and np.isfinite(d.qpos).all() and not d.warning.number.any()
    return seated, blocked, jitter, finite


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)

    if args.n < 1:
        ap.error("--n must be positive")
    passed = True
    for fam in FAMILIES:
        stats = dict(A=0, B=0, C=0, unstable=0, nan=0)
        worst = 0.0
        for _ in range(args.n):
            p = sample(fam, rng)
            xml = to_mjcf(p)
            c = p["clearance"]
            sA, _, jA, fA = run_probe(xml, p, 0.0)
            sB, _, jB, fB = run_probe(xml, p, 0.5 * c)
            _, blocked, jC, fC = run_probe(oversize_xml(p), p, 0.0)
            stats["A"] += sA
            stats["B"] += sB
            stats["C"] += blocked
            stats["nan"] += not (fA and fB and fC)
            stats["unstable"] += max(jA, jB, jC) > 0.05 * MM
            worst = max(worst, jA, jB, jC)
        n = args.n
        passed &= all(stats[k] == n for k in ("A", "B", "C")) and not (stats["unstable"] or stats["nan"])
        print(f"{fam:10s}  A seat {stats['A']}/{n}   B seat {stats['B']}/{n}   "
              f"C blocked {stats['C']}/{n}   jitter>50um {stats['unstable']}   "
              f"NaN {stats['nan']}   worst jitter {worst / MM * 1000:.1f} um")


    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
