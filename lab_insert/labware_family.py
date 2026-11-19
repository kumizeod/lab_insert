"""Parametric labware "sample + receptacle" families for mjfit (v0).

Every call samples ONE assembly = (sample part, receptacle hole). Thousands of
variants come from sampling the parameters, not from hand-made assets.

!!! All nominal numbers below are PLACEHOLDERS. Replace each range with your
    own caliper measurements (several brands / several racks), then widen by a
    margin. The ranges, not the nominal values, are what the policy learns from.

v0 limits (deliberate):
  * Parts are straight extrusions (mjfit's model). The conical tip of
    microtubes / PCR tubes and the conical PCR well are NOT modelled yet (v1).
  * No cap / hinge, no neighbouring holes (v1: rack arrays for wrong-hole).
"""
from __future__ import annotations

import math
import numpy as np
import mjfit

MM = 1e-3


def rounded_rect(w, h, r, n_arc=8):
    """CCW rounded rectangle, arcs circumscribed (same convention as mjfit)."""
    r = min(r, 0.49 * min(w, h))
    k = 1.0 / math.cos(math.pi / (4 * n_arc))  # circumscribe each arc
    cx, cy = w / 2 - r, h / 2 - r
    pts = []
    for (sx, sy, a0) in [(1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)]:
        for i in range(n_arc + 1):
            a = math.radians(a0 + 90 * i / n_arc)
            rr = r if i in (0, n_arc) else r * k
            pts.append((sx * cx + rr * math.cos(a), sy * cy + rr * math.sin(a)))
    return np.array(pts)


# ---------------------------------------------------------------- families
# Each entry: ranges to sample uniformly (lo, hi). Units: metres / kg.
FAMILIES = {
    # Standard 10 mm path-length cuvette: outer 12.5 x 12.5 mm (nominal).
    "cuvette": dict(
        outer=(12.4 * MM, 12.6 * MM),
        corner_r=(0.3 * MM, 1.0 * MM),
        clearance=(0.10 * MM, 0.50 * MM),   # holder fit varies a lot: MEASURE
        depth=(15 * MM, 30 * MM),           # how deep it seats in the holder
        mass=(3e-3, 20e-3),                 # PS ... quartz, + liquid
    ),
    # 1.5 / 2 mL microcentrifuge tube body in a rack hole (cylinder, v0).
    "microtube": dict(
        radius=(5.2 * MM, 5.6 * MM),        # placeholder ~10.5-11 mm OD
        clearance=(0.20 * MM, 1.00 * MM),   # racks are usually loose
        depth=(10 * MM, 30 * MM),
        mass=(1e-3, 4e-3),                  # tube + liquid
    ),
    # 0.2 mL PCR tube upper body in a well (cylinder, v0; real well is conical).
    "pcr": dict(
        radius=(2.9 * MM, 3.1 * MM),        # placeholder ~6 mm OD
        clearance=(0.05 * MM, 0.30 * MM),
        depth=(8 * MM, 15 * MM),
        mass=(0.3e-3, 0.6e-3),              # very light: stability risk
    ),
}


def sample(family: str, rng: np.random.Generator, **fixed) -> dict:
    """One assembly's parameters (dict). `fixed` overrides any sampled value."""
    spec = FAMILIES[family]
    p = {k: (fixed[k] if k in fixed else rng.uniform(*v)) for k, v in spec.items()}
    p["family"] = family
    p["friction"] = fixed.get("friction", float(np.exp(rng.uniform(np.log(0.25), np.log(2.0)))))
    return p


def to_mjcf(p: dict, *, spawn_height=1 * MM, standalone=True) -> str:
    """mjfit MJCF: the part is given; the hole is its exact outward offset."""
    if p["family"] == "cuvette":
        outline = rounded_rect(p["outer"], p["outer"], p["corner_r"])
        rim = 6 * MM
    else:
        outline = {"type": "circle", "radius": p["radius"],
                   "max_error": min(1e-4, 0.2 * p["clearance"])}
        rim = 4 * MM
    return mjfit.peg(
        peg=outline,
        clearance=p["clearance"],
        peg_length=p["depth"] + 5 * MM,      # part sticks out above the hole
        plate_thickness=p["depth"],
        bottom_thickness=2 * MM,             # closed bottom = "seated" stop
        peg_mass=p["mass"],
        friction=p["friction"],
        spawn_height=spawn_height,
        standalone=standalone,
        plate_size=None if p["family"] != "cuvette" else p["outer"] + 2 * rim,
    )
