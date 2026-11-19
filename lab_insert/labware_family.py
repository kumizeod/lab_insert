"""Parametric collision envelopes for rigid labware in a straight rack.
Tube contours are engineering approximations. See README for sources and limits.
"""
from __future__ import annotations

import math
from xml.etree import ElementTree as ET
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
    # 2 mL round-bottom attached-cap tube; dimensions are provisional.
    "microtube": dict(
        radius=(5.2 * MM, 5.6 * MM),        # placeholder ~10.5-11 mm OD
        clearance=(0.20 * MM, 1.00 * MM),   # racks are usually loose
        depth=(10 * MM, 30 * MM),
        mass=(1e-3, 4e-3),                  # tube + liquid
    ),
    # Individual 0.2 mL PCR tube with tapered lower body.
    "pcr": dict(
        radius=(2.9 * MM, 3.1 * MM),        # placeholder ~6 mm OD
        clearance=(0.05 * MM, 0.30 * MM),
        depth=(8 * MM, 15 * MM),
        mass=(0.3e-3, 0.6e-3),              # very light: stability risk
    ),
}


# SKU -> (nominal capacity mL, outside diameter mm, catalog height mm).
# See README for sources and the assumed cap contribution to total height.
ROUND_TUBES = {
    "YA1030": (5, 13.5, 56.0),
    "DC7MCT-5ML": (5, 13.5, 53.5),
    "HR10005": (2, 10.63, 41.91),
    "HR10024": (5, 13.8, 52.37),
    "HR10007": (7, 15.63, 62.41),
    "HR10023": (10, 16.9, 82.24),
}


def sample(family: str, rng: np.random.Generator, *, sku=None, cap_state=None, **fixed) -> dict:
    """One assembly's parameters (dict). `fixed` overrides any sampled value."""
    spec = FAMILIES[family]
    p = {k: (fixed[k] if k in fixed else rng.uniform(*v)) for k, v in spec.items()}
    p["family"] = family
    if family == "microtube":
        sku = "YA1030" if sku is None else sku
        capacity, diameter, height = ROUND_TUBES[sku]
        p.update(sku=sku, capacity_ml=capacity, radius=fixed.get("radius", diameter*MM/2),
                 body_length=fixed.get("body_length", (height-.9)*MM),
                 cap_state="open" if cap_state is None else cap_state)
        if p["cap_state"] not in ("open", "closed"):
            raise ValueError("cap_state must be open or closed")
        # Generic single-hole fixture; depth and clearance are engineering settings.
        p["depth"] = fixed.get("depth", p["body_length"] * rng.uniform(.57,.68))
    elif sku is not None or cap_state is not None:
        raise ValueError("sku and cap_state are specific to round-bottom flip-cap tubes")
    if family == "pcr":
        p["body_length"] = fixed.get("body_length", .01975)
    if family == "pcr":
        p["taper_length"] = fixed.get("taper_length", .5 * p["body_length"])
    p["friction"] = fixed.get("friction", float(np.exp(rng.uniform(np.log(0.25), np.log(2.0)))))
    return p


def to_mjcf(p: dict, *, spawn_height=1 * MM, standalone=True) -> str:
    """Generate a straight rack opening and the family's external contact geometry."""
    if p["family"] == "cuvette":
        outline = rounded_rect(p["outer"], p["outer"], p["corner_r"])
        rim = 6 * MM
    else:
        outline = {"type": "circle", "radius": p["radius"],
                   "max_error": min(1e-4, 0.2 * p["clearance"])}
        rim = 4 * MM
    xml = mjfit.peg(
        peg=outline,
        clearance=p["clearance"],
        peg_length=body_length(p),      # part sticks out above the hole
        plate_thickness=p["depth"],
        bottom_thickness=2 * MM,             # closed bottom = "seated" stop
        peg_mass=p["mass"],
        friction=p["friction"],
        spawn_height=spawn_height,
        standalone=standalone,
        plate_size=None if p["family"] != "cuvette" else p["outer"] + 2 * rim,
    )

    if p["family"] == "cuvette":
        return xml
    root = ET.fromstring(xml)
    asset = root.find("asset")
    body = root.find(".//body[@name='peg']")
    body.remove(body.find("geom"))
    asset.remove(asset.find("mesh[@name='peg_0']"))
    r, length, mass = p["radius"], body_length(p), p["mass"]
    bottom, top = -length / 2, length / 2
    rgba = "0.80 0.87 0.93 1"

    def geom(name, kind, z, fraction, **attrs):
        ET.SubElement(body, "geom", name=name, type=kind, pos=f"0 0 {z}",
                      rgba=rgba, mass=str(mass*fraction), **attrs)

    if p["family"] == "microtube":
        # Lower half of the sphere is exposed; its upper half lies within the barrel.
        transition = bottom + r
        geom("rounded_base", "sphere", transition, .12, size=str(r))
    else:
        taper = p.get("taper_length", .50 * length)
        if not 0 < taper < length:
            raise ValueError("taper_length must be between zero and tube length")
        transition = bottom + taper
        # Small blunt tip; convex frustum with circular sections.
        rings = [(bottom, .12*r), (transition, r)]
        vertices = [(rad*np.cos(a), rad*np.sin(a), z) for z,rad in rings
                    for a in np.linspace(0,2*np.pi,64,endpoint=False)]
        ET.SubElement(asset, "mesh", name="peg_taper",
                      vertex=" ".join(f"{v:.12g}" for point in vertices for v in point))
        geom("tapered_base", "mesh", 0, .12, mesh="peg_taper")

    barrel_top = top - .0012 if p.get("cap_state") == "open" else top
    geom("barrel", "cylinder", (transition+barrel_top)/2, .80,
         size=f"{r} {(barrel_top-transition)/2}")
    # Closed cap and fixed hinge envelope. Moving lids need a separate joint model.
    half_cap = .00045
    if p.get("cap_state") == "open":
        # Photo-style fixed open pose; the lid disk lies beside the mouth.
        # An annular lip and recessed barrel surface make the mouth visible.
        for i in range(24):
            angle = 2*np.pi*i/24
            ET.SubElement(body, "geom", name=f"lip_{i}", type="box", mass="0",
                          pos=f"{.97*r*np.cos(angle)} {.97*r*np.sin(angle)} {top-.0006}",
                          quat=f"{np.cos(angle/2)} 0 0 {np.sin(angle/2)}",
                          size=f"{.045*r} {.14*r} .0006", rgba=rgba)
        lid_y = 2.65 * r
        ET.SubElement(body, "geom", name="open_cap", type="cylinder", rgba=rgba,
                      pos=f"0 {lid_y} {top}", size=f"{1.14*r} {half_cap}",
                      mass=str(.075*mass))
        ET.SubElement(body, "geom", name="hinge_tab", type="box", rgba=rgba,
                      pos=f"0 {1.27*r} {top}", size=f"{.22*r} {.36*r} {.0004}",
                      mass=str(.005*mass))
        return ET.tostring(root, encoding="unicode")
    geom("closed_cap", "cylinder", top+half_cap, .08, size=f"{1.14*r} {half_cap}")
    ET.SubElement(body,"geom",name="hinge_tab",type="box",mass="0",rgba=rgba,
                  pos=f"0 {1.07*r} {top}",size=f"{.26*r} {.20*r} {.0005}")
    return ET.tostring(root, encoding="unicode")


def body_length(p):
    """Length below the cap, independent of the selected rack depth (metres)."""
    if p["family"] == "cuvette":
        return p["depth"] + 5*MM
    return p.get("body_length", .039 if p["family"] == "microtube" else .01975)


def lateral_tolerance(p):
    """Include the polygonal rack approximation and 10 um soft-contact allowance."""
    if p["family"] == "microtube":
        return p["clearance"] + min(.0001, .2*p["clearance"]) + .00001
    return p["clearance"]
