"""Local insertion simulation. SI units; no connection to hardware."""
from pathlib import Path
from xml.etree import ElementTree as ET
import mujoco
import numpy as np

from .labware_family import body_length, sample, to_mjcf

MODELS = Path(__file__).with_name("models")
HOME = np.array([0., 0., 0., -np.pi/2, 0., np.pi/2, 0.])
GAP = .002


def vector(values):
    return " ".join(f"{v:.12g}" for v in values)


class Workcell:
    """20 ms joint-delta interface. Rebuild geometry for each episode seed."""
    def __init__(self, family="cuvette", seed=0, offset_mm=0.):
        self.params = p = sample(family, np.random.default_rng(seed))
        width = p["outer"] if family == "cuvette" else 2*p["radius"]
        robot = ET.parse(MODELS / "fr3.xml").getroot()
        part = ET.fromstring(to_mjcf(p, spawn_height=GAP))
        self.length = body_length(p)
        jaw = robot.find(".//body[@name='fr3_link7']")
        ET.SubElement(jaw, "site", name="grasp", pos="0 0 .165")
        ET.SubElement(jaw, "camera", name="wrist", pos=".035 0 .09", xyaxes="0 1 0 1 0 0")
        for sign, name in ((1,"left"),(-1,"right")):
            finger = ET.SubElement(jaw, "body", name=name,
                                   pos=vector([sign*(width/2+.002), 0, .165]))
            ET.SubElement(finger, "joint", name=name, type="slide", axis=vector([sign,0,0]),
                          range="-.003 .02", damping="2", armature=".003")
            ET.SubElement(finger, "geom", type="box", size=".002 .003 .0015",
                          mass=".015", friction="1 .005 .0001", rgba=".2 .2 .25 1")
            ET.SubElement(robot.find("actuator"), "position", name=name, joint=name,
                          kp="1000", kv="4", forcerange="-2 2", ctrlrange="-.003 .02")
        for tag in ("asset","default"):
            robot.find(tag).extend(part.find(tag))
        for name in ("plate","peg"):
            robot.find("worldbody").append(part.find(f".//body[@name='{name}']"))
        ET.SubElement(robot.find("worldbody"), "light", pos="0 0 2")
        assets = {f.name:f.read_bytes() for f in (MODELS/"assets").glob("*.stl")}
        self.model = m = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"), assets)
        self.data = d = mujoco.MjData(m)
        d.qpos[:7] = HOME
        d.ctrl[:7] = HOME
        mujoco.mj_forward(m,d)
        # Place the gripped portion just below the top, leaving pads above the rim at seat.
        grasp = d.site("grasp").xpos.copy()
        j = m.joint("peg_joint").qposadr[0]
        self.j = j
        spawn = m.qpos0[j:j+3].copy()
        translation = grasp - [0,0,self.length/2-.002] - spawn
        m.body_pos[m.body("plate").id] += translation + [offset_mm/1000,0,0]
        d.qpos[j:j+3] += translation
        self.seat = spawn + translation + [offset_mm/1000,0,-GAP-p["depth"]]
        self.last_delta = np.zeros(7)
        self.closed = True
        self._hold(HOME, 100)

    def _hold(self, target, ticks):
        m,d = self.model,self.data
        d.ctrl[:7] = target
        d.ctrl[7:9] = -.001 if self.closed else .012
        for _ in range(ticks):
            d.qfrc_applied[:7] = d.qfrc_bias[:7]
            mujoco.mj_step(m,d)

    def observe(self):
        d = self.data
        return np.concatenate((d.qpos[:7], d.qvel[:7], self.last_delta)).copy()

    def step(self, delta):
        delta = np.asarray(delta, dtype=float)
        if delta.shape != (7,) or not np.isfinite(delta).all():
            raise ValueError("delta must be seven finite joint increments in radians")
        q = self.data.qpos[:7].copy()
        bounds = self.model.jnt_range[:7]
        target = np.clip(q + np.clip(delta,-.015,.015),bounds[:,0],bounds[:,1])
        self.last_delta = target-q
        self._hold(target,20)
        return self.observe()

    def seat_errors(self):
        d,j = self.data,self.j
        position = d.qpos[j:j+3] - self.seat
        tilt = np.arccos(np.clip(d.body("peg").xmat.reshape(3,3)[2,2],-1,1))
        # Evaluate lateral error where the tube intersects the mouth. Its center
        # may lie above a shallow rack and move farther when a seated tube tilts.
        axis = d.body("peg").xmat.reshape(3, 3)[:, 2]
        dz = self.params["depth"] - self.length/2 - position[2]
        at_mouth = position[:2] + dz * axis[:2] / max(axis[2], 1e-6)
        return np.linalg.norm(at_mouth), position[2], tilt

    def seated(self):
        xy,z,tilt = self.seat_errors()
        return bool(xy <= self.params["clearance"] and abs(z)<.0003 and tilt<np.deg2rad(3))

