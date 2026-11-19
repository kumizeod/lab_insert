import unittest
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from lab_insert.labware_family import sample, to_mjcf


class TubeProfileTest(unittest.TestCase):
    def test_lower_contact_surfaces_and_closed_lid(self):
        for family in ("microtube", "pcr"):
            with self.subTest(family=family):
                p = sample(family, np.random.default_rng(17))
                xml = to_mjcf(p)
                model = mujoco.MjModel.from_xml_string(xml)
                self.assertAlmostEqual(model.body("peg").mass[0], p["mass"])
                self.assertGreater(model.geom("closed_cap").size[0], p["radius"])
                if family == "microtube":
                    self.assertEqual(model.geom("rounded_base").type[0], mujoco.mjtGeom.mjGEOM_SPHERE)
                else:
                    mesh = ET.fromstring(xml).find(".//mesh[@name='peg_taper']")
                    v = np.fromstring(mesh.get("vertex"), sep=" ").reshape(-1, 3)
                    tip = v[v[:, 2] == v[:, 2].min()]
                    shoulder = v[v[:, 2] == v[:, 2].max()]
                    self.assertLess(np.linalg.norm(tip[0, :2]), np.linalg.norm(shoulder[0, :2]))
