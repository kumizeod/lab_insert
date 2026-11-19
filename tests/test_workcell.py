import unittest
import numpy as np
from lab_insert.workcell import Workcell
from lab_insert.baseline import aligned_trial
from lab_insert.labware_family import FAMILIES


class WorkcellTest(unittest.TestCase):
    def test_released_part_remains_seated(self):
        for family in FAMILIES:
            with self.subTest(family=family):
                env = Workcell(family, seed=0)
                self.assertTrue(all(aligned_trial(env)))
                self.assertFalse(env.data.warning.number.any())

    def test_offset_is_not_a_success(self):
        env = Workcell("cuvette", seed=0, offset_mm=3)
        self.assertFalse(aligned_trial(env)[-1])

    def test_step_timing_and_delta(self):
        env = Workcell()
        time = env.data.time
        q = env.observe()[:7]
        obs = env.step(np.full(7, .001))
        np.testing.assert_allclose(env.data.ctrl[:7], q+.001)
        self.assertAlmostEqual(env.data.time-time, .020)
        self.assertEqual(obs.shape,(21,))

