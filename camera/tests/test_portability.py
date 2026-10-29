import unittest
import numpy as np
from coordinate_transform.geometry import BUNDLED_CORE, RobotConvention, convention_matches


class PortabilityTests(unittest.TestCase):
    def test_bundled_math_keeps_verified_identity(self):
        bundled = RobotConvention(BUNDLED_CORE)
        default = RobotConvention()
        self.assertTrue(convention_matches(bundled.evidence(), default.evidence()))
        np.testing.assert_allclose(bundled.rotation([-.9, .2, .7]), default.rotation([-.9, .2, .7]))

    def test_identity_ignores_location_but_rejects_other_math(self):
        evidence = RobotConvention(BUNDLED_CORE).evidence()
        relocated = dict(evidence, source='/another/checkout/core.py')
        self.assertTrue(convention_matches(evidence, relocated))
        self.assertFalse(convention_matches(evidence, dict(relocated, sha256='different')))
