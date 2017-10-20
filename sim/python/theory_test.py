#!/usr/bin/python3
import unittest
from theory_control import Physics, Norm
import numpy as np

class PhysicsTest(unittest.TestCase):
  def setUp(self):
    self.physics = Physics()

  def test_RudderForces(self):
    thetac = 0.0
    vc = 0.0
    deltar = 0.0
    heel = 0.0
    # Check that all zeros gives zero force:
    Fr, _ = self.physics.RudderForces(thetac, vc, deltar, heel)
    self.assertEqual(Fr, 0.0, "Should have zero force")

    # Basic drag case
    vc = 1.0
    deltar = np.pi / 2.0
    F, gamma = self.physics.RudderForces(thetac, vc, deltar, heel)
    self.assertLess(abs(Norm(gamma - np.pi)), 0.1, "Gamma should be near pi");
    self.assertGreater(F, 0.01, "F should be positive");

    # Basic lifting case (no leeway)
    deltar = 0.5
    F, gamma = self.physics.RudderForces(thetac, vc, deltar, heel)
    self.assertLess(abs(Norm(gamma - np.pi * 0.75)), np.pi * 0.25, "Gamma should be in (pi/2, pi)")
    self.assertGreater(F, 0.01, "F should be positive");

    # Some leeway
    deltar = 0.0
    thetac = np.pi / 4.0
    F, gamma = self.physics.RudderForces(thetac, vc, deltar, heel)
    self.assertLess(abs(Norm(gamma - np.pi * 0.5)), np.pi * 0.25, "Gamma should be in (.25pi, .75pi)")
    self.assertGreater(F, 0.01, "F should be positive");

    # Totally dragging, leeway
    deltar = np.pi / 4.0
    thetac = np.pi / 4.0
    F, gamma = self.physics.RudderForces(thetac, vc, deltar, heel)
    self.assertLess(abs(Norm(gamma - np.pi * 0.75)), 0.05, "Gamma should be near 0.75pi")
    self.assertGreater(F, 0.01, "F should be positive");

  def test_SailForces(self):
    thetaw = 0.0
    vw = 0.0
    deltas = 0.0
    heel = 0.0
    # Check that all zeros gives zero force:
    Fs, _ = self.physics.SailForces(thetaw, vw, deltas, heel)
    self.assertEqual(Fs, 0.0, "Should have zero force")

    # Basic drag case
    vw = 1.0
    deltas = np.pi / 2.0
    F, gamma = self.physics.SailForces(thetaw, vw, deltas, heel)
    self.assertLess(abs(Norm(gamma)), 0.1, "Gamma should be near zero");
    self.assertGreater(F, 0.01, "F should be positive");

    # Basic lifting case (Upwind)
    deltas = 0.5
    thetaw = np.pi
    F, gamma = self.physics.SailForces(thetaw, vw, deltas, heel)
    self.assertLess(abs(Norm(gamma - np.pi * 0.75)), np.pi * 0.25, "Gamma should be in (pi/2, pi)")
    self.assertGreater(F, 0.01, "F should be positive");

    # Realistic close-hauled
    deltas = 0.0
    thetaw = 3.0 * np.pi / 4.0
    F, gamma = self.physics.SailForces(thetaw, vw, deltas, heel)
    self.assertLess(abs(Norm(gamma + np.pi * 0.5)), np.pi * 0.25, "Gamma should be in (-.25pi, -.75pi)")
    self.assertGreater(F, 0.01, "F should be positive");

    # Totally dragging, different angle
    deltas = np.pi / 4.0
    thetaw = np.pi / 4.0
    F, gamma = self.physics.SailForces(thetaw, vw, deltas, heel)
    self.assertLess(abs(Norm(gamma + np.pi * 0.25)), 0.05, "Gamma should be near -.25pi")
    self.assertGreater(F, 0.01, "F should be positive");

  def test_KeelForces(self):
    thetac = 0.0
    vc = 0.0
    heel = 0.0
    # Check that all zeros gives zero force:
    Fr, _ = self.physics.KeelForces(thetac, vc, heel)
    self.assertEqual(Fr, 0.0, "Should have zero force")

    # Basic drag case
    vc = 1.0
    thetac = -np.pi / 2.0
    F, gamma = self.physics.KeelForces(thetac, vc, heel)
    self.assertLess(abs(Norm(gamma + np.pi / 2.0)), 0.1, "Gamma should be near -pi / 2");
    self.assertGreater(F, 0.01, "F should be positive");

    # Basic lifting case
    thetac = np.pi / 4.0
    F, gamma = self.physics.KeelForces(thetac, vc, heel)
    self.assertLess(abs(Norm(gamma - np.pi * 0.5)), np.pi * 0.25, "Gamma should be in (.25pi, .75pi)")
    self.assertGreater(F, 0.01, "F should be positive");


if __name__ == '__main__':
  unittest.main()
