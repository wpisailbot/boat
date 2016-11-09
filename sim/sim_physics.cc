#include "sim_physics.h"

// Note on vertical distances:
// because the origin is at the CoM, and the keel drags the CoM way down,
// heights will seem too positive, as a more intuitive origin would be at the
// deck.
SimulatorSaoud2013::SimulatorSaoud2013(float _dt)
    : dt(_dt /*s*/),
      ls(1.5 /*m*/),
      lr(.05 /*m*/),
      hs(2. /*m*/),
      hr(0 /*m*/),
      hk(-.3 /*m*/),
      deltas(.8 /*rad*/),
      deltasdot(0 /*rad/s*/),
      deltar(0 /*rad*/),
      deltardot(0 /*rad/s*/),
      rr(1 /*m*/),
      rs(.5 /*m*/),
      rhoair(1.225 /*kg/m^3*/),
      rhowater(1000 /*kg/m^3*/),
      Ss(5 /*m^2*/),
      Sr(.05 /*m^2*/),
      Sk(.3 /*m^2*/),
      c0s(.2),
      c1s(.4),
      c0r(c0s),
      c1r(c1s),
      c0k(c0s),
      c1k(c1s),
      m0(40 /*kg*/),
      nabla(m0 / rhowater) {
  // TODO(james):
  // Not really sure what these should initialize to. They refer to the
  // resistance along the hull through the water. Definitely shouldn't be zero.
  c1 = 25 * c1.Ones();
  c3 = c3.Ones();
  c1 = c1.Zero();
  c3 = c3.Zero();
  c2 = c2.Zero();
  c4 = c4.Zero();

//  J0 << ; // TODO(james) ?
  J0 = J0.Zero();
  J0.diagonal() = Vector3d(50, 50, 30);

  MA = MA.Zero(); // TODO(james): Might be able to get away with 0...
  // TODO(james): Calculations for metacenter/center of buoyancy.
  // Note: Center of buoyancy is above origin, b/c origin is at cetner of mass,
  // which is dragged down by keel.
  B_B = Vector3d(0, 0, .5);

  x = x.Zero();
  v = v.Zero();
  vw = vw.Zero();
  vc = vc.Zero();
  RBI = RBI.Identity();
  omega = omega.Zero();
}
