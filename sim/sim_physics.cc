#include "sim_physics.h"

SimulatorSaoud2013::SimulatorSaoud2013() : dt(0.01/*s*/),
                                           ls(.5/*m*/),
                                           lr(.05/*m*/),
                                           hs(1.5/*m*/),
                                           hr(-.3/*m*/),
                                           hk(-.6/*m*/),
                                           deltas(0/*rad*/),
                                           deltasdot(0/*rad/s*/),
                                           deltar(0/*rad*/),
                                           deltardot(0/*rad/s*/),
                                           rr(1/*m*/),
                                           rs(.5/*m*/),
                                           rhoair(1.225/*kg/m^3*/),
                                           rhowater(1000/*kg/m^3*/),
                                           Ss(5/*m^2*/),
                                           Sr(.25/*m^2*/),
                                           Sk(.5/*m^2*/),
                                           c0s(.5), c1s(.5), c0r(c0s), c1r(c1s), c0k(c0s), c1k(c1s),
                                           m0(40/*kg*/),
                                           nabla(m0 / rhowater) {
  // TODO(james):
  // Not really sure what these should initialize to. They refer to the
  // resistance along the hull through the water. Definitely shouldn't be zero.
  c1 *= 0;
  c3 *= 0;
  c2 *= 0;
  c4 *= 0;

//  J0 << ; // TODO(james) ?

  MA *= 0; // TODO(james): Might be able to get away with 0...
  // TODO(james): Calculations for metacenter/center of buoyancy.
  B = TBI(Vector3d(0, 0, -.5));

  x *= 0;
  v *= 0;
  vw *= 0;
  vc *= 0;
  RBI = RBI.Identity();
  omega *= 0;
}
