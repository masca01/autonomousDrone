#include <cassert>
#include <cmath>
#include <limits>
#include "control/pid_controller.hpp"
bool near(float a,float b) { return std::fabs(a-b)<1e-5F; }
int main() {
  control::PidController pid({0.02F,0,0.003F},{-1,1,-0.2F,0.2F},0);
  auto t=pid.update(0,20,0.01F,0);
  assert(near(t.error,-20) && near(t.output,-0.4F));
  t=pid.update(0,-20,0.01F,0); assert(near(t.output,0.4F));
  t=pid.update(0,0,0.01F,30); assert(near(t.derivative,-0.09F));
  t=pid.update(0,0,0.01F,-30); assert(near(t.derivative,0.09F));
  // A setpoint step has no derivative kick with a measured rate.
  t=pid.update(10,0,0.01F,0); assert(near(t.derivative,0));
  for(int i=0;i<1000;++i)t=pid.update(0,20,0.01F,0);
  assert(near(t.integral,0));
  t=pid.update(0,100,0.01F,0); assert(near(t.output,-1));
  assert(near(pid.update(0,0,0,0).output,-1));
  control::PidController filtered({0,0,0.003F},{-1,1,-0.2F,0.2F},10);
  t=filtered.update(0,0,0.01F,30);
  assert(t.derivative<0 && t.derivative>-0.09F);
  for(int i=0;i<100;++i)t=filtered.update(0,0,0.01F,30);
  assert(near(t.derivative,-0.09F));
}
