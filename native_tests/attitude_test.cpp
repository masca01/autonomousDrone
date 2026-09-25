#include <cassert>
#include <cmath>
#include "control/gyro_calibration.hpp"
#include "control/complementary_attitude.hpp"
int main() {
 control::GyroCalibration c;
 imu_test::Sample s{22,0,0,1,0.3f,0.1f,-0.7f};
 for(int i=0;i<499;++i) assert(!c.add(s));
 assert(c.add(s)); assert(std::fabs(c.bias(2)+0.7f)<1e-5f);
 c.reset(); for(int i=0;i<200;++i)c.add(s);
 auto moving=s; moving.gz=40; assert(!c.add(moving)); assert(c.count()==0);
 for(int i=0;i<500;++i) { auto noisy=s; noisy.gx=(i%2?1.0f:-1.0f); c.add(noisy); }
 assert(!c.ready()); assert(c.count()==0);
 control::ComplementaryAttitude a;
 auto v=a.update(0,0.5f,0.8660254f,0,0,0,0.01f);
 for(int i=0;i<500;++i)v=a.update(0,0.5f,0.8660254f,0,0,0,0.01f);
 assert(std::fabs(v.roll-30)<0.01f);
 a.reset(); a.update(0,0,1,0,0,0,0.01f);
 for(int i=0;i<100;++i)v=a.update(0,0,1,0,0,90,0.01f);
 assert(std::fabs(v.yaw-90)<0.01f);
 for(int i=0;i<100;++i)v=a.update(0,0,1,0,0,0,0.01f);
 assert(std::fabs(v.yaw-90)<0.01f);
}
