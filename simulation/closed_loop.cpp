// Educational one-axis plant driven by the real firmware PID implementation.
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include "control/pid_controller.hpp"

int main(int argc, char** argv) {
  const float kp = argc>1 ? std::stof(argv[1]) : 0.02F;
  const float kd = argc>2 ? std::stof(argv[2]) : 0.003F;
  const int substeps = argc>3 ? std::stoi(argv[3]) : 10;
  if (!std::isfinite(kp) || !std::isfinite(kd) || kp<0 || kd<0 || substeps<1 || substeps>1000) return 1;
  constexpr double pi=3.14159265358979323846;
  constexpr double inertia=0.0025; // kg m^2, illustrative, not measured
  constexpr double drag=0.004; // N m s/rad
  constexpr double max_torque=0.08; // N m per unit controller correction
  constexpr double actuator_tau=0.03; // seconds; lumped motor/thrust lag
  constexpr double dt=0.01; // Same nominal 100 Hz as pid_demo
  const double h=dt/substeps;
  control::PidController pid({kp,0,kd},{-1,1,-0.2F,0.2F},10);
  double angle=20*pi/180, rate=0, torque=0;
  std::cout << "time_s,angle_deg,rate_deg_s,correction,p_term,d_term,torque_nm,gust_nm\n" << std::setprecision(10);
  for (int step=0; step<=1000; ++step) {
    const double t=step*dt;
    const auto terms=pid.update(0,angle*180/pi,dt,rate*180/pi);
    const double gust=(step>=400 && step<420) ? 0.02 : 0;
    std::cout << t << ',' << angle*180/pi << ',' << rate*180/pi << ','
              << terms.output << ',' << terms.proportional << ',' << terms.derivative << ','
              << torque << ',' << gust << '\n';
    // Zero-order-held controller command. Integrate the plant at a finer rate.
    for (int j=0;j<substeps;++j) {
      torque+=(max_torque*terms.output-torque)*(1-std::exp(-h/actuator_tau));
      rate+=(torque+gust-drag*rate)/inertia*h;
      angle+=rate*h;
    }
    if (!std::isfinite(angle) || !std::isfinite(rate)) return 2;
  }
}
