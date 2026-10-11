// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/telemetry.hpp"

namespace ord {

void writeCsvHeader(std::ostream& os, std::size_t wheelCount) {
    os << "time,x,y,z,speed_ms,speed_kmh,vy,sideslip_rad,yaw_rate,roll,pitch,ax,ay,az,kinetic_energy,"
          "steer_in,throttle_in,brake_in,steer_angle,throttle_out,gear,clutch,engine_rpm,engine_torque,"
          "tcs_cut,esc_yaw_error,countersteer,solver_iters";
    for (std::size_t w = 0; w < wheelCount; ++w) {
        os << ",w" << w << "_contact,w" << w << "_comp,w" << w << "_fz,w" << w << "_omega,w" << w << "_kappa,w" << w
           << "_alpha,w" << w << "_fx,w" << w << "_fy,w" << w << "_brake,w" << w << "_abs,w" << w << "_esc";
    }
    os << '\n';
}

void writeCsvRow(std::ostream& os, const TelemetryFrame& f) {
    os << f.time << ',' << f.position.x << ',' << f.position.y << ',' << f.position.z << ',' << f.speed << ','
       << msToKmh(f.speed) << ',' << f.lateralVelocity << ',' << f.sideslip << ',' << f.yawRate << ',' << f.rollAngle
       << ',' << f.pitchAngle << ',' << f.accelBody.x << ',' << f.accelBody.y << ',' << f.accelBody.z << ','
       << f.kineticEnergy << ',' << f.steerInput << ',' << f.throttleInput << ',' << f.brakeInput << ','
       << f.steerAngle << ',' << f.throttleOut << ',' << f.gear << ',' << f.clutchEngagement << ','
       << f.drivetrain.engineRpm << ',' << f.drivetrain.engineTorque << ',' << f.assists.tcsThrottleCut << ','
       << f.assists.escYawError << ',' << f.assists.countersteerAngle << ',' << f.drivetrain.activeSolverIterations;
    for (std::size_t w = 0; w < f.wheelCount; ++w) {
        const WheelTelemetry& t = f.wheels[w];
        os << ',' << t.contact << ',' << t.compression << ',' << t.load << ',' << t.omega << ',' << t.slipRatio << ','
           << t.slipAngle << ',' << t.fx << ',' << t.fy << ',' << t.brakeTorque << ',' << f.assists.absActive[w] << ','
           << f.assists.escBrakeTorque[w];
    }
    os << '\n';
}

}  // namespace ord
