#include <stdint.h>
#include "math_ops.h"

// useful conversions
#define rad2pulse(x) float(x) * (4096.0f/(2*PI)) + 2048 // 0 rad = 2048 = upright pos // Motor -> CW - , CCW + // -pi < x < pi
#define pulse2rad(x) (float(x)-2048) * ((2*PI)/4096.0f) // check that this works
#define rpm2rads(x) float(x)*(0.229f*2.0f*PI)/60.0f // = 0.0239

// transform positions from joint-space to motor-space
void JointPos2MotorPos(float* joint_pos_in, int32_t* motor_pos_out);
// transform positions from motor-space to joint-space
void MotorPos2JointPos(int32_t* motor_pos_in, float* joint_pos_out);

// transform velocities from joint-space to motor-space
void JointVel2MotorVel(float* joint_vel_in, int32_t* motor_vel_out); // NOT IMPLEMENTED
// transform velocities from motor-space to joint-space
void MotorVel2JointVel(int32_t* motor_vel_in, float* joint_vel_out);

// transform torques from joint-scpae to motor-space
void JointTau2MotorTau(float* joint_tau_in, float* motor_tau_out);
// transform torques from motor-space to joint-space
void MotorTau2JointTau(float* motor_tau_in, float* joint_tau_out);

