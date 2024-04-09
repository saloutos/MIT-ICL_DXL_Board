#include "dxl_transforms.h"
#include "math.h"
#include <cstdint>

//Driver Pulley
float rm = 14.38f;   // Motor driver pulley diameter for MCP, PIP, DIP
float rmcr = 15.98f; // Motor driver pulley diameter for MCR

//MCR
float r1 = 15.98f;   // MCR output
float r12 = 16.38f;  // MCP routing
float r13 = 11.48f;  // PIP routing
float r14 = 8.08f;   // DIP routing

//MCP
float r2 = 14.38f;   // MCP output
float r23 = 9.98f;   // PIP routing
float r24 = 5.58f;   // DIP routing

//PIP
float r3 = 14.38f;   // PIP output
float r34 = 9.98f;   // DIP routing

//DIP
float r4 = 14.38f;   // DIP output

// from joint space to actuator space... phidot = Jact*thetadot
float JactL[4][4] = { {(14.38f/14.38f), 0.0f, 0.0f, -(16.38f/15.98f)},
                     {(9.98f/14.38f), (14.38f/14.38f), 0.0f, (11.48f/15.98f)},
                     {(5.58f/14.38f), (9.98f/14.38f), (14.38f/14.38f), -(8.08f/15.98f)},
                     {0.0f, 0.0f, 0.0f, (15.98f/15.98f)} };
float JactR[4][4] = { {(14.38f/14.38f), 0.0f, 0.0f, -(16.38f/15.98f)},
                     {(9.98f/14.38f), (14.38f/14.38f), 0.0f, (11.48f/15.98f)},
                     {(5.58f/14.38f), (9.98f/14.38f), (14.38f/14.38f), -(8.08f/15.98f)},
                     {0.0f, 0.0f, 0.0f, (15.98f/15.98f)} };

// from actuator space to joint space... thetadot = Jjoint*phidot
float JjointL[4][4] = {{1.0, 0.0f, 0.0f, 1.025031289f},
                     {-0.694019471f, 1.0f, 0.0f, -1.429789671f},
                     {0.0936240838f, -0.6940194714f, 1.0f, 1.1001818539f},
                     {0.0f, 0.0f, 0.0f, 1.0f}};
float JjointR[4][4] = {{1.0, 0.0f, 0.0f, 1.025031289f},
                     {-0.694019471f, 1.0f, 0.0f, -1.429789671f},
                     {0.0936240838f, -0.6940194714f, 1.0f, 1.1001818539f},
                     {0.0f, 0.0f, 0.0f, 1.0f}};

// transform positions from joint-space to motor-space
void JointPos2MotorPos(float* joint_pos_in, int32_t* motor_pos_out){
    // joint_pos_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // motor_pos_out: same as joint_pos_in

    // Left finger
    float mcr = joint_pos_in[3];
    float mcp = joint_pos_in[0];
    float pip = joint_pos_in[1];
    float dip = joint_pos_in[2];
    motor_pos_out[3] = (int32_t)round(rad2pulse( (r1/rmcr)* mcr));
    motor_pos_out[0] = (int32_t)round(rad2pulse( (r2/rm)*( -(r12/r1)*mcr + mcp ) ));
    motor_pos_out[1] = (int32_t)round(rad2pulse( (r3/rm)*( (r13/r1)*mcr + (r23/r2)*mcp + pip ) ));
    motor_pos_out[2] = (int32_t)round(rad2pulse( (r4/rm)*( -(r14/r1)*mcr + (r24/r2)*mcp + (r34/r3)*pip + dip ) ));
    // Right finger
    mcr = joint_pos_in[7];
    mcp = joint_pos_in[4];
    pip = joint_pos_in[5];
    dip = joint_pos_in[6];
    motor_pos_out[7] = (int32_t)round(rad2pulse( (r1/rmcr)* mcr));
    motor_pos_out[4] = (int32_t)round(rad2pulse( (r2/rm)*( -(r12/r1)*mcr + mcp ) ));
    motor_pos_out[5] = (int32_t)round(rad2pulse( (r3/rm)*( (r13/r1)*mcr + (r23/r2)*mcp + pip ) ));
    motor_pos_out[6] = (int32_t)round(rad2pulse( (r4/rm)*( -(r14/r1)*mcr + (r24/r2)*mcp + (r34/r3)*pip + dip ) ));
}

// transform positions from motor-space to joint-space
void MotorPos2JointPos(int32_t* motor_pos_in, float* joint_pos_out){
    // motor_pos_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // joint_pos_out: same as motor_pos_in

    // Left finger
    float mcr = pulse2rad(motor_pos_in[3]);
    float mcp = pulse2rad(motor_pos_in[0]);
    float pip = pulse2rad(motor_pos_in[1]);
    float dip = pulse2rad(motor_pos_in[2]);
    joint_pos_out[3] = (rmcr/r1)*mcr;
    joint_pos_out[0] = (r12/r1)*joint_pos_out[3] + (rm/r2)*mcp;
    joint_pos_out[1] = (rm/r3)*pip - (r13/r1)*joint_pos_out[3] - (r23/r2)*joint_pos_out[0];
    joint_pos_out[2] = (r14/r1)*joint_pos_out[3] - (r24/r2)*joint_pos_out[0] - (r34/r3)*joint_pos_out[1] + (rm/r4)*dip;
    // Right finger
    mcr = pulse2rad(motor_pos_in[7]);
    mcp = pulse2rad(motor_pos_in[4]);
    pip = pulse2rad(motor_pos_in[5]);
    dip = pulse2rad(motor_pos_in[6]);
    joint_pos_out[7] = (rmcr/r1)*mcr;
    joint_pos_out[4] = (r12/r1)*joint_pos_out[7] + (rm/r2)*mcp;
    joint_pos_out[5] = (rm/r3)*pip - (r13/r1)*joint_pos_out[7] - (r23/r2)*joint_pos_out[4];
    joint_pos_out[6] = (r14/r1)*joint_pos_out[7] - (r24/r2)*joint_pos_out[4] - (r34/r3)*joint_pos_out[5] + (rm/r4)*dip;
}

// transform velocities from joint-space to motor-space
void JointVel2MotorVel(float* joint_vel_in, int32_t* motor_vel_out){
    // TODO: implement this if necessary
    for (int i=0; i<8; i++){
        motor_vel_out[i] = (int32_t)joint_vel_in[i];
    }
}

// transform velocities from motor-space to joint-space
void MotorVel2JointVel(int32_t* motor_vel_in, float* joint_vel_out){
    // motor_vel_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // joint_vel_out: same as motor_pos_in

    // Left finger
    float mcr = rpm2rads(motor_vel_in[3]);
    float mcp = rpm2rads(motor_vel_in[0]);
    float pip = rpm2rads(motor_vel_in[1]);
    float dip = rpm2rads(motor_vel_in[2]);
    joint_vel_out[3] = (rmcr/r1)*mcr;
    joint_vel_out[0] = (r12/r1)*joint_vel_out[3] + (rm/r2)*mcp;
    joint_vel_out[1] = (rm/r3)*pip - (r13/r1)*joint_vel_out[3] - (r23/r2)*joint_vel_out[0];
    joint_vel_out[2] = (r14/r1)*joint_vel_out[3] - (r24/r2)*joint_vel_out[0] - (r34/r3)*joint_vel_out[1] + (rm/r4)*dip;
    // Right finger
    mcr = rpm2rads(motor_vel_in[7]);
    mcp = rpm2rads(motor_vel_in[4]);
    pip = rpm2rads(motor_vel_in[5]);
    dip = rpm2rads(motor_vel_in[6]);
    joint_vel_out[7] = (rmcr/r1)*mcr;
    joint_vel_out[4] = (r12/r1)*joint_vel_out[7] + (rm/r2)*mcp;
    joint_vel_out[5] = (rm/r3)*pip - (r13/r1)*joint_vel_out[7] - (r23/r2)*joint_vel_out[4];
    joint_vel_out[6] = (r14/r1)*joint_vel_out[7] - (r24/r2)*joint_vel_out[4] - (r34/r3)*joint_vel_out[5] + (rm/r4)*dip;
}

// transform torques from joint-space to motor-space
void JointTau2MotorTau(float* joint_tau_in, float* motor_tau_out){
    // joint_tau_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // motor_tau_out: same as joint_tau_in

    // motor_tau = Jjoint^T * joint_tau
    for(int i=0; i<4; i++){
        // Left finger
        motor_tau_out[i] = JjointL[0][i]*joint_tau_in[0] + JjointL[1][i]*joint_tau_in[1] 
                            + JjointL[2][i]*joint_tau_in[2] + JjointL[3][i]*joint_tau_in[3];
        // Right finger
        motor_tau_out[i+4] = JjointR[0][i]*joint_tau_in[4] + JjointR[1][i]*joint_tau_in[5] 
                            + JjointR[2][i]*joint_tau_in[6] + JjointR[3][i]*joint_tau_in[7];
    }
}

// transform torques from motor-space to joint-space
void MotorTau2JointTau(float* motor_tau_in, float* joint_tau_out){
    // motor_tau_in: 0:mcp_l, 1:pip_l, 2:dip_l, 3:mcr_l, 4:mcp_r, 5:pip_r, 6:dip_r, 7:mcr_r
    // joint_tau_out: same as motor_tau_in

    // joint_tau = Jact^T * motor_tau
    for(int i=0; i<4; i++){
        // Left finger
		joint_tau_out[i] = JactL[0][i]*motor_tau_in[0] + JactL[1][i]*motor_tau_in[1] 
                            + JactL[2][i]*motor_tau_in[2] + JactL[3][i]*motor_tau_in[3];
        // Right finger
		joint_tau_out[i] = JactR[0][i]*motor_tau_in[4] + JactR[1][i]*motor_tau_in[5] 
                            + JactR[2][i]*motor_tau_in[6] + JactR[3][i]*motor_tau_in[7];
	}
}