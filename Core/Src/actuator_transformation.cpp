#include "actuator_transformation.h"
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

#define transf(x) (uint32_t)round(rad2pulse(x))

void ActuatorTransformationL(uint32_t (&transform)[4], float mcp, float pip, float dip, float mcr) { // M1=mcp, M2=pip, M3=dip, M4=mcr
    //Left finger

    transform[3] = (uint32_t)round(rad2pulse( (r1/rmcr)* mcr));
    transform[0] = (uint32_t)round(rad2pulse( (r2/rm)*( -(r12/r1)*mcr + mcp ) ));
    transform[1] = (uint32_t)round(rad2pulse( (r3/rm)*( (r13/r1)*mcr + (r23/r2)*mcp + pip ) ));
    transform[2] = (uint32_t)round(rad2pulse( (r4/rm)*( -(r14/r1)*mcr + (r24/r2)*mcp + (r34/r3)*pip + dip ) ));

}

void ActuatorTransformationR(uint32_t (&transform)[4], float mcp, float pip, float dip, float mcr) { // M1=mcp, M2=pip, M3=dip, M4=mcr

    // Right finger

    // transform[3] = (uint32_t)round(rad2pulse( (15.98f/15.48f) * -mcr));
    // transform[0] = (uint32_t)round(rad2pulse( (13.88f/15.48f) * ( -(16.38f/13.88f)*mcr + mcp ) ));
    // transform[1] = (uint32_t)round(rad2pulse( (13.88f/15.48f) * ( -(11.48f/13.88f)*mcr - (9.68f/13.88f)*mcp - pip ) ));
    // transform[2] = (uint32_t)round(rad2pulse( (13.88f/15.48f) * ( -(8.08f/13.88f)*mcr + (5.48f/13.88f)*mcp + (5.48f/13.88f)*pip + dip ) ));

//    transform[3] = (uint32_t)round(rad2pulse( (15.98f/15.48f)* mcr));
//    transform[0] = (uint32_t)round(rad2pulse( (13.88f/15.48f)*( -(16.38f/13.88f)*mcr + mcp ) ));
//    transform[1] = (uint32_t)round(rad2pulse( (13.88f/15.48f)*( (11.48f/13.88f)*mcr + (9.68f/13.88f)*mcp + pip ) ));
//    transform[2] = (uint32_t)round(rad2pulse( (13.88f/15.48f)*( -(8.08f/13.88f)*mcr + (5.48f/13.88f)*mcp + (5.48f/13.88f)*pip + dip ) ));

    transform[3] = (uint32_t)round(rad2pulse( (r1/rmcr)* mcr));
    transform[0] = (uint32_t)round(rad2pulse( (r2/rm)*( -(r12/r1)*mcr + mcp ) ));
    transform[1] = (uint32_t)round(rad2pulse( (r3/rm)*( (r13/r1)*mcr + (r23/r2)*mcp + pip ) ));
    transform[2] = (uint32_t)round(rad2pulse( (r4/rm)*( -(r14/r1)*mcr + (r24/r2)*mcp + (r34/r3)*pip + dip ) ));


}

void InverseActuatorTransformationL(double* d_transform, int32_t m1, int32_t m2, int32_t m3, int32_t m4) { // M1=mcp, M2=pip, M3=dip, M4=mcr 
    // Left finger, going from actuator space angles to joint space angles
    
    d_transform[3] = (rmcr/r1)*pulse2rad(m4);
    d_transform[0] = (r12/r1)*d_transform[3] + (rm/r2)*pulse2rad(m1);
    d_transform[1] = (rm/r3)*pulse2rad(m2) - (r13/r1)*d_transform[3] - (r23/r2)*d_transform[0];
    d_transform[2] = (r14/r1)*d_transform[3] - (r24/r2)*d_transform[0] - (r34/r3)*d_transform[1] + (rm/r4)*pulse2rad(m3);

}

void InverseActuatorTransformationR(double* d_transform, int32_t m1, int32_t m2, int32_t m3, int32_t m4) { // M1=mcp, M2=pip, M3=dip, M4=mcr
    // Right finger, going from actuator space angles to joint space angles

    // d_transform[3] = -(15.48f/15.98f)*pulse2rad(m4);
    // d_transform[0] = (16.38f/13.88f)*d_transform[3] + (15.48f/13.88f)*pulse2rad(m1);
    // d_transform[1] = -(15.48f/13.88f)*pulse2rad(m2) - (11.48f/13.88f)*d_transform[3] - (9.68f/13.88f)*d_transform[0];
    // d_transform[2] = (8.08f/13.88f)*d_transform[3] - (5.48f/13.88f)*d_transform[0] - (5.48f/13.88f)*d_transform[1] + (15.48f/13.88f)*pulse2rad(m3);

//    d_transform[3] = (15.48f/15.98f)*pulse2rad(m4);
//    d_transform[0] = (16.38f/13.88f)*d_transform[3] + (15.48f/13.88f)*pulse2rad(m1);
//    d_transform[1] = (15.48f/13.88f)*pulse2rad(m2) - (11.48f/13.88f)*d_transform[3] - (9.68f/13.88f)*d_transform[0];
//    d_transform[2] = (8.08f/13.88f)*d_transform[3] - (5.48f/13.88f)*d_transform[0] - (5.48f/13.88f)*d_transform[1] + (15.48f/13.88f)*pulse2rad(m3);

    d_transform[3] = (rmcr/r1)*pulse2rad(m4);
    d_transform[0] = (r12/r1)*d_transform[3] + (rm/r2)*pulse2rad(m1);
    d_transform[1] = (rm/r3)*pulse2rad(m2) - (r13/r1)*d_transform[3] - (r23/r2)*d_transform[0];
    d_transform[2] = (r14/r1)*d_transform[3] - (r24/r2)*d_transform[0] - (r34/r3)*d_transform[1] + (rm/r4)*pulse2rad(m3);
}

void InverseActuatorVelocityTransformationL(double* v_transform, float vm1, float vm2, float vm3, float vm4){
    // Left finger, going from actuator space velocities to joint space velocities

    v_transform[3] = (rmcr/r1)*vm4;
    v_transform[0] = (r12/r1)*v_transform[3] + (rm/r2)*vm1;
    v_transform[1] = (rm/r3)*vm2 - (r13/r1)*v_transform[3] - (r23/r2)*v_transform[0];
    v_transform[2] = (r14/r1)*v_transform[3] - (r24/r2)*v_transform[0] - (r34/r3)*v_transform[1] + (rm/r4)*vm3;

}

void InverseActuatorVelocityTransformationR(double* v_transform, float vm1, float vm2, float vm3, float vm4){
    // Right finger, going from actuator space velocities to joint space velocities 

    // v_transform[3] = -(15.48f/15.98f)*vm4;
    // v_transform[0] = (16.38f/13.88f)*v_transform[3] + (15.48f/13.88f)*vm1;
    // v_transform[1] = -(15.48f/13.88f)*vm2 - (11.48f/13.88f)*v_transform[3] - (9.68f/13.88f)*v_transform[0];
    // v_transform[2] = (8.08f/13.88f)*v_transform[3] - (5.48f/13.88f)*v_transform[0] - (5.48f/13.88f)*v_transform[1] + (15.48f/13.88f)*vm3;

//    v_transform[3] = (15.48f/15.98f)*vm4;
//    v_transform[0] = (16.38f/13.88f)*v_transform[3] + (15.48f/13.88f)*vm1;
//    v_transform[1] = (15.48f/13.88f)*vm2 - (11.48f/13.88f)*v_transform[3] - (9.68f/13.88f)*v_transform[0];
//    v_transform[2] = (8.08f/13.88f)*v_transform[3] - (5.48f/13.88f)*v_transform[0] - (5.48f/13.88f)*v_transform[1] + (15.48f/13.88f)*vm3;
    v_transform[3] = (rmcr/r1)*vm4;
    v_transform[0] = (r12/r1)*v_transform[3] + (rm/r2)*vm1;
    v_transform[1] = (rm/r3)*vm2 - (r13/r1)*v_transform[3] - (r23/r2)*v_transform[0];
    v_transform[2] = (r14/r1)*v_transform[3] - (r24/r2)*v_transform[0] - (r34/r3)*v_transform[1] + (rm/r4)*vm3;

}

