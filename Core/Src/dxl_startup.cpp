#include <dxl_startup.h>
#include "main.h"

extern XM430_bus dxl_bus_1;
extern XM430_bus dxl_bus_2;
extern XM430_bus dxl_bus_3;

extern uint8_t dxl_ID1[];
extern uint8_t dxl_ID2[];
extern uint8_t dxl_ID3[];
extern uint8_t idLength1;
extern uint8_t idLength2;
extern uint8_t idLength3;

extern uint8_t DXL_MODE;

void Dynamixel_Shutdown_Routine(){
	// disable all of the motors
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x00);
		HAL_Delay(10);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x00);
		HAL_Delay(10);
	}
	for (int i=0; i<idLength3; i++) {
		dxl_bus_3.SetTorqueEn(dxl_ID3[i],0x00);
		HAL_Delay(10);
	}
}

void Dynamixel_Startup_Routine (bool torque_disable){
	// Enable dynamixels and set control mode...individual version
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.TurnOnLED(dxl_ID1[i], 0x00); // turn off LED
		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x00);
		dxl_bus_1.SetRetDelTime(dxl_ID1[i],0x02); // 4us delay time
		dxl_bus_1.SetControlMode(dxl_ID1[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 168,  84); // KP
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 170,  85);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 172,  80); // KD
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 174,  81);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 176, 116); // goal position
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 178, 117);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 180, 118);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 182, 119);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 184, 104); // goal velocity
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 186, 105);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 188, 106);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 190, 107);
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 192,  94); // feedforward current
		dxl_bus_1.SetIndirectAddress(dxl_ID1[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_1.TurnOnLED(dxl_ID1[i], 0x01);
		dxl_bus_1.SetTorqueEn(dxl_ID1[i],0x01); // to be able to move
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.TurnOnLED(dxl_ID2[i], 0x00); // turn off LED
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x00);
		dxl_bus_2.SetRetDelTime(dxl_ID2[i],0x02); // 4us delay time
		dxl_bus_2.SetControlMode(dxl_ID2[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 168,  84); // KP
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 170,  85);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 172,  80); // KD
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 174,  81);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 176, 116); // goal position
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 178, 117);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 180, 118);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 182, 119);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 184, 104); // goal velocity
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 186, 105);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 188, 106);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 190, 107);
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 192,  94); // feedforward current
		dxl_bus_2.SetIndirectAddress(dxl_ID2[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_2.TurnOnLED(dxl_ID2[i], 0x01);
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x01); // to be able to move
		HAL_Delay(100);
	}
	for (int i=0; i<idLength3; i++) {
		dxl_bus_3.TurnOnLED(dxl_ID3[i], 0x00); // turn off LED
		dxl_bus_3.SetTorqueEn(dxl_ID3[i],0x00);
		dxl_bus_3.SetRetDelTime(dxl_ID3[i],0x02); // 4us delay time
		dxl_bus_3.SetControlMode(dxl_ID3[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 168,  84); // KP
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 170,  85);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 172,  80); // KD
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 174,  81);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 176, 116); // goal position
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 178, 117);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 180, 118);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 182, 119);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 184, 104); // goal velocity
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 186, 105);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 188, 106);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 190, 107);
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 192,  94); // feedforward current
		dxl_bus_3.SetIndirectAddress(dxl_ID3[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_3.TurnOnLED(dxl_ID3[i], 0x01);
		dxl_bus_3.SetTorqueEn(dxl_ID3[i],0x01); // to be able to move
		HAL_Delay(100);
	}

	// set smooth DXL profile
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID1[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
		dxl_bus_1.SetAccelerationProfile(dxl_ID1[i], 100); // 80(17166) rev/min^2
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetVelocityProfile(dxl_ID2[i], 414);
		dxl_bus_2.SetAccelerationProfile(dxl_ID2[i], 100);
		HAL_Delay(100);
	}
	for (int i=0; i<idLength3; i++) {
		dxl_bus_3.SetVelocityProfile(dxl_ID3[i], 414);
		dxl_bus_3.SetAccelerationProfile(dxl_ID3[i], 100);
		HAL_Delay(100);
	}

	// controlled setup to send fingers to zero joint angles
	float home_joint_pos[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	int32_t home_motor_pos[8];
	JointPos2MotorPos(home_joint_pos, home_motor_pos);
	int32_t pos1[3];
	int32_t pos2[3];
	for (int i=0; i<3; i++) {
		pos1[i] = home_motor_pos[i];
		pos2[i] = home_motor_pos[i+4];
	}
	int32_t pos3[2];
	pos3[0] = home_motor_pos[3];
	pos3[1] = home_motor_pos[7];
	if (!torque_disable){
		dxl_bus_1.SetMultGoalPositions(dxl_ID1, idLength1, pos1);
		dxl_bus_2.SetMultGoalPositions(dxl_ID2, idLength2, pos2);
		dxl_bus_3.SetMultGoalPositions(dxl_ID3, idLength3, pos3);
		HAL_Delay(100);
	}

	// re-set to fast DXL profile, if not in current control mode set current limit
	for (int i=0; i<idLength1; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID1[i], 0);
		dxl_bus_1.SetAccelerationProfile(dxl_ID1[i], 0);
		dxl_bus_1.SetPosPGain(dxl_ID1[i], 0);
		dxl_bus_1.SetPosDGain(dxl_ID1[i], 0);
		if(DXL_MODE!=0x00){ dxl_bus_1.SetGoalCurrent(dxl_ID1[i], 1193); }
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetVelocityProfile(dxl_ID2[i], 0);
		dxl_bus_2.SetAccelerationProfile(dxl_ID2[i], 0);
		dxl_bus_2.SetPosPGain(dxl_ID2[i], 0);
		dxl_bus_2.SetPosDGain(dxl_ID2[i], 0);
		if(DXL_MODE!=0x00){ dxl_bus_2.SetGoalCurrent(dxl_ID2[i], 1193); }
		HAL_Delay(100);
	}
	for (int i=0; i<idLength3; i++) {
		dxl_bus_3.SetVelocityProfile(dxl_ID3[i], 0);
		dxl_bus_3.SetAccelerationProfile(dxl_ID3[i], 0);
		dxl_bus_3.SetPosPGain(dxl_ID3[i], 0);
		dxl_bus_3.SetPosDGain(dxl_ID3[i], 0);
		if(DXL_MODE!=0x00){ dxl_bus_3.SetGoalCurrent(dxl_ID3[i], 1193); }
		HAL_Delay(100);
	}

	// motors are enabled by default on startup, but if torque_disable flag is set they can be disabled after the setup
	if (torque_disable){
		Dynamixel_Shutdown_Routine();
	}

	printf("Start Up Routine Finished!!\r\n");

}
