#include <startup_dxl.h>
#include "main.h"

extern XM430_bus dxl_bus_1;
extern XM430_bus dxl_bus_2;
extern XM430_bus dxl_bus_3;
extern XM430_bus dxl_bus_4;

extern uint8_t dxl_ID[];
extern uint8_t dxl_ID2[];
extern uint8_t dxl_IDPC[];
extern uint8_t dxl_IDWR[];
extern uint8_t idLength;
extern uint8_t idLength2;
extern uint8_t idLengthPC;
extern uint8_t idLengthWR;

extern uint32_t multiHomePosL[4];
extern uint32_t multiHomePosR[4];
extern uint32_t multiHomePos_1[3];
extern uint32_t multiHomePos_2[3];
extern uint32_t multiHomePos_3[2];

extern uint32_t multiGoalPos_1[3];
extern uint32_t multiGoalPos_2[3];
extern uint32_t multiGoalPos_3[2];

extern uint32_t multiGoalPosL[4], multiGoalPosR[4];

#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)

extern uint8_t DXL_MODE;

void Dynamixel_Startup_Routine (bool torque_disable){
// Enable dynamixels and set control mode...individual version

	// start by disabling all of the motors
	for (int i=0; i<idLength; i++) {
		dxl_bus_1.SetTorqueEn(dxl_ID[i],0x00);
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x00);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthPC; i++) {
		dxl_bus_3.SetTorqueEn(dxl_IDPC[i],0x00);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthWR; i++) {
		dxl_bus_4.SetTorqueEn(dxl_IDWR[i],0x00);
		HAL_Delay(100);
	}

	for (int i=0; i<idLength; i++) {
		dxl_bus_1.TurnOnLED(dxl_ID[i], 0x00); // turn off LED
		dxl_bus_1.SetTorqueEn(dxl_ID[i],0x00);
		dxl_bus_1.SetRetDelTime(dxl_ID[i],0x02); // 4us delay time?
		dxl_bus_1.SetControlMode(dxl_ID[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 168,  84); // KP
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 170,  85);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 172,  80); // KD
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 174,  81);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 176, 116); // goal position
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 178, 117);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 180, 118);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 182, 119);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 184, 104); // goal velocity
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 186, 105);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 188, 106);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 190, 107);
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 192,  94); // feedforward current
		dxl_bus_1.SetIndirectAddress(dxl_ID[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_1.TurnOnLED(dxl_ID[i], 0x01);
		dxl_bus_1.SetTorqueEn(dxl_ID[i],0x01); //to be able to move
		HAL_Delay(100);
	}
	HAL_Delay(100);

	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.TurnOnLED(dxl_ID2[i], 0x00); // turn off LED
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x00);
		dxl_bus_2.SetRetDelTime(dxl_ID2[i],0x02); // 4us delay time?
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
		dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x01); //to be able to move
		HAL_Delay(100);
	}
	HAL_Delay(100);

	for (int i=0; i<idLengthPC; i++) {
		dxl_bus_3.TurnOnLED(dxl_IDPC[i], 0x00); // turn off LED
		dxl_bus_3.SetTorqueEn(dxl_IDPC[i],0x00);
		dxl_bus_3.SetRetDelTime(dxl_IDPC[i],0x02); // 4us delay time?
		dxl_bus_3.SetControlMode(dxl_IDPC[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 168,  84); // KP
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 170,  85);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 172,  80); // KD
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 174,  81);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 176, 116); // goal position
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 178, 117);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 180, 118);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 182, 119);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 184, 104); // goal velocity
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 186, 105);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 188, 106);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 190, 107);
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 192,  94); // feedforward current
		dxl_bus_3.SetIndirectAddress(dxl_IDPC[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_3.TurnOnLED(dxl_IDPC[i], 0x01);
		dxl_bus_3.SetTorqueEn(dxl_IDPC[i],0x01); //to be able to move
		HAL_Delay(100);
	}
	HAL_Delay(100);

	for (int i=0; i<idLengthWR; i++) {
		dxl_bus_4.TurnOnLED(dxl_IDWR[i], 0x00); // turn off LED
		dxl_bus_4.SetTorqueEn(dxl_IDWR[i],0x00);
		dxl_bus_4.SetRetDelTime(dxl_IDWR[i],0x02); // 4us delay time?
		dxl_bus_4.SetControlMode(dxl_IDWR[i], DXL_MODE);
		// set up indirect addresses for faster writing
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 168,  84); // KP
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 170,  85);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 172,  80); // KD
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 174,  81);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 176, 116); // goal position
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 178, 117);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 180, 118);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 182, 119);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 184, 104); // goal velocity
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 186, 105);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 188, 106);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 190, 107);
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 192,  94); // feedforward current
		dxl_bus_4.SetIndirectAddress(dxl_IDWR[i], 194,  95);
		// re-enable motor
		HAL_Delay(100);
		dxl_bus_4.TurnOnLED(dxl_IDWR[i], 0x01);
		dxl_bus_4.SetTorqueEn(dxl_IDWR[i],0x01); //to be able to move
		HAL_Delay(100);
	}
	HAL_Delay(100);

	// setup from David
	// TODO: take this out? could leave it in for more controlled startup of the hand
	ActuatorTransformationL(multiHomePosL, 0, 0, 0, 0);
	ActuatorTransformationR(multiHomePosR, 0, 0, 0, 0);
	ActuatorTransformationL(multiGoalPosL, 0.4, -0.9, 1.2, 0);
	ActuatorTransformationR(multiGoalPosR, -0.4, 0.9, -1.2, 0);
	for (int i=0; i<idLength; i++) {
		multiHomePos_1[i] = multiHomePosL[i];
		multiHomePos_2[i] = multiHomePosR[i];
	}
	multiHomePos_3[0] = multiHomePosL[3];
	multiHomePos_3[1] = multiHomePosR[3];

	for (int i=0; i<idLength; i++) {
		multiGoalPos_1[i] = multiGoalPosL[i];
		multiGoalPos_2[i] = multiGoalPosR[i];
	}
	multiGoalPos_3[0] = multiGoalPosL[3];
	multiGoalPos_3[1] = multiGoalPosR[3];

	// DXL profile smooth
	for (int i=0; i<idLength; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
		dxl_bus_1.SetAccelerationProfile(dxl_ID[i], 100); // 80(17166) rev/min^2
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetVelocityProfile(dxl_ID2[i], 414);
		dxl_bus_2.SetAccelerationProfile(dxl_ID2[i], 100);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthPC; i++) {
		dxl_bus_3.SetVelocityProfile(dxl_IDPC[i], 414);
		dxl_bus_3.SetAccelerationProfile(dxl_IDPC[i], 100);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthWR; i++) {
		dxl_bus_4.SetVelocityProfile(dxl_IDWR[i], 414);
		dxl_bus_4.SetAccelerationProfile(dxl_IDWR[i], 100);
		HAL_Delay(100);
	}

	// set home positions
	if (!torque_disable){
//		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		dxl_bus_1.SetMultGoalPositions(dxl_ID, idLength, multiHomePos_1);
		dxl_bus_2.SetMultGoalPositions(dxl_ID2, idLength2, multiHomePos_2);
		dxl_bus_3.SetMultGoalPositions(dxl_IDPC, idLengthPC, multiHomePos_3);
		dxl_bus_4.SetGoalPosition(9, 2048);
		HAL_Delay(500);
	}

	// DXL profile fast remove gains?
	for (int i=0; i<idLength; i++) {
		dxl_bus_1.SetVelocityProfile(dxl_ID[i], 0);
		dxl_bus_1.SetAccelerationProfile(dxl_ID[i], 0);
		dxl_bus_1.SetPosPGain(dxl_ID[i], 800);
		dxl_bus_1.SetPosDGain(dxl_ID[i], 0);
		dxl_bus_1.SetGoalCurrent(dxl_ID[i], 1193);
		HAL_Delay(100);
	}
	for (int i=0; i<idLength2; i++) {
		dxl_bus_2.SetVelocityProfile(dxl_ID2[i], 0);
		dxl_bus_2.SetAccelerationProfile(dxl_ID2[i], 0);
		dxl_bus_2.SetPosPGain(dxl_ID2[i], 800);
		dxl_bus_2.SetPosDGain(dxl_ID2[i], 0);
		dxl_bus_2.SetGoalCurrent(dxl_ID2[i], 1193);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthPC; i++) {
		dxl_bus_3.SetVelocityProfile(dxl_IDPC[i], 0);
		dxl_bus_3.SetAccelerationProfile(dxl_IDPC[i], 0);
		dxl_bus_3.SetPosPGain(dxl_IDPC[i], 800);
		dxl_bus_3.SetPosDGain(dxl_IDPC[i], 0);
		dxl_bus_3.SetGoalCurrent(dxl_IDPC[i], 1193);
		HAL_Delay(100);
	}
	for (int i=0; i<idLengthWR; i++) {
		dxl_bus_4.SetVelocityProfile(dxl_IDWR[i], 0);
		dxl_bus_4.SetAccelerationProfile(dxl_IDWR[i], 0);
		dxl_bus_4.SetPosPGain(dxl_IDWR[i], 800);
		dxl_bus_4.SetPosDGain(dxl_IDWR[i], 0);
		dxl_bus_4.SetGoalCurrent(dxl_IDWR[i], 2047);
		HAL_Delay(100);
	}

	// motors are enabled by default on startup, but if torque_disable flag is set they can be disabled after the setup
	if (torque_disable){
		for (int i=0; i<idLength; i++) {
			dxl_bus_1.SetTorqueEn(dxl_ID[i],0x00);
			HAL_Delay(100);
		}
		for (int i=0; i<idLength2; i++) {
			dxl_bus_2.SetTorqueEn(dxl_ID2[i],0x00);
			HAL_Delay(100);
		}
		for (int i=0; i<idLengthPC; i++) {
			dxl_bus_3.SetTorqueEn(dxl_IDPC[i],0x00);
			HAL_Delay(100);
		}
		for (int i=0; i<idLengthWR; i++) {
			dxl_bus_4.SetTorqueEn(dxl_IDWR[i],0x00);
			HAL_Delay(100);
		}
	}

	printf("Start Up Routine Finished!!\r\n");

}
