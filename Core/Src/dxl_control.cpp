#include <dxl_control.h>
#include <pack_can_functions.h>
#include <startup_dxl.h>
#include "fdcan.h"
#include "math_ops.h"
#include <stdio.h>
#include <stdint.h>
#include "crc.h"
#include "string.h"
#include <XM430_bus.h>
#include "math.h"
#include "actuator_transformation.h"
#include "ForceSensor.h"
#include "neural_nets.h"
#define rad2pulse_t(x) uint32_t(rad2pulse(x))
#define deg2rad(x) float((PI/180.0f)*x)
#define pulse2deg(x) (360.0f/4096.0f)*(float)(x-2048.0f)
#define VERSION_NUMBER 1.00f

uint32_t eval_time[3] = {0, 0, 0};
uint32_t cycle_count= 0;
uint32_t old_cycle_count = 0;
uint32_t f1_ct = 0;
uint32_t f2_ct = 0;
uint32_t t1_ct = 0;
uint32_t t2_ct = 0;
uint32_t tp_ct = 0;
uint32_t cf_ct = 0;
uint32_t ct[6];

// System Control Mode
volatile bool CURR_CONTROL = true;
volatile bool MODE_SELECTED = false;
uint8_t DXL_MODE;

// Initialize CAN FD
FDCAN_RxHeaderTypeDef rxMsg_sys, rxMsg_sense;
FDCAN_TxHeaderTypeDef txHeader_fd_joints, txHeader_fd_sens;
FDCAN_FilterTypeDef sys_can_filt, sense_can_filt;
uint8_t txMsg_fd_joints[48];
uint8_t txMsg_fd_sens[48];

uint8_t sys_rx_buf[48]; // TODO: could make this shorter?
uint8_t sense_rx_buf[8]; // TODO: could make this shorter?

/* Initialization */
XM430_bus dxl_bus_1(&huart1, RTS1_GPIO_Port, RTS1_Pin);
XM430_bus dxl_bus_2(&huart2, RTS2_GPIO_Port, RTS2_Pin);
XM430_bus dxl_bus_3(&huart7, RTS7_GPIO_Port, RTS7_Pin);
XM430_bus dxl_bus_4(&huart5, RTS5_GPIO_Port, RTS5_Pin); // use just for wrist motor? // getting weird

uint8_t dxl_IDs[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
uint8_t dxl_ID[] =  {1, 2, 3}; //, 4, 5, 6, 7, 8, 9};
uint8_t dxl_ID2[] = {5, 6, 7};
uint8_t dxl_IDPC[] = {4, 8};
uint8_t dxl_IDWR[] = {9};
uint8_t idLength = sizeof(dxl_ID) / sizeof(dxl_ID[0]);
uint8_t idLength2 = sizeof(dxl_ID2) / sizeof(dxl_ID2[0]);
uint8_t idLengthPC = 2;
uint8_t idLengthWR = 1;


volatile uint8_t tx_flag_1;
volatile uint8_t tx_flag_2;
volatile uint8_t tx_flag_3;
volatile uint8_t tx_flag_5;
volatile uint8_t rx_flag_1;
volatile uint8_t rx_flag_2;
volatile uint8_t rx_flag_3;
volatile uint8_t rx_flag_5;

// Variables for force sensor data
int32_t pressure_raw1[8];
int32_t pressure_raw2[8];
uint8_t tof1[8];
uint8_t tof2[8];
uint8_t palmTOF;

int32_t phal1[3];
int32_t phal2[3];
int32_t phal3[3];
int32_t phal4[3];

float force1[5];
float force2[5];

extern NeuralNet sensorB3;
ForceSensor forcesensor1(0, &sensorB3);
extern NeuralNet sensorB4;
ForceSensor forcesensor2(1, &sensorB4);


// Variables for dynamixel bus
uint32_t goalDes1[4];
uint32_t goalDes2[4];
uint32_t goalPos[8];
float currentPos[9];
float currentVel[9];
float currentCur[9];
float currentJointTau[9];

float Kt = 2.0f * (3.7f / 2.7f);
float Ktinv = 1/Kt;
float current_limit = 2.70f; //2.30f; // in A, max is 3.2(?)

float Kt_WR = 2.0f * (8.9f / 5.5f);
float Ktinv_WR = 1.0f/Kt_WR;
float current_limit_WR = 4.40f; // in A

float pulse_to_rad = (2.0f*PI)/4096.0f; // = 0.001534
float rpm_to_rads = (0.229f*2.0f*PI)/60.0f; // = 0.0239

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


int32_t dxl_position[9];
int32_t dxl_velocity[9];
int16_t dxl_current[9];
uint32_t abad_pos[2];
float desired_current[9];
uint16_t current_command[9];

// CAN command variables
float dxl_pos_des[9] = {0.12f, 0.12f, 0.12f, 0.3f, -0.12f, -0.12f, -0.12f, 0.3f, 0.0f};
float dxl_vel_des[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float dxl_tff_des[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float dxl_kp[9] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
float dxl_kd[9] = {0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f};

// more command variables
uint32_t pos_command[9];
uint32_t vel_command[9];
uint32_t cur_command[9]; // TODO: remove conflict with current_command variable in line 92!!!
uint32_t kp_command[9];
uint32_t kd_command[9];

// demo trajectory variables
uint32_t multiHomePosL[4];
uint32_t multiHomePosR[4];
uint32_t rtPos_L[4];
uint32_t rtPos_R[4];
uint16_t multiGoalCurrent[4];
uint32_t multiGoalPosL[4], multiGoalPosR[4];
uint32_t multiHomePos_1[3];
uint32_t multiHomePos_2[3];
uint32_t multiHomePos_3[2];

uint32_t multiGoalPos_1[3], multiGoalPos_2[3], multiGoalPos_3[2];

uint32_t rtPos_1[3];
uint32_t rtPos_2[3];
uint32_t rtPos_3[2];
uint32_t trajPos[9];

double dt = 0.001; //1.5ms
uint32_t time_step = 0;
int32_t dxl_time;
float freq1 = 1; //9.5
float freq2 = freq1/2.0f;
float amp1 = (PI/180)*20;
float amp2 = (PI/180)*12;
uint8_t idx_freq = 0;
float max_freq =22;
bool updown = true;


void GetBulkData_DMA()
{

	// set up transmissions
	uint8_t data_len = 10;
	uint8_t start_addr = PRESENT_CURRENT;
	dxl_bus_1.sRead(data_len, start_addr, dxl_ID, idLength);
	dxl_bus_1.rPacketLength = 8 + idLength*(4+data_len);
	dxl_bus_2.sRead(data_len, start_addr, dxl_ID2, idLength2);
	dxl_bus_2.rPacketLength = 8 + idLength2*(4+data_len);
	dxl_bus_3.sRead(data_len, start_addr, dxl_IDPC, idLengthPC);
	dxl_bus_3.rPacketLength = 8 + idLengthPC*(4+data_len);
	dxl_bus_4.sRead(data_len, start_addr, dxl_IDWR, idLengthWR);
	dxl_bus_4.rPacketLength = 8 + idLengthWR*(4+data_len);
	// DMA transmissions
	tx_flag_1 = 0;
	rx_flag_1 = 0;

	tx_flag_2 = 0;
	rx_flag_2 = 0;

	tx_flag_3 = 0;
	rx_flag_3 = 0;

	tx_flag_5 = 0;
	rx_flag_5 = 0;

//	dxl_bus_1.sendIPacket_DMA();
//	while(!tx_flag_1){;}
//	dxl_bus_1.getRPacket_DMA();
//	while(!rx_flag_1){;}
//
//	dxl_bus_2.sendIPacket_DMA();
//	while(!tx_flag_2){;}
//	dxl_bus_2.getRPacket_DMA();
//	while(!rx_flag_2){;}
//
//	dxl_bus_3.sendIPacket_DMA();
//	while(!tx_flag_3){;}
//	dxl_bus_3.getRPacket_DMA();
//	while(!rx_flag_3){;}
//
//	dxl_bus_4.sendIPacket_DMA();
//	while(!tx_flag_5){;}
//	dxl_bus_4.getRPacket_DMA();
//	while(!rx_flag_5){;}

	dxl_bus_1.sendIPacket_DMA();
	dxl_bus_2.sendIPacket_DMA();
	dxl_bus_3.sendIPacket_DMA();
	dxl_bus_4.sendIPacket_DMA();
	while((!tx_flag_1)||(!tx_flag_2)||(!tx_flag_3)||(!tx_flag_5)){;}
	dxl_bus_1.getRPacket_DMA();
	dxl_bus_2.getRPacket_DMA();
	dxl_bus_3.getRPacket_DMA();
	dxl_bus_4.getRPacket_DMA();
	while((!rx_flag_1)||(!rx_flag_2)||(!rx_flag_3)||(!rx_flag_5)){;}

//	while((rx_flag_1==0)||(rx_flag_2==0)||(rx_flag_6==0)||(rx_flag_7==0)){;}
	// for loop to extract ret_vals from rPackets
	int i = 10;
	uint8_t ret_vals1[data_len]; // could make this a 2D array?
	for(int j=0; j<idLength; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals1[k] = dxl_bus_1.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_ID[j]-1;
		dxl_current[h] = (int16_t) ((uint16_t)ret_vals1[0] | (((uint16_t)ret_vals1[1]<<8)&0xFF00));
		dxl_velocity[h] = (int32_t) ((uint32_t)ret_vals1[2] | (((uint32_t)ret_vals1[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals1[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals1[5]<<24)&0xFF000000));
		dxl_position[h] = (int32_t) ((uint32_t)ret_vals1[6] | (((uint32_t)ret_vals1[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals1[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals1[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}
	i = 10; // reset index
	uint8_t ret_vals2[data_len]; // could make this a 2D array?
	for(int j=0; j<idLength2; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals2[k] = dxl_bus_2.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_ID2[j]-1;
		dxl_current[h] = (int16_t) ((uint16_t)ret_vals2[0] | (((uint16_t)ret_vals2[1]<<8)&0xFF00));
		dxl_velocity[h] = (int32_t) ((uint32_t)ret_vals2[2] | (((uint32_t)ret_vals2[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals2[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals2[5]<<24)&0xFF000000));
		dxl_position[h] = (int32_t) ((uint32_t)ret_vals2[6] | (((uint32_t)ret_vals2[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals2[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals2[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}
	i = 10; // reset index
	uint8_t ret_vals3[data_len]; // could make this a 2D array?
	for(int j=0; j<idLengthPC; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals3[k] = dxl_bus_3.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_IDPC[j]-1;
		dxl_current[h] = (int16_t) ((uint16_t)ret_vals3[0] | (((uint16_t)ret_vals3[1]<<8)&0xFF00));
		dxl_velocity[h] = (int32_t) ((uint32_t)ret_vals3[2] | (((uint32_t)ret_vals3[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals3[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals3[5]<<24)&0xFF000000));
		dxl_position[h] = (int32_t) ((uint32_t)ret_vals3[6] | (((uint32_t)ret_vals3[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals3[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals3[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}
	i = 10; // reset index
	uint8_t ret_vals4[data_len]; // could make this a 2D array?
	for(int j=0; j<idLengthWR; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals4[k] = dxl_bus_4.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_IDWR[j]-1;
		dxl_current[h] = (int16_t) ((uint16_t)ret_vals4[0] | (((uint16_t)ret_vals4[1]<<8)&0xFF00));
		dxl_velocity[h] = (int32_t) ((uint32_t)ret_vals4[2] | (((uint32_t)ret_vals4[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals4[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals4[5]<<24)&0xFF000000));
		dxl_position[h] = (int32_t) ((uint32_t)ret_vals4[6] | (((uint32_t)ret_vals4[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals4[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals4[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}

}


//void SetCurrentCommands_DMA()
//{
//    uint8_t num_params1 = idLength*3; // 1 for ID + 2 for data length
//    uint8_t parameter1[num_params1];
//    uint8_t num_params2 = idLength2*3; // 1 for ID + 2 for data length
//	uint8_t parameter2[num_params2];
//	uint8_t num_params3 = idLengthPC*3; // 1 for ID + 2 for data length
//	uint8_t parameter3[num_params3];
//	uint8_t num_params4 = idLengthWR*3; // 1 for ID + 2 for data length
//	uint8_t parameter4[num_params4];
//
//    for (int i=0; i<idLength; i++){
//    	int j = i*3;
//		int k = dxl_ID[i]-1;
//		uint32_t cur = cur_command[k];
//		parameter1[j]    = (uint8_t) k;
//		parameter1[j+1] = SHIFT_TO_LSB(cur);
//		parameter1[j+2] = SHIFT_TO_MSB(cur);
//	}
//    for (int i=0; i<idLength2; i++){
//    	int j = i*3;
//		int k = dxl_ID2[i]-1;
//		uint32_t cur = cur_command[k];
//		parameter2[j]    = (uint8_t) k;
//		parameter2[j+1] = SHIFT_TO_LSB(cur);
//		parameter2[j+2] = SHIFT_TO_MSB(cur);
//	}
//    for (int i=0; i<idLengthPC; i++){
//		int j = i*3;
//		int k = dxl_IDPC[i]-1;
//		uint32_t cur = cur_command[k];
//		parameter3[j]    = (uint8_t) k;
//		parameter3[j+1] = SHIFT_TO_LSB(cur);
//		parameter3[j+2] = SHIFT_TO_MSB(cur);
//	}
//    for (int i=0; i<idLengthWR; i++){
//    	int j = i*3;
//		int k = dxl_IDWR[i]-1;
//		uint32_t cur = cur_command[k];
//		parameter4[j]   = (uint8_t) k;
//		parameter4[j+1] = SHIFT_TO_LSB(cur);
//		parameter4[j+2] = SHIFT_TO_MSB(cur);
//	}
//
//    dxl_bus_1.sWrite(2, FF_CUR_OFST, parameter1, num_params1);
//    dxl_bus_2.sWrite(2, FF_CUR_OFST, parameter2, num_params2);
//    dxl_bus_3.sWrite(2, FF_CUR_OFST, parameter3, num_params3);
//    dxl_bus_4.sWrite(2, FF_CUR_OFST, parameter4, num_params4);
//    tx_flag_1 = 0;
//	tx_flag_2 = 0;
//	tx_flag_3 = 0;
//	tx_flag_5 = 0;
//    dxl_bus_1.sendIPacket_DMA();
//    while(!tx_flag_1){;}
//	dxl_bus_2.sendIPacket_DMA();
//	while(!tx_flag_2){;}
//	dxl_bus_3.sendIPacket_DMA();
//	while(!tx_flag_3){;}
//	dxl_bus_4.sendIPacket_DMA();
//	while(!tx_flag_5){;}
//
//}


void SetFullControlCommands_DMA()
{
    uint8_t num_params1 = idLength*15; // 1 for ID + 14 for data length
    uint8_t parameter1[num_params1];
    uint8_t num_params2 = idLength2*15; // 1 for ID + 14 for data length
	uint8_t parameter2[num_params2];
	uint8_t num_params3 = idLengthPC*15; // 1 for ID + 14 for data length
	uint8_t parameter3[num_params3];
	uint8_t num_params4 = idLengthWR*15; // 1 for ID + 14 for data length
	uint8_t parameter4[num_params4];

    for (int i=0; i<idLength; i++){
    	int j = i*15;
		int k = dxl_ID[i]-1;
		uint32_t pos = pos_command[k];
		uint32_t vel = vel_command[k];
		uint32_t cur = cur_command[k];
		uint32_t kp = kp_command[k];
		uint32_t kd = kd_command[k];
		parameter1[j]    = dxl_ID[i];
		parameter1[j+1]  = SHIFT_TO_LSB(kp);
		parameter1[j+2]  = SHIFT_TO_MSB(kp);
		parameter1[j+3]  = SHIFT_TO_LSB(kd);
		parameter1[j+4]  = SHIFT_TO_MSB(kd);
		parameter1[j+5]  = (uint8_t) (pos&0x00000FF);
		parameter1[j+6]  = (uint8_t) ((pos&0x0000FF00)>>8);
		parameter1[j+7]  = (uint8_t) ((pos&0x00FF0000)>>16);
		parameter1[j+8]  = (uint8_t) (pos>>24);
		parameter1[j+9]  = (uint8_t) (vel&0x00000FF);
		parameter1[j+10] = (uint8_t) ((vel&0x0000FF00)>>8);
		parameter1[j+11] = (uint8_t) ((vel&0x00FF0000)>>16);
		parameter1[j+12] = (uint8_t) (vel>>24);
		parameter1[j+13] = SHIFT_TO_LSB(cur);
		parameter1[j+14] = SHIFT_TO_MSB(cur);
	}
    for (int i=0; i<idLength2; i++){
    	int j = i*15;
		int k = dxl_ID2[i]-1;
		uint32_t pos = pos_command[k];
		uint32_t vel = vel_command[k];
		uint32_t cur = cur_command[k];
		uint32_t kp = kp_command[k];
		uint32_t kd = kd_command[k];
		parameter2[j]    = dxl_ID2[i];
		parameter2[j+1]  = SHIFT_TO_LSB(kp);
		parameter2[j+2]  = SHIFT_TO_MSB(kp);
		parameter2[j+3]  = SHIFT_TO_LSB(kd);
		parameter2[j+4]  = SHIFT_TO_MSB(kd);
		parameter2[j+5]  = (uint8_t) (pos&0x00000FF);
		parameter2[j+6]  = (uint8_t) ((pos&0x0000FF00)>>8);
		parameter2[j+7]  = (uint8_t) ((pos&0x00FF0000)>>16);
		parameter2[j+8]  = (uint8_t) (pos>>24);
		parameter2[j+9]  = (uint8_t) (vel&0x00000FF);
		parameter2[j+10] = (uint8_t) ((vel&0x0000FF00)>>8);
		parameter2[j+11] = (uint8_t) ((vel&0x00FF0000)>>16);
		parameter2[j+12] = (uint8_t) (vel>>24);
		parameter2[j+13] = SHIFT_TO_LSB(cur);
		parameter2[j+14] = SHIFT_TO_MSB(cur);
	}
    for (int i=0; i<idLengthPC; i++){
		int j = i*15;
		int k = dxl_IDPC[i]-1;
		uint32_t pos = pos_command[k];
		uint32_t vel = vel_command[k];
		uint32_t cur = cur_command[k];
		uint32_t kp = kp_command[k];
		uint32_t kd = kd_command[k];
		parameter3[j]    = dxl_IDPC[i];
		parameter3[j+1]  = SHIFT_TO_LSB(kp);
		parameter3[j+2]  = SHIFT_TO_MSB(kp);
		parameter3[j+3]  = SHIFT_TO_LSB(kd);
		parameter3[j+4]  = SHIFT_TO_MSB(kd);
		parameter3[j+5]  = (uint8_t) (pos&0x00000FF);
		parameter3[j+6]  = (uint8_t) ((pos&0x0000FF00)>>8);
		parameter3[j+7]  = (uint8_t) ((pos&0x00FF0000)>>16);
		parameter3[j+8]  = (uint8_t) (pos>>24);
		parameter3[j+9]  = (uint8_t) (vel&0x00000FF);
		parameter3[j+10] = (uint8_t) ((vel&0x0000FF00)>>8);
		parameter3[j+11] = (uint8_t) ((vel&0x00FF0000)>>16);
		parameter3[j+12] = (uint8_t) (vel>>24);
		parameter3[j+13] = SHIFT_TO_LSB(cur);
		parameter3[j+14] = SHIFT_TO_MSB(cur);
	}
    for (int i=0; i<idLengthWR; i++){
    	int j = i*15;
		int k = dxl_IDWR[i]-1;
		uint32_t pos = pos_command[k];
		uint32_t vel = vel_command[k];
		uint32_t cur = cur_command[k];
		uint32_t kp = kp_command[k];
		uint32_t kd = kd_command[k];
		parameter4[j]    = dxl_IDWR[i];
		parameter4[j+1]  = SHIFT_TO_LSB(kp);
		parameter4[j+2]  = SHIFT_TO_MSB(kp);
		parameter4[j+3]  = SHIFT_TO_LSB(kd);
		parameter4[j+4]  = SHIFT_TO_MSB(kd);
		parameter4[j+5]  = (uint8_t) (pos&0x00000FF);
		parameter4[j+6]  = (uint8_t) ((pos&0x0000FF00)>>8);
		parameter4[j+7]  = (uint8_t) ((pos&0x00FF0000)>>16);
		parameter4[j+8]  = (uint8_t) (pos>>24);
		parameter4[j+9]  = (uint8_t) (vel&0x00000FF);
		parameter4[j+10] = (uint8_t) ((vel&0x0000FF00)>>8);
		parameter4[j+11] = (uint8_t) ((vel&0x00FF0000)>>16);
		parameter4[j+12] = (uint8_t) (vel>>24);
		parameter4[j+13] = SHIFT_TO_LSB(cur);
		parameter4[j+14] = SHIFT_TO_MSB(cur);
	}

    dxl_bus_1.sWrite(14, CTRL_WRITE_START, parameter1, num_params1);
    dxl_bus_2.sWrite(14, CTRL_WRITE_START, parameter2, num_params2);
    dxl_bus_3.sWrite(14, CTRL_WRITE_START, parameter3, num_params3);
    dxl_bus_4.sWrite(14, CTRL_WRITE_START, parameter4, num_params4);
    tx_flag_1 = 0;
	tx_flag_2 = 0;
	tx_flag_3 = 0;
	tx_flag_5 = 0;
    dxl_bus_1.sendIPacket_DMA();
//  while(!tx_flag_1){;}
	dxl_bus_2.sendIPacket_DMA();
//	while(!tx_flag_2){;}
	dxl_bus_3.sendIPacket_DMA();
//	while(!tx_flag_3){;}
	dxl_bus_4.sendIPacket_DMA();
//	while(!tx_flag_5){;}

}




void updateBusses(){
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
//	__HAL_TIM_SET_COUNTER(&htim5,0);
	// update busses vars
	double jointAngles1[4];
	double jointVelocities1[4];
	double jointAngles2[4];
	double jointVelocities2[4];
	float motorTorques[8];
	float jointTorques1[4];
	float jointTorques2[4];
	float velData[8];
	__HAL_TIM_SET_COUNTER(&htim1,0);
	GetBulkData_DMA();
	eval_time[0] = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,0);
	if (CURR_CONTROL){
		for(int i=0; i<8; i++){
			velData[i] = rpm_to_rads*(float)dxl_velocity[i];
			motorTorques[i] = Kt*0.001f*(float)dxl_current[i];
		}

		// printf("Transforms...");
		InverseActuatorTransformationL(jointAngles1, dxl_position[0], dxl_position[1], dxl_position[2], dxl_position[3]);
		InverseActuatorVelocityTransformationL(jointVelocities1, velData[0], velData[1], velData[2], velData[3]);
		InverseActuatorTransformationR(jointAngles2, dxl_position[4], dxl_position[5], dxl_position[6], dxl_position[7]);
		InverseActuatorVelocityTransformationR(jointVelocities2, velData[4], velData[5], velData[6], velData[7]);

		for(int i=0; i<4; i++){
			// convert from actuator torques to joint torques: tauJ = Jact^T * tauM
			jointTorques1[i] = JactL[0][i]*motorTorques[0] + JactL[1][i]*motorTorques[1] + JactL[2][i]*motorTorques[2] + JactL[3][i]*motorTorques[3];
			jointTorques2[i] = JactR[0][i]*motorTorques[4] + JactR[1][i]*motorTorques[5] + JactR[2][i]*motorTorques[6] + JactR[3][i]*motorTorques[7];
		}

		// printf("Current values...");
		for(int i=0; i<3; i++){
			currentPos[i] = (float)jointAngles1[i];
			currentVel[i] = (float)jointVelocities1[i];
			currentPos[3+i] = (float)jointAngles2[i];
			currentVel[3+i] = (float)jointVelocities2[i];
			currentCur[i] = 0.001f*(float)dxl_current[i];
			currentCur[3+i] = 0.001f*(float)dxl_current[4+i];
			currentJointTau[i] = (float)jointTorques1[i];
			currentJointTau[3+i] = (float)jointTorques2[i];
		}
		currentPos[6] = (float)jointAngles1[3];
		currentVel[6] = (float)jointVelocities1[3];
		currentPos[7] = (float)jointAngles2[3];
		currentVel[7] = (float)jointVelocities2[3];
		currentCur[6] = 0.001f*(float)dxl_current[3];
		currentCur[7] = 0.001f*(float)dxl_current[7];
		currentJointTau[6] = (float)jointTorques1[3];
		currentJointTau[7] = (float)jointTorques2[3];

		currentPos[8] = (float(dxl_position[8])-2048.0f)*((2*PI/4096.0f)); // (float(x)-2048) * ((2*PI)/4096.0f)
		currentVel[8] = rpm_to_rads*(float)dxl_velocity[8];
		currentCur[8] = 0.001f*(float)dxl_current[8];
		currentJointTau[8] = Kt_WR*0.001f*(float)dxl_current[8];

		// calculate desired joint torques here, torques are in Nm
		// printf("Desired joint torques...");
		// TODO: could initialize these values on startup to save some time?
		float pos_act[9] = {(float)jointAngles1[0], (float)jointAngles1[1], (float)jointAngles1[2], (float)jointAngles1[3], (float)jointAngles2[0], (float)jointAngles2[1], (float)jointAngles2[2], (float)jointAngles2[3], (float)currentPos[8]};
		float vel_act[9] = {(float)jointVelocities1[0], (float)jointVelocities1[1], (float)jointVelocities1[2], (float)jointVelocities1[3], (float)jointVelocities2[0], (float)jointVelocities2[1], (float)jointVelocities2[2], (float)jointVelocities2[3], (float)currentVel[8]};

//		float pos_des[9] = {dxl_pos_des[0], dxl_pos_des[1], dxl_pos_des[2], dxl_pos_des[3], dxl_pos_des[4], dxl_pos_des[5], dxl_pos_des[6],  dxl_pos_des[7], dxl_pos_des[8]};
//		float vel_des[9] = {dxl_vel_des[0], dxl_vel_des[1], dxl_vel_des[2],  dxl_vel_des[3], dxl_vel_des[4], dxl_vel_des[5], dxl_vel_des[6],  dxl_vel_des[7], dxl_vel_des[8]};
//
//		float KP_des[9] = {dxl_kp[0],dxl_kp[1],dxl_kp[2], dxl_kp[3], dxl_kp[4], dxl_kp[5], dxl_kp[6], dxl_kp[7], dxl_kp[8]}; // 0.8f*1.76f
//		float KD_des[9] = {dxl_kd[0],dxl_kd[1],dxl_kd[2], dxl_kd[3], dxl_kd[4], dxl_kd[5], dxl_kd[6],dxl_kd[7], dxl_kd[8]}; // 0.6f*0.026f
//		float tau_ff_des[9] = {dxl_tff_des[0],dxl_tff_des[1],dxl_tff_des[2],dxl_tff_des[3],dxl_tff_des[4],dxl_tff_des[5],dxl_tff_des[6],dxl_tff_des[7], dxl_tff_des[8] };

		float desired_joint_torques[9];
		float desired_actuator_torques[9];
		for(int i=0; i<9; i++)  {
//			 desired_joint_torques[i] = KP_des[i]*(pos_des[i]-pos_act[i]) + KD_des[i]*(vel_des[i]-vel_act[i]) + tau_ff_des[i];
			 desired_joint_torques[i] = dxl_kp[i]*(dxl_pos_des[i]-pos_act[i]) + dxl_kd[i]*(dxl_vel_des[i]-vel_act[i]) + dxl_tff_des[i];
	//         desired_joint_torques[i] = -0.1f*pos_act[i] - 0.01f*vel_act[i]; // basic impedance controller in joint space
		}

		// convert to desired actuator torques
		for(int i=0; i<4; i++){
			desired_actuator_torques[i] = JjointL[0][i]*desired_joint_torques[0] + JjointL[1][i]*desired_joint_torques[1] + JjointL[2][i]*desired_joint_torques[2] + JjointL[3][i]*desired_joint_torques[3];
			desired_actuator_torques[i+4] = JjointR[0][i]*desired_joint_torques[4] + JjointR[1][i]*desired_joint_torques[5] + JjointR[2][i]*desired_joint_torques[6] + JjointR[3][i]*desired_joint_torques[7];
		}
		desired_actuator_torques[8] = desired_joint_torques[8];

	//    for (int i=0; i<9; i++){
	//    	desired_actuator_torques[i] = desired_joint_torques[i];
	//    }

	//    for (int i=0; i<9; i++) {
	//    	desired_actuator_torques[i] = -0.5f*(float)dxl_position[i] - 0.0f*(float)dxl_velocity[i]; // basic impedance controller in actuator space, in mA (?)
	//    	Ktinv = 1;
	//    	Ktinv_WR = 1;
	//    }

		// convert to desired currents
		for(int i = 0; i<8; i++){
			desired_current[i] = fmaxf(fminf( Ktinv*desired_actuator_torques[i], current_limit ), -current_limit );
			current_command[i] = (int16_t)(1000.0f*desired_current[i]); // commanded current is in mA!
		}
		// different parameters for wrist roll motor
		desired_current[8] = fmaxf(fminf( Ktinv_WR*desired_actuator_torques[8], current_limit_WR), -current_limit_WR );
		current_command[8] = (int16_t)(1000.0f*desired_current[8]);


    // 1. obtain desired actuator positions, velocities, and torques from joint data
    // 2. convert gains, torques to correct units
		for (int i=0; i<8; i++){
			pos_command[i] = 0;
			vel_command[i] = 0;
			cur_command[i] = current_command[i]; // could just set this now, with some bonus damping (non-zero KD gain)
			kp_command[i] = 0; // abad was 800
			kd_command[i] = 100; //150; // abad was 15000 (!!)
		}
		kd_command[3] = 900;
		kd_command[7] = 900;

		kp_command[8] = 800;
		kd_command[8] = 500;
		pos_command[8] = 2048;

	}

    else {
		if (time_step % 4000==0) {
			if (freq1 < max_freq && updown) {
				freq1++;
			}
			else {
				updown = false;
				freq1--;
				if (freq1 < 2) {
				updown = true;
				}
			}
			freq2 = freq1 / 2.0f;
		}
	ActuatorTransformationL(rtPos_L, 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), 0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), (amp2)*(sinf(freq2*PI*(float)(dt*time_step))));
	ActuatorTransformationR(rtPos_R, -0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), -0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), -0.5*(amp1)*(-1*cosf(freq1*PI*(float)(dt*time_step))+1.0f), -(amp2)*(sinf(freq2*PI*(float)(dt*time_step))));
	time_step++;
	for(int k = 0; k<4; k++){
		trajPos[k] = rtPos_L[k];
		trajPos[k+4] = rtPos_R[k];
	}
		trajPos[8] = 2048;
    	for (int i=0; i<9; i++){
			pos_command[i] = trajPos[i];
			vel_command[i] = 0;
			cur_command[i] = 0;
			kp_command[i] = 800;
			kd_command[i] = 0;
		}
    }
	eval_time[1] = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,0);
    SetFullControlCommands_DMA();
    eval_time[2] = __HAL_TIM_GET_COUNTER(&htim1);
}


void sendCAN(){
	pack_reply48_joints(txMsg_fd_joints, currentPos, currentVel, currentJointTau);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader_fd_joints, txMsg_fd_joints);

	pack_reply48_sens(txMsg_fd_sens, force1, force2);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader_fd_sens, txMsg_fd_sens);
}




// main CPP loop
int dxl_main(void)
{
	printf("\r\n--------MIT Hand Control Board Firmware--------\r\n");
	printf("Version No: %.2f\r\n", VERSION_NUMBER);
	printf("FINGER SENSOR DEBUGGING VERSION\r\n\n\n");
	//Tx Headers
	txHeader_fd_joints.Identifier = 0x01;
	txHeader_fd_joints.IdType = FDCAN_STANDARD_ID;
	txHeader_fd_joints.TxFrameType = FDCAN_DATA_FRAME;
	txHeader_fd_joints.DataLength = FDCAN_DLC_BYTES_48;
	txHeader_fd_joints.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader_fd_joints.BitRateSwitch = FDCAN_BRS_ON;
	txHeader_fd_joints.FDFormat = FDCAN_FD_CAN;
	txHeader_fd_joints.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader_fd_joints.MessageMarker = 0;

	txHeader_fd_sens.Identifier = 0x02;
	txHeader_fd_sens.IdType = FDCAN_STANDARD_ID;
	txHeader_fd_sens.TxFrameType = FDCAN_DATA_FRAME;
	txHeader_fd_sens.DataLength = FDCAN_DLC_BYTES_48;
	txHeader_fd_sens.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader_fd_sens.BitRateSwitch = FDCAN_BRS_ON;
	txHeader_fd_sens.FDFormat = FDCAN_FD_CAN;
	txHeader_fd_sens.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader_fd_sens.MessageMarker = 0;

	//Rx Filters
	sys_can_filt.IdType = FDCAN_STANDARD_ID;
	sys_can_filt.FilterIndex = 0;
	sys_can_filt.FilterType = FDCAN_FILTER_MASK;
	sys_can_filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sys_can_filt.FilterID1 = 0x00;
	sys_can_filt.FilterID2 = 0x00;
	sys_can_filt.RxBufferIndex = 0;

	sense_can_filt.IdType = FDCAN_STANDARD_ID;
	sense_can_filt.FilterIndex = 0;
	sense_can_filt.FilterType = FDCAN_FILTER_RANGE;
	sense_can_filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sense_can_filt.FilterID1 = 0x05;
	sense_can_filt.FilterID2 = 0x0E; // up to 0x0E for phalange sensors
	sense_can_filt.RxBufferIndex = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sys_can_filt) != HAL_OK)
	{
		printf("Error in filter config. CAN FD1 \n\r");
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sense_can_filt) != HAL_OK)
	{
		printf("Error in filter config. CAN FD2 \n\r");
		Error_Handler();
	}

	if ((HAL_FDCAN_Start(&hfdcan1)) != HAL_OK ) //Initialize CAN Bus
	{
		printf("Failed to start system CAN.\n\r");
		while(1);
	}

	if ((HAL_FDCAN_Start(&hfdcan2)) != HAL_OK ) //Initialize CAN Bus
	{
		printf("Failed to start sensor CAN.\n\r");
		while(1);
	}

	HAL_Delay(100);

	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim1);

//	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);//

//	while (!MODE_SELECTED){
//		HAL_Delay(500);
//		printf("Waiting for Mode Select..\n\r");
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	}
//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
//	printf("Mode Selected..\n\r");
//	HAL_FDCAN_DeactivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
//
//	if (CURR_CONTROL) {
//		DXL_MODE = CURRENT_POS_CONTROL;
//		} else DXL_MODE = POSITION_CONTROL;
//
//	// Setup Routine for Dynamixels
//	printf("Setting up Dynamixel bus.\n\r");
//	Dynamixel_Startup_Routine();

//	if (CURR_CONTROL){
//		for (int i=0; i<idLength; i++) {
//			dxl_bus_1.SetVelocityProfile(dxl_ID[i], 414); // 414(94.81RPM) @ 14.8V, 330(75.57RPM) @ 12V
//			dxl_bus_1.SetAccelerationProfile(dxl_ID[i], 100); // 80(17166) rev/min^2
//			dxl_bus_1.SetPosPGain(dxl_ID[i], 800);
//			dxl_bus_1.SetPosDGain(dxl_ID[i], 0);
//			dxl_bus_1.SetGoalCurrent(dxl_ID[i], 1193);
//			HAL_Delay(100);
//		}
//		for (int i=0; i<idLength2; i++) {
//			dxl_bus_2.SetVelocityProfile(dxl_ID2[i], 414);
//			dxl_bus_2.SetAccelerationProfile(dxl_ID2[i], 100);
//			dxl_bus_2.SetPosPGain(dxl_ID2[i], 800);
//			dxl_bus_2.SetPosDGain(dxl_ID2[i], 0);
//			dxl_bus_2.SetGoalCurrent(dxl_ID[i], 1193);
//			HAL_Delay(100);
//		}
//		for (int i=0; i<idLengthPC; i++) {
//			dxl_bus_3.SetVelocityProfile(dxl_IDPC[i], 414);
//			dxl_bus_3.SetAccelerationProfile(dxl_IDPC[i], 100);
//			dxl_bus_3.SetPosPGain(dxl_IDPC[i], 800);
//			dxl_bus_3.SetPosDGain(dxl_IDPC[i], 0);
//			dxl_bus_3.SetGoalCurrent(dxl_ID[i], 1193);
//			HAL_Delay(100);
//		}
//		for (int i=0; i<idLengthWR; i++) {
//			dxl_bus_4.SetVelocityProfile(dxl_IDWR[i], 414);
//			dxl_bus_4.SetAccelerationProfile(dxl_IDWR[i], 100);
//			dxl_bus_4.SetPosPGain(dxl_IDWR[i], 800);
//			dxl_bus_4.SetPosDGain(dxl_IDWR[i], 0);
//			dxl_bus_4.SetGoalCurrent(dxl_ID[i], 2047);
//			HAL_Delay(100);
//		}
//		dxl_bus_1.SetMultGoalPositions(dxl_ID, idLength, multiGoalPos_1);
//		dxl_bus_2.SetMultGoalPositions(dxl_ID2, idLength2, multiGoalPos_2);
//		dxl_bus_3.SetMultGoalPositions(dxl_IDPC, idLengthPC, multiGoalPos_3);
//		dxl_bus_4.SetGoalPosition(9, 2048);
//		HAL_Delay(100);
//	}

	// enable CAN Interrupts
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);// Initialize CAN1 Rx0 Interrupt
	HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);// Initialize CAN2 Rx1 Interrupt

	// Enable Timer Interrupts
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	int loop_count = 0;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	while (1)
	{
		if(loop_count % 1000000 == 0){
//			printf("loop time: %lu \r\n",eval_time);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//			printf("%lu, f1: %lu, f2: %lu, t1: %lu, t2: %lu, tp: %lu\r\n",cf_ct, f1_ct, f2_ct, t1_ct, t2_ct, tp_ct);
		}
		loop_count++;
	}
}

// ISR functions

// ISR for timer 2 and timer 3
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance==TIM2){
//		printf("C\n\r");

//		updateBusses();

	} else if (htim->Instance==TIM3){
//		printf("S\n\r");
		sendCAN();

		// in this version...slowed down timer 3 interrupt and print sensor values
//		printf("Ph 1: %ld, %ld, %ld; Ph 2: %ld, %ld, %ld\n\r", phal1[0], phal1[1], phal1[2], phal2[0], phal2[1], phal2[2]);
//		printf("Ph 3: %ld, %ld, %ld; Ph 4: %ld, %ld, %ld\n\r\n\r", phal3[0], phal3[1], phal3[2], phal4[0], phal4[1], phal4[2]);

	} else if (htim->Instance==TIM4){

	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1) {
//		printf("Rx 1 done!\n\r");
		rx_flag_1 = 1;
	} else if(huart->Instance==USART2){
//		printf("Rx 2 done!\n\r");
		rx_flag_2 = 1;
	} else if(huart->Instance==UART7){
//		printf("Rx 3 done!\n\r");
		rx_flag_3 = 1;
	} else if(huart->Instance==UART5){
//		printf("Rx 5 done!\n\r");
		rx_flag_5 = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	printf("Global Tx Interrupt Entered\r\n");
	if(huart->Instance==USART1) {
		HAL_GPIO_WritePin(RTS1_GPIO_Port, RTS1_Pin, GPIO_PIN_RESET);
//		printf("Tx 1 done!\n\r");
		tx_flag_1 = 1;
	} else if(huart->Instance==USART2){
		HAL_GPIO_WritePin(RTS2_GPIO_Port, RTS2_Pin, GPIO_PIN_RESET);
//		printf("Tx 2 done!\n\r");
		tx_flag_2 = 1;
	} else if(huart->Instance==UART7){
		HAL_GPIO_WritePin(RTS7_GPIO_Port, RTS7_Pin, GPIO_PIN_RESET);
//		printf("Tx 3 done!\n\r");
		tx_flag_3 = 1;
	} else if(huart->Instance==UART5){
		HAL_GPIO_WritePin(RTS5_GPIO_Port, RTS5_Pin, GPIO_PIN_RESET);
//		printf("Tx 5 done!\n\r");
		tx_flag_5 = 1;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *canHandle, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
//		if (canHandle->Instance==FDCAN1){ // system CAN
			HAL_FDCAN_GetRxMessage(canHandle, FDCAN_RX_FIFO0, &rxMsg_sys, sys_rx_buf);
			uint32_t id = rxMsg_sys.Identifier;

			if(id==3){ // LEFT FINGER
				int p_int[4], v_int[4], kp_int[4], kd_int[4], t_int[4];
				for(int i=0;i<4;i++){
					p_int[i] = (sys_rx_buf[i*8+0]<<8)|sys_rx_buf[i*8+1];
					v_int[i] = (sys_rx_buf[i*8+2]<<4)|(sys_rx_buf[i*8+3]>>4);
					kp_int[i] = ((sys_rx_buf[i*8+3]&0xF)<<8)|sys_rx_buf[i*8+4];
					kd_int[i] = (sys_rx_buf[i*8+5]<<4)|(sys_rx_buf[i*8+6]>>4);
					t_int[i] = ((sys_rx_buf[i*8+6]&0xF)<<8)|sys_rx_buf[i*8+7];
				}

				for(int j=0;j<4;j++){
					dxl_pos_des[j] = uint_to_float(p_int[j], P_MIN, P_MAX, 16);
					dxl_vel_des[j] = uint_to_float(v_int[j], V_MIN, V_MAX, 12);
					dxl_kp[j] = uint_to_float(kp_int[j], KP_MIN, KP_MAX, 12)/KP_SCALE;
					dxl_kd[j] = uint_to_float(kd_int[j], KD_MIN, KD_MAX, 12)/KD_SCALE;
					dxl_tff_des[j] = uint_to_float(t_int[j], T_MIN, T_MAX, 12)/T_SCALE;
				}
			}

			else if(id==4){ // RIGHT FINGER
				int p_int[5], v_int[5], kp_int[5], kd_int[5], t_int[5];
				for(int i=0;i<5;i++){
					p_int[i] = (sys_rx_buf[i*8+0]<<8)|sys_rx_buf[i*8+1];
					v_int[i] = (sys_rx_buf[i*8+2]<<4)|(sys_rx_buf[i*8+3]>>4);
					kp_int[i] = ((sys_rx_buf[i*8+3]&0xF)<<8)|sys_rx_buf[i*8+4];
					kd_int[i] = (sys_rx_buf[i*8+5]<<4)|(sys_rx_buf[i*8+6]>>4);
					t_int[i] = ((sys_rx_buf[i*8+6]&0xF)<<8)|sys_rx_buf[i*8+7];
				}

				for(int j=0;j<5;j++){
					dxl_pos_des[j+4] = uint_to_float(p_int[j], P_MIN, P_MAX, 16);
					dxl_vel_des[j+4] = uint_to_float(v_int[j], V_MIN, V_MAX, 12);
					dxl_kp[j+4] = uint_to_float(kp_int[j], KP_MIN, KP_MAX, 12)/KP_SCALE;
					dxl_kd[j+4] = uint_to_float(kd_int[j], KD_MIN, KD_MAX, 12)/KD_SCALE;
					dxl_tff_des[j+4] = uint_to_float(t_int[j], T_MIN, T_MAX, 12)/T_SCALE;
				}
			}

			else if(id==80){ // MODE SELECT MSG

				if(sys_rx_buf[7] == 0xFC){
					CURR_CONTROL = true;
					MODE_SELECTED = true;
				}
				else if (sys_rx_buf[7] == 0xFD){
					CURR_CONTROL = false;
					MODE_SELECTED = true;
				}
			}
	}

}

// new unpacking for neural net values from fingertip sensors and extra ToF
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *canHandle, uint32_t RxFifo1ITs)
{
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET){
//		cf_ct++;
//		if (canHandle->Instance==FDCAN2){ // sensor CAN
			HAL_FDCAN_GetRxMessage(canHandle, FDCAN_RX_FIFO1, &rxMsg_sense, sense_rx_buf);
	//		unpack_sensor(sense_rx_buf, rxMsg_sense.StdId);

			uint8_t id = rxMsg_sense.Identifier;
//			printf("%d\n\r", id);

			if (id == CAN2_FORCE_1){
				// unpack forces and angles
//				cycle_count = __HAL_TIM_GET_COUNTER(&htim1);
//				printf("Cycle: %lu\r\n",cycle_count);
//				__HAL_TIM_SET_COUNTER(&htim1, 0);
//				f1_ct++;
				uint16_t fx_int = ((sense_rx_buf[0]&0x0F)<<8)|sense_rx_buf[1];
				uint16_t fy_int = (sense_rx_buf[2]<<4)|(sense_rx_buf[3]>>4);
				uint16_t fz_int = ((sense_rx_buf[3]&0x0F)<<8)|sense_rx_buf[4];
				uint16_t theta_int = (sense_rx_buf[5]<<4)|(sense_rx_buf[6]>>4);
				uint16_t phi_int = ((sense_rx_buf[6]&0x0F)<<8)|sense_rx_buf[7];
				/// convert uints to floats ///
				force1[0] = uint_to_float(fx_int, FT_MIN, FT_MAX, 12);
				force1[1]  = uint_to_float(fy_int, FT_MIN, FT_MAX, 12);
				force1[2]  = uint_to_float(fz_int, FN_MIN, FN_MAX, 12);
				force1[3]  = uint_to_float(theta_int, ANG_MIN, ANG_MAX, 12);
				force1[4]  = uint_to_float(phi_int, ANG_MIN, ANG_MAX, 12);
			}
			else if (id == CAN2_FORCE_2){
//				f2_ct++;
				// unpack forces and angles
				uint16_t fx_int = ((sense_rx_buf[0]&0x0F)<<8)|sense_rx_buf[1];
				uint16_t fy_int = (sense_rx_buf[2]<<4)|(sense_rx_buf[3]>>4);
				uint16_t fz_int = ((sense_rx_buf[3]&0x0F)<<8)|sense_rx_buf[4];
				uint16_t theta_int = (sense_rx_buf[5]<<4)|(sense_rx_buf[6]>>4);
				uint16_t phi_int = ((sense_rx_buf[6]&0x0F)<<8)|sense_rx_buf[7];
				/// convert uints to floats ///
				force2[0] = uint_to_float(fx_int, FT_MIN, FT_MAX, 12);
				force2[1]  = uint_to_float(fy_int, FT_MIN, FT_MAX, 12);
				force2[2]  = uint_to_float(fz_int, FN_MIN, FN_MAX, 12);
				force2[3]  = uint_to_float(theta_int, ANG_MIN, ANG_MAX, 12);
				force2[4]  = uint_to_float(phi_int, ANG_MIN, ANG_MAX, 12);
			}
			else if (id == CAN2_TOF_1){
//				t1_ct++;
				for(int i = 0;i<8;i++){
					tof1[i] = sense_rx_buf[i];
				}
			}
			else if (id == CAN2_TOF_2){
//				t2_ct++;
				for(int i = 0;i<8;i++){
					tof2[i] = sense_rx_buf[i];
				}
			}
			else if (id == CAN2_TOF_PALM){ // TODO: change this!!!
//				tp_ct++;
				palmTOF = sense_rx_buf[0];
			}

			// phalange IDs (TOF and FSR)
			else if (id == CAN2_PHAL_1){

				phal1[0] = sense_rx_buf[0]; // TOF
				phal1[1] = (sense_rx_buf[1]<<8)|sense_rx_buf[2]; // FSR 1
				phal1[2] = (sense_rx_buf[3]<<8)|sense_rx_buf[4]; // FSR 2
			}
			else if (id == CAN2_PHAL_2){
				phal2[0] = sense_rx_buf[0];
				phal2[1] = (sense_rx_buf[1]<<8)|sense_rx_buf[2];
				phal2[2] = (sense_rx_buf[3]<<8)|sense_rx_buf[4];
			}
			else if (id == CAN2_PHAL_3){
				phal3[0] = sense_rx_buf[0];
				phal3[1] = (sense_rx_buf[1]<<8)|sense_rx_buf[2];
				phal3[2] = (sense_rx_buf[3]<<8)|sense_rx_buf[4];
			}
			else if (id == CAN2_PHAL_4){
				phal4[0] = sense_rx_buf[0];
				phal4[1] = (sense_rx_buf[1]<<8)|sense_rx_buf[2];
				phal4[2] = (sense_rx_buf[3]<<8)|sense_rx_buf[4];
			}
			// pressure sensor message ids
			// TODO: fix this!
//			else if (id == CAN2_RAW_BMP_1){
//				for(int i = 0;i<8;i++){
//					pressure_raw1[i] = sense_rx_buf[i];
//				}
//			}



		}
//	}
}
