#include "fdcan.h"
#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "math.h"

#include "math_ops.h"
#include "crc.h"
#include <XM430_bus.h>
#include <dxl_control.h>
#include <dxl_startup.h>
#include "dxl_transforms.h"
#include <pack_can.h>

// defines
#define VERSION_NUMBER 	1.21f
#define MOTOR_KT 		3.7f/2.7f 	// TODO: replace this!
#define MOTOR_CUR_LIM 	2.7f 		// TODO: replace this!

#define cur_count2amp(x) (x*(2.69f/1000.0f))
#define cur_amp2count(x) (int16_t)(x*(1000.0f/2.69f))

// System control stuff
volatile bool CURR_CONTROL = true; // false is position control mode
volatile bool MODE_SELECTED = false;
volatile bool SENSOR_DEBUG = false;
volatile bool HAND_RESET = false;
uint8_t DXL_MODE;
uint32_t eval_time[3] = {0, 0, 0}; // for timing and debugging

// Initialize CAN FD stuff
FDCAN_RxHeaderTypeDef rxMsg_joints;
FDCAN_TxHeaderTypeDef txHeader_joints;
FDCAN_FilterTypeDef canFilt_joints;
uint8_t txMsg_joints[48];
uint8_t rxBuf_joints[48];

// Initialize dynamixel stuff
XM430_bus dxl_bus_1(&huart1, RTS1_GPIO_Port, RTS1_Pin); // left MCP, PIP, DIP
XM430_bus dxl_bus_2(&huart2, RTS2_GPIO_Port, RTS2_Pin); // right MCP, PIP, DIP
XM430_bus dxl_bus_3(&huart7, RTS7_GPIO_Port, RTS7_Pin); // left MCR, right MCR

uint8_t dxl_IDs[] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t dxl_ID1[] =  {1, 2, 3};
uint8_t dxl_ID2[] = {5, 6, 7};
uint8_t dxl_ID3[] = {4, 8};
uint8_t idLength1 = 3;
uint8_t idLength2 = 3;
uint8_t idLength3 = 2;
volatile uint8_t tx_flag_1;
volatile uint8_t tx_flag_2;
volatile uint8_t tx_flag_3;
volatile uint8_t rx_flag_1;
volatile uint8_t rx_flag_2;
volatile uint8_t rx_flag_3;

// joint-space states
float joint_pos[8];
float joint_vel[8];
float joint_tau[8];

// joint-space commands
float joint_tau_des[8]; 
float joint_pos_des[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float joint_vel_des[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float joint_tau_ff[8]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float joint_kp[8]      = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
float joint_kd[8]      = {0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f, 0.02f};

// motor-space states
int32_t motor_pos[8];
int32_t motor_vel[8];
int16_t motor_cur[8];
float motor_cur_A[8];
float motor_tau[8];

// motor-space commands
float motor_tau_des[8];
int32_t motor_pos_des[8];
int32_t motor_vel_des[8];
int16_t motor_cur_des[8];
float motor_cur_des_A[8];
uint32_t motor_kp[8];
uint32_t motor_kd[8];

// get all of the motor data
void GetBulkData_DMA()
{
	// set up transmissions
	uint8_t data_len = 10;
	uint8_t start_addr = PRESENT_CURRENT;
	dxl_bus_1.sRead(data_len, start_addr, dxl_ID1, idLength1);
	dxl_bus_1.rPacketLength = 8 + idLength1*(4+data_len);
	dxl_bus_2.sRead(data_len, start_addr, dxl_ID2, idLength2);
	dxl_bus_2.rPacketLength = 8 + idLength2*(4+data_len);
	dxl_bus_3.sRead(data_len, start_addr, dxl_ID3, idLength3);
	dxl_bus_3.rPacketLength = 8 + idLength3*(4+data_len);

	// DMA transmissions
	tx_flag_1 = 0;
	rx_flag_1 = 0;
	tx_flag_2 = 0;
	rx_flag_2 = 0;
	tx_flag_3 = 0;
	rx_flag_3 = 0;
	dxl_bus_1.sendIPacket_DMA();
	dxl_bus_2.sendIPacket_DMA();
	dxl_bus_3.sendIPacket_DMA();
	while((!tx_flag_1)||(!tx_flag_2)||(!tx_flag_3)){;}
	dxl_bus_1.getRPacket_DMA();
	dxl_bus_2.getRPacket_DMA();
	dxl_bus_3.getRPacket_DMA();
	while((!rx_flag_1)||(!rx_flag_2)||(!rx_flag_3)){;}

	// for loop to extract ret_vals from rPackets
	int i = 10;
	uint8_t ret_vals1[data_len];
	for(int j=0; j<idLength1; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals1[k] = dxl_bus_1.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_ID1[j]-1;
		motor_cur[h] = (int16_t) ((uint16_t)ret_vals1[0] | (((uint16_t)ret_vals1[1]<<8)&0xFF00));
		motor_vel[h] = (int32_t) ((uint32_t)ret_vals1[2] | (((uint32_t)ret_vals1[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals1[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals1[5]<<24)&0xFF000000));
		motor_pos[h] = (int32_t) ((uint32_t)ret_vals1[6] | (((uint32_t)ret_vals1[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals1[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals1[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}
	i = 10; // reset index
	uint8_t ret_vals2[data_len];
	for(int j=0; j<idLength2; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals2[k] = dxl_bus_2.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_ID2[j]-1;
		motor_cur[h] = (int16_t) ((uint16_t)ret_vals2[0] | (((uint16_t)ret_vals2[1]<<8)&0xFF00));
		motor_vel[h] = (int32_t) ((uint32_t)ret_vals2[2] | (((uint32_t)ret_vals2[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals2[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals2[5]<<24)&0xFF000000));
		motor_pos[h] = (int32_t) ((uint32_t)ret_vals2[6] | (((uint32_t)ret_vals2[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals2[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals2[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}
	i = 10; // reset index
	uint8_t ret_vals3[data_len];
	for(int j=0; j<idLength3; j++){ // for each ID
		// pull data out from rPacket
		for (int k=0; k<data_len; k++){
			ret_vals3[k] = dxl_bus_3.rPacket[i];
			i++;
		}
		// pack dxl variables
		int h = dxl_ID3[j]-1;
		motor_cur[h] = (int16_t) ((uint16_t)ret_vals3[0] | (((uint16_t)ret_vals3[1]<<8)&0xFF00));
		motor_vel[h] = (int32_t) ((uint32_t)ret_vals3[2] | (((uint32_t)ret_vals3[3]<<8)&0x0000FF00) | (((uint32_t)ret_vals3[4]<<16)&0x00FF0000) | (((uint32_t)ret_vals3[5]<<24)&0xFF000000));
		motor_pos[h] = (int32_t) ((uint32_t)ret_vals3[6] | (((uint32_t)ret_vals3[7]<<8)&0x0000FF00) | (((uint32_t)ret_vals3[8]<<16)&0x00FF0000) | (((uint32_t)ret_vals3[9]<<24)&0xFF000000));
		i+=4; // increment for next ID in rPacket
	}

}

// send all of the motor commands
void SetFullControlCommands_DMA()
{
    uint8_t num_params1 = idLength1*15; // 1 for ID + 14 for data length
    uint8_t parameter1[num_params1];
    uint8_t num_params2 = idLength2*15; // 1 for ID + 14 for data length
	uint8_t parameter2[num_params2];
	uint8_t num_params3 = idLength3*15; // 1 for ID + 14 for data length
	uint8_t parameter3[num_params3];

    for (int i=0; i<idLength1; i++){
    	int j = i*15;
		int k = dxl_ID1[i]-1;
		uint32_t pos = motor_pos_des[k];
		uint32_t vel = motor_vel_des[k];
		uint32_t cur = motor_cur_des[k];
		uint32_t kp = motor_kp[k];
		uint32_t kd = motor_kd[k];
		parameter1[j]    = dxl_ID1[i];
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
		uint32_t pos = motor_pos_des[k];
		uint32_t vel = motor_vel_des[k];
		uint32_t cur = motor_cur_des[k];
		uint32_t kp = motor_kp[k];
		uint32_t kd = motor_kd[k];
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
    for (int i=0; i<idLength3; i++){
		int j = i*15;
		int k = dxl_ID3[i]-1;
		uint32_t pos = motor_pos_des[k];
		uint32_t vel = motor_vel_des[k];
		uint32_t cur = motor_cur_des[k];
		uint32_t kp = motor_kp[k];
		uint32_t kd = motor_kd[k];
		parameter3[j]    = dxl_ID3[i];
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

    dxl_bus_1.sWrite(14, CTRL_WRITE_START, parameter1, num_params1);
    dxl_bus_2.sWrite(14, CTRL_WRITE_START, parameter2, num_params2);
    dxl_bus_3.sWrite(14, CTRL_WRITE_START, parameter3, num_params3);

    tx_flag_1 = 0;
	tx_flag_2 = 0;
	tx_flag_3 = 0;

    dxl_bus_1.sendIPacket_DMA();
	dxl_bus_2.sendIPacket_DMA();
	dxl_bus_3.sendIPacket_DMA();

}

// run motor control laws
void updateBusses(){

	// start by getting new data from the motors
	__HAL_TIM_SET_COUNTER(&htim1,0);
	GetBulkData_DMA();
	eval_time[0] = __HAL_TIM_GET_COUNTER(&htim1);
	__HAL_TIM_SET_COUNTER(&htim1,0);

	// transform motor positions to joint positions
	MotorPos2JointPos(motor_pos, joint_pos);

	// transform motor velocities to joint velocities 
	MotorVel2JointVel(motor_vel, joint_vel);
	
	// transform motor currents to joint torques
	for(int i=0; i<8; i++){
		motor_cur_A[i] = cur_count2amp(motor_cur[i]);
		motor_tau[i] = MOTOR_KT*motor_cur_A[i];
	}
	MotorTau2JointTau(motor_tau, joint_tau);

	// run control (depending on control mode)
	if (!SENSOR_DEBUG){
		if (CURR_CONTROL){
			// PD control plus feedforward torque in joint-space
			for(int i=0; i<8; i++)  {
				 joint_tau_des[i] = joint_kp[i]*(joint_pos_des[i]-joint_pos[i]) 
				 						+ joint_kd[i]*(joint_vel_des[i]-joint_vel[i]) 
											+ joint_tau_ff[i];
			}
			// convert to desired torques in motor-space
			JointTau2MotorTau(joint_tau_des, motor_tau_des);
			// convert to desired motor currents
			for(int i = 0; i<8; i++){
				motor_cur_des_A[i] = fmaxf(fminf(motor_tau_des[i]/MOTOR_KT, MOTOR_CUR_LIM), -MOTOR_CUR_LIM);
				motor_cur_des[i] = cur_amp2count(motor_cur_des_A[i]);
			}
			// set the rest of the commands to send to motors
			for (int i=0; i<8; i++){
				motor_pos_des[i] = 0;
				motor_vel_des[i] = 0;
				motor_kp[i] = 0; 
				motor_kd[i] = 100;
			}
			motor_kd[3] = 600; // higher damping gains for abad motors
			motor_kd[7] = 600;
		}
		else { // POSITION CONTROL
			// transform desired joint positions to desired actuator positions
			JointPos2MotorPos(joint_pos_des, motor_pos_des);
			// set the rest of the commands to send to motors
			for (int i=0; i<8; i++){
				motor_vel_des[i] = 0;
				motor_cur_des[i] = 0;
				motor_kp[i] = 800;
				motor_kd[i] = 0;
			}
		}
		eval_time[1] = __HAL_TIM_GET_COUNTER(&htim1);

		// send commands
		__HAL_TIM_SET_COUNTER(&htim1,0);
		SetFullControlCommands_DMA();
		eval_time[2] = __HAL_TIM_GET_COUNTER(&htim1);
	}
}

// compile and send CAN message with joint data
void sendCAN(){
	pack_reply48_joints(txMsg_joints, joint_pos, joint_vel, joint_tau);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader_joints, txMsg_joints);
}

// main CPP loop
int dxl_main(void)
{
	printf("\r\n--------MIT Hand Control Board Firmware--------\r\n");
	printf("Version No: %.2f\r\n\n\n", VERSION_NUMBER);

	// Tx Headers
	txHeader_joints.Identifier = 0x01;
	txHeader_joints.IdType = FDCAN_STANDARD_ID;
	txHeader_joints.TxFrameType = FDCAN_DATA_FRAME;
	txHeader_joints.DataLength = FDCAN_DLC_BYTES_48;
	txHeader_joints.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader_joints.BitRateSwitch = FDCAN_BRS_ON;
	txHeader_joints.FDFormat = FDCAN_FD_CAN;
	txHeader_joints.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader_joints.MessageMarker = 0;

	// Rx Filters
	canFilt_joints.IdType = FDCAN_STANDARD_ID;
	canFilt_joints.FilterIndex = 0;
	canFilt_joints.FilterType = FDCAN_FILTER_MASK;
	canFilt_joints.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	canFilt_joints.FilterID1 = 0x00;
	canFilt_joints.FilterID2 = 0x00;
	canFilt_joints.RxBufferIndex = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilt_joints) != HAL_OK)
	{
		printf("Error in filter config. CAN FD1 \n\r");
		Error_Handler();
	}


	if ((HAL_FDCAN_Start(&hfdcan1)) != HAL_OK ) // Initialize CAN Bus
	{
		printf("Failed to start CAN.\n\r");
		while(1);
	}

	HAL_Delay(100);

	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_Base_Start(&htim1);

	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

	while (!MODE_SELECTED){
		HAL_Delay(500);
		printf("Waiting for Mode Select..\n\r");
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}
	HAND_RESET = false; // this flag doesn't need to be set this time
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	printf("Mode Selected..\n\r");
	HAL_FDCAN_DeactivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

	if (CURR_CONTROL) {
		DXL_MODE = CURRENT_POS_CONTROL;
	} else {
		DXL_MODE = POSITION_CONTROL;
	} 

	// Setup Routine for Dynamixels
	printf("Setting up Dynamixel bus.\n\r");
	Dynamixel_Startup_Routine(SENSOR_DEBUG);
	if (SENSOR_DEBUG) {
		printf("Starting in sensor debug mode.\n\r");
	}

	// enable CAN Interrupts
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0); // Initialize CAN1 Rx0 Interrupt

	// enable Timer Interrupts
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	int loop_count = 0;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	while (1)
	{
		// just blink the LED in sensor debug mode
		if (SENSOR_DEBUG) {
			if(loop_count % 1000000 == 0){
//				printf("Loop time: %u \r\n",eval_time);
				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			}
			loop_count++;
		}
		// check for gripper reset flag
		if (HAND_RESET){
			// deactivate all of the interrupts
			// disable CAN Interrupts
			HAL_FDCAN_DeactivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
			// disable Timer Interrupts
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);

			// turn off some LEDs
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

			// re-run the startup routine
			printf("Received new mode. Re-starting motors.\n\r");
			if (MODE_SELECTED) {
				if (CURR_CONTROL) {
					DXL_MODE = CURRENT_POS_CONTROL;
					} else DXL_MODE = POSITION_CONTROL;
				// start by disabling all of the motors
				Dynamixel_Shutdown_Routine();
				Dynamixel_Startup_Routine(SENSOR_DEBUG); // in debug mode, the dynamixels will be disabled
			} else {
				// disable message was received
				Dynamixel_Shutdown_Routine();
			}

			// reset CAN commands
			for (int i=0; i<8; i++){
				joint_pos_des[i] = 0.0f;
				joint_vel_des[i] = 0.0f;
				joint_tau_ff[i] = 0.0f;
				joint_kp[i] = 0.5f;
				joint_kd[i] = 0.02f;
			}

			// turn on LEDs
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			// reset flag
			HAND_RESET = false;

			// and reactivate the interrupts
			// enable CAN Interrupts
			HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
			// enable Timer Interrupts
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim3);
		}

	}

}

// ISR functions
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance==TIM2){
//		printf("C\n\r");
		updateBusses();
	} else if (htim->Instance==TIM3){
//		printf("S\n\r");
		sendCAN();
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
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *canHandle, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
		HAL_FDCAN_GetRxMessage(canHandle, FDCAN_RX_FIFO0, &rxMsg_joints, rxBuf_joints);
		uint32_t id = rxMsg_joints.Identifier;
		// Left finger
		if(id==3){ 
			int p_int[4], v_int[4], kp_int[4], kd_int[4], t_int[4];
			for(int i=0;i<4;i++){
				p_int[i] = (rxBuf_joints[i*8+0]<<8)|rxBuf_joints[i*8+1];
				v_int[i] = (rxBuf_joints[i*8+2]<<4)|(rxBuf_joints[i*8+3]>>4);
				kp_int[i] = ((rxBuf_joints[i*8+3]&0xF)<<8)|rxBuf_joints[i*8+4];
				kd_int[i] = (rxBuf_joints[i*8+5]<<4)|(rxBuf_joints[i*8+6]>>4);
				t_int[i] = ((rxBuf_joints[i*8+6]&0xF)<<8)|rxBuf_joints[i*8+7];
			}
			for(int j=0;j<4;j++){
				joint_pos_des[j] = uint_to_float(p_int[j], P_MIN, P_MAX, 16);
				joint_vel_des[j] = uint_to_float(v_int[j], V_MIN, V_MAX, 12);
				joint_kp[j] = uint_to_float(kp_int[j], KP_MIN, KP_MAX, 12)/KP_SCALE;
				joint_kd[j] = uint_to_float(kd_int[j], KD_MIN, KD_MAX, 12)/KD_SCALE;
				joint_tau_ff[j] = uint_to_float(t_int[j], T_MIN, T_MAX, 12)/T_SCALE;
			}
		}
		// Right finger
		else if(id==4){
			int p_int[4], v_int[4], kp_int[4], kd_int[4], t_int[4];
			for(int i=0;i<4;i++){
				p_int[i] = (rxBuf_joints[i*8+0]<<8)|rxBuf_joints[i*8+1];
				v_int[i] = (rxBuf_joints[i*8+2]<<4)|(rxBuf_joints[i*8+3]>>4);
				kp_int[i] = ((rxBuf_joints[i*8+3]&0xF)<<8)|rxBuf_joints[i*8+4];
				kd_int[i] = (rxBuf_joints[i*8+5]<<4)|(rxBuf_joints[i*8+6]>>4);
				t_int[i] = ((rxBuf_joints[i*8+6]&0xF)<<8)|rxBuf_joints[i*8+7];
			}
			for(int j=0;j<4;j++){
				joint_pos_des[j+4] = uint_to_float(p_int[j], P_MIN, P_MAX, 16);
				joint_vel_des[j+4] = uint_to_float(v_int[j], V_MIN, V_MAX, 12);
				joint_kp[j+4] = uint_to_float(kp_int[j], KP_MIN, KP_MAX, 12)/KP_SCALE;
				joint_kd[j+4] = uint_to_float(kd_int[j], KD_MIN, KD_MAX, 12)/KD_SCALE;
				joint_tau_ff[j+4] = uint_to_float(t_int[j], T_MIN, T_MAX, 12)/T_SCALE;
			}
		}
		// Mode select message
		else if(id==80){
			if(rxBuf_joints[7] == 0xFC){
				CURR_CONTROL = true;
				MODE_SELECTED = true;
				SENSOR_DEBUG = false;
				HAND_RESET = true;
			}
			else if (rxBuf_joints[7] == 0xFD){
				CURR_CONTROL = false;
				MODE_SELECTED = true;
				SENSOR_DEBUG = false;
				HAND_RESET = true;
			}
			else if (rxBuf_joints[7] == 0xFB){
				CURR_CONTROL = false;
				MODE_SELECTED = true;
				SENSOR_DEBUG = true;
				HAND_RESET = true;
			}
			else if (rxBuf_joints[7] == 0xFA){
				CURR_CONTROL = false;
				MODE_SELECTED = false;
				SENSOR_DEBUG = false;
				HAND_RESET = true;
			}
		}
	}
}
