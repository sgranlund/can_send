

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


void startup_blink(){
	int i = 0;
	while(i<20){
	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
	  HAL_Delay(30); //delay 30ms
	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
	  HAL_Delay(30); //delay 30ms
	  i++;
	}
}
void recieve_CAN(uint8_t *RxData){
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
}
void send_CAN(uint32_t Identifier,uint8_t *TxData,uint32_t TxMailbox){

	TxHeader.DLC = 8; //Data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = Identifier; //Sender ID

	uint32_t CAN_ERROR = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	if (CAN_ERROR == HAL_OK){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
		HAL_Delay(100); //delay 100ms
	}
}

void send_UART(){
	uint8_t Test[] = "11\r\n";//"Hello World !!!\r\n"; //Data to send
	uint32_t Error = HAL_UART_Transmit(&huart7,Test,sizeof(Test),12);// Sending in normal mode
	if (Error != HAL_OK){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);

	}
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
	HAL_Delay(100);

}


void state_machine(){
	enum states {
	    STARTUP,
	    PATHFOLLOWER,
	    NEARFIELD,
		DOCKED,
		EMERGENCYSTOP,
	} state;
	state = STARTUP;
	switch(state) {
		case STARTUP:
			startup_blink();
		break;
		case PATHFOLLOWER:

		break;
		case NEARFIELD:


		break;
		case DOCKED:

		break;

		case EMERGENCYSTOP:

		break;
	}

}
