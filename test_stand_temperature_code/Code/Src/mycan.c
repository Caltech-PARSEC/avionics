/*
 * mycan.c
 *
 *  Created on: Jun 10, 2018
 *      Author: Malia
 */

/*
 * mycan.c
 *
 *  Created on: Apr 8, 2017
 *      Author: Tyler
 */

#include "stm32f4xx_hal.h"

#include "mycan.h"

void Init_MyCAN()
{
	CAN1->MCR &= ~1;
  	CAN1->sFilterRegister[0].FR1 = 0;
 	CAN1->sFilterRegister[0].FR2 = 0;
 	CAN1->FA1R = 1;
 	CAN1->FS1R = 1;
 	CAN1->FMR &= ~1;
}

uint16_t create_ID(uint16_t board, uint16_t type)
{
	return (((uint16_t)(board << 6) & 0b0000011111000000) | ((uint16_t)type & 0b00111111));
}

/*void create_ACK(uint16_t id, uint16_t board, can_msg_t *msg) {
    uint16_t identifier = create_ID(board, MID_ACKNOWLEDGE);
    uint16_t payload = id;
    CAN_short_msg(msg, identifier, payload);
}*/

bool CAN_can_transmit() {
	// Retrieve mailbox empty bits from CAN transmit register
	int empty_flags = (CAN1->TSR >> 26) & 0x07;

	// Check that at least one mailbox is empty
	return (empty_flags != 0x00);
}

void CAN_byte_msg(can_msg_t *msg, uint16_t identifier, uint8_t data) {
	msg->identifier = identifier;
	msg->data_length = 1;
	*(uint8_t*)(msg->data + 7) = data;
}
void CAN_short_msg(can_msg_t *msg, uint16_t identifier, uint16_t data) {
	msg->identifier = identifier;
	msg->data_length = 2;
	*(uint16_t*)(msg->data + 6) = data;
}
void CAN_long_msg(can_msg_t *msg, uint16_t identifier, uint32_t data) {
	msg->identifier = identifier;
	msg->data_length = 4;
	*(uint32_t*)(msg->data + 4) = data;
}

uint8_t CAN_decode_byte(can_msg_t *msg) {
	return msg->data[4];
}
uint16_t CAN_decode_short(can_msg_t *msg) {
	return (msg->data[4] << 8) | msg->data[5];
}
uint32_t CAN_decode_long(can_msg_t *msg) {
 return (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | msg->data[7];
}

bool CAN_queue_transmit(can_msg_t *msg) {
	// Check if at least one mailbox is available
	if (CAN_can_transmit()) {

		// Find an available mailbox
		int mailbox = (CAN1->TSR >> 24) & 0x03;

		int identifier_reg = 0;
		int dat_len_reg = 0;
		int dat_low_reg = 0;
		int dat_high_reg = 0;

		// Setup transmit register for data frame with standard identifier
		identifier_reg |= (msg->identifier << 21);
		// Request start of transmission
		identifier_reg |= 0x01;

		// Setup data length register with no timestamp
		dat_len_reg = msg->data_length;

		// Setup low data register with data[4] & data[5] & data[6] & data[7] (concat)
		dat_low_reg = (msg->data[4] << 24) | (msg->data[5] << 16) | (msg->data[6] << 8) | (msg->data[7]);
		// Setup high data register with data[0] & data[1] & data[2] & data[3] (concat)
		dat_high_reg = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | (msg->data[3]);

		CAN1->sTxMailBox[mailbox].TDLR = dat_low_reg;
		CAN1->sTxMailBox[mailbox].TDHR = dat_high_reg;
		CAN1->sTxMailBox[mailbox].TDTR = dat_len_reg;

		// Set the identifier register to start the transaction
		CAN1->sTxMailBox[mailbox].TIR = identifier_reg;

		return true;

	} else {
		return false;
	}
}

bool CAN_has_msg() {
	int fifo0 = (CAN1->RF0R & 0x03);
	int fifo1 = (CAN1->RF1R & 0x03);
	return (fifo0 > 0 || fifo1 > 0);
}

bool CAN_dequeue_msg(can_msg_t *msg) {
	if (CAN_has_msg()) {
		int fifo0 = (CAN1->RF0R & 0x03);
		int fifo1 = (CAN1->RF1R & 0x03);
		if (fifo0 > 0) {
			// Release the mail
			CAN1->RF0R |= 0x20;
			msg->identifier = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
			msg->data_length = (CAN1->sFIFOMailBox[0].RDTR) & 0x0F;
			*(uint32_t*)(msg->data) = CAN1->sFIFOMailBox[0].RDHR;
			*(uint32_t*)(msg->data + 4) = CAN1->sFIFOMailBox[0].RDLR;
		} else {
			// Release the mail
			CAN1->RF1R |= 0x20;
			msg->identifier = (CAN1->sFIFOMailBox[1].RIR >> 21) & 0x7FF;
			msg->data_length = (CAN1->sFIFOMailBox[1].RDTR) & 0x0F;
			*(uint32_t*)(msg->data) = CAN1->sFIFOMailBox[1].RDHR;
			*(uint32_t*)(msg->data + 4) = CAN1->sFIFOMailBox[1].RDLR;
		}
		return true;
	} else {
		return false;
	}
}

