/*
 * can.h
 *
 *  Created on: Apr 8, 2017
 *      Author: Tyler
 */

#ifndef MY_CAN_H_
#define MY_CAN_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint8_t data_length;

	uint16_t identifier;
	uint8_t data[8];
} can_msg_t;

void Init_MyCAN();
uint16_t create_ID(uint16_t board, uint16_t type);

bool CAN_can_transmit();
bool CAN_queue_transmit(can_msg_t *msg);

bool CAN_has_msg();
bool CAN_dequeue_msg(can_msg_t *msg);

void CAN_byte_msg(can_msg_t *msg, uint16_t identifier, uint8_t data);
void CAN_short_msg(can_msg_t *msg, uint16_t identifier, uint16_t data);
void CAN_long_msg(can_msg_t *msg, uint16_t identifier, uint32_t data);

uint8_t CAN_decode_byte(can_msg_t *msg);
uint16_t CAN_decode_short(can_msg_t *msg);
uint32_t CAN_decode_long(can_msg_t *msg);

#endif /* CAN_H_ */
