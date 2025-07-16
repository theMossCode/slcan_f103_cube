/*
 * can_ring_buf.h
 *
 *  Created on: Apr 2, 2025
 *      Author: collo
 */

#ifndef INC_CAN_RING_BUF_H_
#define INC_CAN_RING_BUF_H_

#include <stdint.h>

struct can_data{
	uint32_t id;
	uint32_t ide;
	uint32_t rtr;
	uint32_t dlc;
	uint8_t data[8];
};

struct can_ring{
	struct can_data *can;
	uint32_t size;
	uint32_t begin;
	uint32_t end;
};

int can_ring_buf_add(struct can_ring *ring_ptr);

int can_ring_buf_get(struct can_ring *ring_ptr, struct can_data *data);

#endif /* INC_CAN_RING_BUF_H_ */
