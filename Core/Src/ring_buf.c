/*
 * ring_buf.c
 *
 *  Created on: Feb 3, 2025
 *      Author: collombuvi
 */


/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ring_buf.h"

static void ring_init(struct ring* ring, uint8_t* buf, ring_size_t size)
{
    ring->data = buf;
    ring->size = size;
    ring->begin = 0;
    ring->end = 0;
}

uint32_t ring_write_ch(struct ring* ring, uint8_t ch)
{
	uint32_t next;
	uint8_t tmp;

	next = ring->end + 1;
	if(next >= ring->size){
		next = 0;
	}

	if(next == ring->begin){
		// Discard oldest
		(void)ring_read_ch(ring, &tmp);
	}

	ring->data[ring->end] = ch;
	ring->end = next;
	return 1;
}

uint32_t ring_write(struct ring* ring, uint8_t* data, ring_size_t size)
{
    uint32_t i;

    for (i = 0; i < size; i++) {
        if (ring_write_ch(ring, data[i]) <= 0){
            return i;
        }
    }
    return i;
}

uint32_t ring_read_ch(struct ring* ring, uint8_t* ch)
{
	uint32_t next;

	next = ring->begin + 1;
	if(next >= ring->size){
		next = 0;
	}

	if(ring->begin == ring->end){
		return 0;
	}

	*ch = ring->data[ring->begin];
	ring->begin = next;
	return 1;
}

uint32_t ring_bytes_free(const struct ring* rb)
{
	if(rb->end < rb->begin){
		return (rb->size - rb->begin) + (rb->end);
	}
	else{
		return rb->end - rb->begin;
	}
}

void ring_setup(struct ring *ring, uint8_t *buf, ring_size_t buf_len)
{
	ring_init(ring, buf, buf_len);
}
