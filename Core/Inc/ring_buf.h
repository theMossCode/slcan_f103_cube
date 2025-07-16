/*
 * ring_buf.h
 *
 *  Created on: Feb 3, 2025
 *      Author: collombuvi
 */

#ifndef INC_RING_BUF_H_
#define INC_RING_BUF_H_

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


#include <stdint.h>

#define RING_BUFFER_SIZE 1024

typedef uint32_t ring_size_t;

struct ring {
    uint8_t *data;
    ring_size_t size;
    uint32_t begin;
    uint32_t end;
};


uint32_t ring_write_ch(struct ring* ring, uint8_t ch);

uint32_t ring_write(struct ring* ring, uint8_t* data, ring_size_t size);

uint32_t ring_read_ch(struct ring* ring, uint8_t* ch);

uint32_t ring_bytes_free(const struct ring* rb);


void ring_setup(struct ring *ring, uint8_t *buf, ring_size_t buf_len);






#endif /* INC_RING_BUF_H_ */
