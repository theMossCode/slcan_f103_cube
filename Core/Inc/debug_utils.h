/*
 * debug_utils.h
 *
 *  Created on: Feb 27, 2024
 *      Author: G3
 */

#ifndef INC_DEBUG_UTILS_H_
#define INC_DEBUG_UTILS_H_

#define DEBUG_SLCAN 1

#ifdef DEBUG
#include <stdio.h>

#if DEBUG_SLCAN
#define DEBUG_PRINT(fmt, ...) do{printf(fmt"\r\n", ##__VA_ARGS__);}while(0)
#else
#define DEBUG_PRINT(fmt, ...)
#endif // DEBUG ECI
#else
#define DEBUG_PRINT(fmt, ...)
#endif

#endif /* INC_DEBUG_UTILS_H_ */
