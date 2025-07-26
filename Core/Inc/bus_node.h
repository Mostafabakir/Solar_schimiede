/*
 * bus_node.h
 *
 *  Created on: Jun 15, 2025
 *      Author: iliasalaur
 */

#ifndef INC_BUS_NODE_H_
#define INC_BUS_NODE_H_

#include "protocol.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void BusNode_Init(void);
void BusNode_RunLoop(void);
void BusNode_Continue(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_BUS_NODE_H_ */
