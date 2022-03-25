/*
 * CAN303x8.h
 *
 *  Created on: Mar 25, 2022
 *      Author: paripal
 */

#ifndef SRC_CAN303X8_H_
#define SRC_CAN303X8_H_

#include "stm32f3xx_hal.h"

namespace stm_CAN {

typedef enum {
	mask = CAN_FILTERMODE_IDMASK,
	list = CAN_FILTERMODE_IDLIST
}Filter_mode;

typedef enum {
	_16 = CAN_FILTERSCALE_16BIT,
	_32 = CAN_FILTERSCALE_32BIT
}Filter_scale;

typedef enum {
	disable = CAN_FILTER_DISABLE,
	enable = CAN_FILTER_ENABLE
}Filter_activation;

typedef enum {
	_0 = CAN_FILTER_FIFO0,
	_1 = CAN_FILTER_FIFO1
}FIFO;

typedef enum {
	std = CAN_ID_STD,
	ext = CAN_ID_EXT
}ID_type;

typedef enum {
	data = CAN_RTR_DATA,
	remote = CAN_RTR_REMOTE
}Frame_type;

class CAN_303x8 {
private:
	CAN_HandleTypeDef *hcan;
	uint32_t subscribed_id[28];
	uint8_t subscribed_id_len;
public:
	HAL_StatusTypeDef send(uint32_t ID, ID_type ide, Frame_type rtr, uint8_t *data, uint32_t data_len);
	CAN_303x8();
	virtual ~CAN_303x8();
};

} /* namespace CAN */

#endif /* SRC_CAN303X8_H_ */
