/*
 * ws2812double.cpp
 *
 *  Created on: Mar 23, 2022
 *      Author: paripal
 */

#include "ws2812double.h"

namespace ws2812 {

ws2812_double::update_write_buffer(){
	for(i = 0; i < pixel_num; i++){
		for(j = 0; j < color_num; j++){
			uint8_t *color = 	(j == 0) ? &ws2812_double::colors[i].green
							: 	(j == 1) ? &ws2812_double::colors[i].red
							: 	(j == 2) ? &ws2812_double::colors[i].blue : 0;
			for(k = 0; k < byte; k++){
				ws2812_double::write_buffer[(i * color_num + j) * byte + k] = ((color >> (byte -1 + k)) & 1) && 1 ? ws2812_double::high : ws2812_double::low;
			}
		}
	}
}

ws2812_double::ws2812_double(TIM_HandleTypeDef *htim, uint32_t Channel, DMA_HandleTypeDef *hdma, uint8_t high_level_pulse_len, uint8_t low_level_pulse_len) {
	// TODO Auto-generated constructor stub
	ws2812_double::htim = htim;
	ws2812_double::Channel = Channel;
	ws2812_double::hdma = hdma;
	ws2812_double::high = high_level_pulse_len;
	ws2812_double::low = low_level_pulse_len;
}

} /* namespace ws2812 */
