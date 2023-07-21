/*
 * CAN303x8.cpp
 *
 *  Created on: Mar 25, 2022
 *      Author: paripal
 */

#include "CAN303x8.h"

namespace stm_CAN {

HAL_StatusTypeDef CAN_303x8::send(uint32_t ID, ID_type ide, Frame_type rtr, uint8_t *data, uint32_t data_len) {
  if(data_len > 8) return HAL_ERROR;
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.StdId = ID;
  TxHeader.ExtId = ID;
  TxHeader.IDE = ide;
  TxHeader.RTR = rtr;
  TxHeader.DLC = data_len;
  TxHeader.TransmitGlobalTime = DISABLE;
  return HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

HAL_StatusTypeDef CAN_303x8::subscribe_message(uint32_t ID, ID_type ide, Frame_type rtr, FIFO fifo) {
  uint32_t id_for_filter = (ID << ((ide == ID_type::std) ? 21 : 3)) | ide | rtr;
  CAN_FilterTypeDef filter;
  filter.FilterIdHigh = id_for_filter >> 16;
  filter.FilterIdLow = id_for_filter;
  filter.FilterMaskIdHigh = id_for_filter >> 16;
  filter.FilterMaskIdLow = id_for_filter;
  filter.FilterScale = Filter_scale::_32;
  filter.FilterMode = Filter_mode::mask;
  filter.FilterFIFOAssignment = fifo;
  filter.FilterActivation = Filter_activation::enable;
  filter.FilterBank = 0;
  return HAL_CAN_ConfigFilter(this->hcan, &filter);
}

read_retval CAN_303x8::read(FIFO fifo, uint8_t *data) {
  uint32_t fifofilllevel = HAL_CAN_GetRxFifoFillLevel(this->hcan, fifo);
  if(fifofilllevel == 0) return read_retval::no_message;
  CAN_RxHeaderTypeDef RxHeader;
  if(HAL_CAN_GetRxMessage(this->hcan, fifo, &RxHeader, data) != HAL_OK) return read_retval::error;
  if(fifofilllevel == 1) {
    return read_retval::message_received;
  } else {
    return read_retval::more_message_received;
  }
}

read_retval CAN_303x8::read(FIFO fifo, uint8_t *data, CAN_RxHeaderTypeDef *RxHeader) {
  uint32_t fifofilllevel = HAL_CAN_GetRxFifoFillLevel(this->hcan, fifo);
  if(fifofilllevel == 0) return read_retval::no_message;
  if(HAL_CAN_GetRxMessage(this->hcan, fifo, RxHeader, data) != HAL_OK) return read_retval::error;
  if(fifofilllevel == 1) {
    return read_retval::message_received;
  } else {
    return read_retval::more_message_received;
  }
}

CAN_303x8::CAN_303x8(CAN_HandleTypeDef *hcan) {
  // TODO Auto-generated constructor stub
  this->hcan = hcan;
  HAL_CAN_Start(this->hcan);
}

CAN_303x8::~CAN_303x8() {
  // TODO Auto-generated destructor stub
  HAL_CAN_Stop(this->hcan);
}

}  // namespace stm_CAN
