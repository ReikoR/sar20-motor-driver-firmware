#include "drv8323.h"

uint16_t DRV8323_Write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, uint16_t value)
{
  uint16_t result;

  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET); // activate chip select
  // nSCS input setup time >=50ns
  // nSCS input hold time >=50ns
  HAL_SPI_TransmitReceive(hspi, &value, &result, 1, 1000);
  // nSCS disable time = 10ns
  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET); // deactivate chip select
  //nSCS minimum high time before active low >=400ns
  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET); // activate chip select
  // nSCS input setup time >=50ns
  // nSCS input hold time >=50ns
  HAL_SPI_TransmitReceive(hspi, &value, &result, 1, 1000); // result of previous write
  // nSCS disable time = 10ns
  HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET); // deactivate chip select

  return result;
}

uint16_t DRV8323_readRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, DRV8323_RegisterAddress address)
{
    return DRV8323_Write(hspi, CS_Port, CS_Pin, (1 << 15) | address);
}

void DRV8323_writeRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, DRV8323_RegisterAddress address, uint16_t value)
{
  DRV8323_Write(hspi, CS_Port, CS_Pin, address | value);
}
