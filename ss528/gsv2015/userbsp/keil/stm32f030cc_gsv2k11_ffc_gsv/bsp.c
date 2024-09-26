/**
 * @file bsp.c
 *
 * @brief sample bsp support
 */
#include "bsp.h"

#include "global.h"

extern uint8 LogicOutputSel;
//extern UART_HandleTypeDef huart1;
#if AvEnableKeyInput
static uint8 KeyValue;
static uint8 KeyDelay;
#endif

AvRet ManI2cWrite(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit);
AvRet ManI2cRead(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit);

/**
 * @brief  init bsp
 * @return AvOk - success
 */
AvRet BspInit(void)
{
    AvRet ret = AvOk;
    return ret;
}

/**
 * @brief  bsp i2c read function, support bus/dev address, 8/16 register address
 * @return AvOk - success
 */
AvRet BspI2cRead(uint32 devAddress, uint32 regAddress, uint8 *data, uint16 count)
{
    AvRet ret = AvOk;
    uint8 deviceAddress = (uint8)AvGetI2cDeviceAddress(devAddress);
    uint8 busAddress = (uint8)AvGetI2cBusAddress(devAddress);
    uint8 regAddressWidth = (uint8)AvGetRegAddressWidth(devAddress);
    regAddressWidth = (regAddressWidth == 0) ? 0 : 1;
    uint16 regAdress = (uint32)((AvGetRegAddress(devAddress)<<8) | AvGetRegAddress(regAddress));
    ret = ManI2cRead(deviceAddress, regAdress, data, count, busAddress, regAddressWidth);
    return ret;
}

/**
 * @brief  bsp i2c write function, support bus/dev address, 8/16 register address
 * @return AvOk - success
 */
AvRet BspI2cWrite(uint32 devAddress, uint32 regAddress, uint8 *data, uint16 count)
{
    AvRet ret = AvOk;
    uint8 deviceAddress = (uint8)AvGetI2cDeviceAddress(devAddress);
    uint8 busAddress = (uint8)AvGetI2cBusAddress(devAddress);
    uint8 regAddressWidth = (uint8)AvGetRegAddressWidth(devAddress);
    regAddressWidth = (regAddressWidth == 0) ? 0 : 1;
    uint16 regAdress = (uint32)((AvGetRegAddress(devAddress)<<8) | AvGetRegAddress(regAddress));
    ret  = ManI2cWrite(deviceAddress, regAdress, data, count, busAddress, regAddressWidth);
    return ret;
}
/*HAL_StatusTypeDef UART_Receive(uint8_t *pData, uint16_t Size)
{
    while(Size > 0)
    {
      Size--;
      if(USART1->ISR&(1<<5))
      {
         *pData++ = USART1->RDR;
         return HAL_OK;
      }
    }
    return HAL_BUSY;
}*/

//void UART_Transmit(uint8_t *pData, uint16_t Size)
//{
//    while(Size > 0)
//    {
//     Size--;
//     while((USART1->ISR&0X40)==0);
//      USART1->TDR = (*pData++);
//    }
//  /* Check that a Tx process is not already ongoing */
//}
/**
 * @brief  send one byte from uart
 * @return AvOk - success
 */
AvRet BspUartSendByte(uint8 *data, uint16 size)
{
    AvRet ret = AvOk;
#if GSV_HAL_COMPRESS
   UART_Transmit(data, size);
#else
   //HAL_UART_Transmit(&huart1, data, size, 100);
#endif
    return ret;
}

/**
 * @brief  get one byte from uart
 * @return AvOk - success
 */
AvRet BspUartGetByte(uint8 *data)
{
    AvRet ret = AvOk;
#if GSV_HAL_COMPRESS
    if(UART_Receive(data,1) != HAL_OK)
        ret = AvError;
#else
    //if(HAL_UART_Receive(&huart1, data, 1, 0) != HAL_OK)
    //    ret = AvError;
#endif
    return ret;
}

/**
 * @brief  get current time in ms
 * @return AvOk - success
 */
AvRet BspGetMilliSecond(uint32 *ms)
{
    AvRet ret = AvOk;
    //*ms = HAL_GetTick();
    return ret;
}

AvRet BspGetKey(uint8 *data)
{
    AvRet ret = AvNotAvailable;
#if AvEnableKeyInput
    uint8 value;
    uint8 CombinedValue = 0;

    value = HAL_GPIO_ReadPin(GPIOA, KEY0_Pin);
    CombinedValue = CombinedValue | ((1-value)<<0);

    if(CombinedValue == 0)
    {
        if(KeyValue != 0)
        {
            *data = KeyValue;
            ret = AvOk;
        }
        KeyValue = 0;
        KeyDelay = 0;
    }
    else if(KeyValue != CombinedValue)
    {
        KeyValue = CombinedValue;
        KeyDelay = 0;
    }
    else if(KeyValue == CombinedValue)
    {
        if(KeyDelay <= AvKeyDelayThreshold)
            KeyDelay = KeyDelay + 1;
        if(KeyDelay == AvKeyDelayThreshold)
        {
            /* Reset System */
            HAL_NVIC_SystemReset();
        }
    }

#endif
    return ret;
}
AvRet BspIrdaGetByte(uint8 *data)
{
    AvRet ret = AvOk;
    return ret;
}

//#include "i2c.c"
