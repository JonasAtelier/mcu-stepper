/* The stub definitions every test binary links. */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32_hal_stub.h"

uint32_t stub_tick;
char stub_log_last[256];

uint32_t HAL_GetTick(void)
{
	return stub_tick;
}

int __wrap_printf(const char *fmt, ...);
int __wrap_printf(const char *fmt, ...)
{
	va_list ap;
	size_t n;

	va_start(ap, fmt);
	vsnprintf(stub_log_last, sizeof(stub_log_last), fmt, ap);
	va_end(ap);

	n = strlen(stub_log_last);
	if (n && stub_log_last[n - 1] == '\n')
		stub_log_last[n - 1] = '\0';
	return fprintf(stdout, "%s\n", stub_log_last);
}

/* As the real one: at least Delay + 1 ticks, so never short. */
__attribute__((weak)) void HAL_Delay(uint32_t Delay)
{
	stub_tick += Delay + 1;
}

/*
 * src/stepper_port_stm.c is in every binary, and it calls these. A test with
 * a fake chip defines the ones it needs, which win over these weak ones; a
 * test that reaches one it did not define has a bug, so stop it there.
 */
__attribute__((weak)) void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx,
					     uint16_t GPIO_Pin,
					     GPIO_PinState PinState)
{
	(void)GPIOx;
	(void)GPIO_Pin;
	(void)PinState;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		      uint32_t Trials, uint32_t Timeout)
{
	(void)hi2c;
	(void)DevAddress;
	(void)Trials;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		 uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
		 uint16_t Size, uint32_t Timeout)
{
	(void)hi2c;
	(void)DevAddress;
	(void)MemAddress;
	(void)MemAddSize;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		  uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
		  uint16_t Size, uint32_t Timeout)
{
	(void)hi2c;
	(void)DevAddress;
	(void)MemAddress;
	(void)MemAddSize;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
			uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	(void)hi2c;
	(void)DevAddress;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
		       uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	(void)hi2c;
	(void)DevAddress;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData,
		 uint16_t Size, uint32_t Timeout)
{
	(void)hspi;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData,
			uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	(void)hspi;
	(void)pTxData;
	(void)pRxData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
		  uint16_t Size, uint32_t Timeout)
{
	(void)huart;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) HAL_StatusTypeDef
HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size,
		 uint32_t Timeout)
{
	(void)huart;
	(void)pData;
	(void)Size;
	(void)Timeout;
	abort();
}

__attribute__((weak)) int stub_uart_get_flag(UART_HandleTypeDef *huart,
					     uint32_t flag)
{
	(void)huart;
	(void)flag;
	abort();
}

__attribute__((weak)) void stub_uart_clear_ore(UART_HandleTypeDef *huart)
{
	(void)huart;
	abort();
}

__attribute__((weak)) void stub_uart_flush(UART_HandleTypeDef *huart)
{
	(void)huart;
	abort();
}
