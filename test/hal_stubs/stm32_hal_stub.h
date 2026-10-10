#ifndef STM32_HAL_STUB_H
#define STM32_HAL_STUB_H

/*
 * Host stub for the slice of the STM32 HAL this library uses, standing in
 * for "stm32f4xx.h" through CMSIS_device_header (see test/Makefile). Names,
 * signatures and values match the real F4/F7 HAL; the handles carry only the
 * fields the drivers read. HAL_Delay() moves stub_tick; the tests define the
 * transfer functions their fake chips answer.
 */

#include <stdint.h>

typedef enum {
	HAL_OK		= 0x00,
	HAL_ERROR	= 0x01,
	HAL_BUSY	= 0x02,
	HAL_TIMEOUT	= 0x03,
} HAL_StatusTypeDef;

/* A port is a register block on silicon; here it only needs an identity. */
typedef struct {
	uint32_t id;
} GPIO_TypeDef;

typedef enum {
	GPIO_PIN_RESET = 0,
	GPIO_PIN_SET,
} GPIO_PinState;

#define GPIO_PIN_0		((uint16_t)0x0001)
#define GPIO_PIN_4		((uint16_t)0x0010)
#define GPIO_PIN_14		((uint16_t)0x4000)
#define GPIO_PIN_15		((uint16_t)0x8000)

typedef struct {
	uint32_t id;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT	0x00000001U

typedef struct {
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
} SPI_InitTypeDef;

typedef struct {
	SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
	uint32_t id;
} UART_HandleTypeDef;

/* The F4 values. The drivers compare against the names, never the numbers. */
#define SPI_DATASIZE_8BIT	0x00000000U
#define SPI_DATASIZE_16BIT	0x00000800U
#define SPI_POLARITY_LOW	0x00000000U
#define SPI_POLARITY_HIGH	0x00000002U
#define SPI_PHASE_1EDGE		0x00000000U
#define SPI_PHASE_2EDGE		0x00000001U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
		       GPIO_PinState PinState);

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t DevAddress, uint32_t Trials,
					uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
				   uint16_t DevAddress, uint16_t MemAddress,
				   uint16_t MemAddSize, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t DevAddress, uint16_t MemAddress,
				    uint16_t MemAddSize, uint8_t *pData,
				    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
					  uint16_t DevAddress, uint8_t *pData,
					  uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
					 uint16_t DevAddress, uint8_t *pData,
					 uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
				   const uint8_t *pData, uint16_t Size,
				   uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
					  const uint8_t *pTxData,
					  uint8_t *pRxData, uint16_t Size,
					  uint32_t Timeout);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
				    const uint8_t *pData, uint16_t Size,
				    uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);

/*
 * The UART flag macros the port uses, as functions over a fake receive
 * register: RXNE is "a stale byte is waiting", and the flush reads one.
 */
#define UART_FLAG_RXNE				0x20U
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) \
	stub_uart_get_flag((__HANDLE__), (__FLAG__))
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__)	stub_uart_clear_ore(__HANDLE__)
#define __HAL_UART_FLUSH_DRREGISTER(__HANDLE__)	stub_uart_flush(__HANDLE__)

int stub_uart_get_flag(UART_HandleTypeDef *huart, uint32_t flag);
void stub_uart_clear_ore(UART_HandleTypeDef *huart);
void stub_uart_flush(UART_HandleTypeDef *huart);

/* --- test hooks, defined in hal_stubs.c --------------------------------- */

/* What HAL_GetTick() returns, in ms; tests set it, HAL_Delay() moves it. */
extern uint32_t stub_tick;

/*
 * The last printf() line, without its newline. The tests link with
 * -Wl,--wrap=printf, so every stepper_* log line lands here as well as on
 * stdout.
 */
extern char stub_log_last[256];

#endif
