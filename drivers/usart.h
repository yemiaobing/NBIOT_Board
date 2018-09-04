#ifndef __USART_H__
#define __USART_H__
/******************************************************************************
* Include Files
******************************************************************************/
#include "stm32l1xx.h"

/******************************************************************************
* Macros
******************************************************************************/
#define	USE_UART1	1
#define	USE_UART2	0
#define	USE_UART3	0
#define	USE_UART4	0
#define	USE_UART5	0

/* 定义串口波特率和缓冲区队列大小 */
#if USE_UART1 == 1
	#define UART1_BAUD_RATE		115200
	#define UART1_TX_BUF_SIZE	1024
	#define UART1_RX_BUF_SIZE	1024
#endif

#if USE_UART2 == 1
	#define UART2_BAUD_RATE		115200
	#define UART2_TX_BUF_SIZE	1024
	#define UART2_RX_BUF_SIZE	1024
#endif

#if USE_UART3 == 1
	#define UART3_BAUD_RATE		115200
	#define UART3_TX_BUF_SIZE	1024
	#define UART3_RX_BUF_SIZE	1024
#endif

#if USE_UART4 == 1
	#define UART4_BAUD_RATE		115200
	#define UART4_TX_BUF_SIZE	1024
	#define UART4_RX_BUF_SIZE	1024
#endif

#if USE_UART5 == 1
	#define UART5_BAUD_RATE		115200
	#define UART5_TX_BUF_SIZE	1024
	#define UART5_RX_BUF_SIZE	1024
#endif
/******************************************************************************
* Enum
******************************************************************************/
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 */
	COM2 = 1,	/* USART2 PA2, PA3 */
	COM3 = 2,	/* USART3 PB10, PB11 */
	COM4 = 3,	/* UART4 PC10, PC11 */
	COM5 = 4,	/* UART5 PC12, PD2 */
}COM_PORT_ENUM;

/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
	UART_HandleTypeDef uart_handle;		/* STM32 HAL library uart handle  */
    uint8_t *p_hal_uart_txbuf;          /* STM32 HAL library uart recv buf */
    uint16_t p_hal_uart_txbuf_size;
	uint8_t *p_txbuf;			/* send buffer pointer */
	uint8_t *p_rxbuf;			/* receive buffer pointer */
	uint16_t txbuf_size;		/* send buffer size */
	uint16_t rxbuf_size;		/* receive buffer size */
	__IO uint16_t tx_write;			/* write pointer of send buffer */
	__IO uint16_t tx_read;			/* read pointer of send buffer */
	__IO uint16_t tx_count;			/* count of send buffer */

	__IO uint16_t rx_write;			/* write pointer of receive buffer */
	__IO uint16_t rx_read;			/* write pointer of receive buffer */
	__IO uint16_t rx_count;			/* write pointer of receive buffer */
}UART_T;

/******************************************************************************
* Extern Functions
******************************************************************************/
extern void uart_init(void);
extern void com_send_buf(COM_PORT_ENUM com_port, uint8_t *buf, uint16_t len);
extern void com_send_char(COM_PORT_ENUM com_port, uint8_t write_byte);
extern uint8_t com_get_char(COM_PORT_ENUM com_port, uint8_t *read_byte);
#endif
