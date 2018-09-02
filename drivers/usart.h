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

/* ���崮�ڲ����ʺͻ��������д�С */
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
	USART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *p_txbuf;			/* ���ͻ����� */
	uint8_t *p_rxbuf;			/* ���ջ����� */
	uint16_t txbuf_size;		/* ���ͻ�������С */
	uint16_t rxbuf_size;		/* ���ջ�������С */
	__IO uint16_t tx_write;			/* ���ͻ�����дָ�� */
	__IO uint16_t tx_read;			/* ���ͻ�������ָ�� */
	__IO uint16_t tx_count;			/* �ȴ����͵����ݸ��� */

	__IO uint16_t rx_write;			/* ���ջ�����дָ�� */
	__IO uint16_t rx_read;			/* ���ջ�������ָ�� */
	__IO uint16_t rx_count;			/* ��δ��ȡ�������ݸ��� */
}UART_T;

/******************************************************************************
* Extern Functions
******************************************************************************/
extern void uart_init(void);

#endif
