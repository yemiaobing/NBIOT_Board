#include "usart.h"

#define ENABLE_INT()	__set_PRIMASK(0) /* enable global interrupt */
#define DISABLE_INT()	__set_PRIMASK(1) /* disable global interrupt */

#if USE_UART1 == 1
#define HAL_UART1_RX_BUF_SIZE   1
static uint8_t HAL_UART1_RX_BUF[HAL_UART1_RX_BUF_SIZE];
static UART_T g_uart1;
static uint8_t uart1_tx_buf[UART1_TX_BUF_SIZE];
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
#endif

#if USE_UART2 == 1
static UART_T g_uart2;
static uint8_t uart2_tx_buf[UART2_TX_BUF_SIZE];
static uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
#endif

#if USE_UART3 == 1
static UART_T g_uart3;
static uint8_t uart3_tx_buf[UART3_TX_BUF_SIZE];
static uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];
#endif

#if USE_UART4 == 1
static UART_T g_uart4;
static uint8_t uart4_tx_buf[UART4_TX_BUF_SIZE];
static uint8_t uart4_rx_buf[UART4_RX_BUF_SIZE];
#endif

#if USE_UART5 == 1
static UART_T g_uart5;
static uint8_t uart5_tx_buf[UART5_TX_BUF_SIZE];
static uint8_t uart5_rx_buf[UART5_RX_BUF_SIZE];
#endif

static void uart_var_init(void);
static void uart_hard_init(void);
static uint8_t uart_get_char(UART_T *p_uart, uint8_t *read_byte);
static void uart_send(UART_T *p_uart, uint8_t *buf, uint16_t len);

void uart_init(void)
{
	uart_var_init();

	uart_hard_init();
}

UART_T *com_to_uart(COM_PORT_ENUM com_port)
{
	if (com_port == COM1)
	{
		#if USE_UART1 == 1
			return &g_uart1;
		#else
			return 0;
		#endif
	}
	else if (com_port == COM2)
	{
		#if USE_UART2 == 1
			return &g_uart2;
		#else
			return 0;
		#endif
	}
	else if (com_port == COM3)
	{
		#if USE_UART3 == 1
			return &g_uart3;
		#else
			return 0;
		#endif
	}
	else if (com_port == COM4)
	{
		#if USE_UART4 == 1
			return &g_uart4;
		#else
			return 0;
		#endif
	}
	else if (com_port == COM5)
	{
		#if USE_UART5 == 1
			return &g_uart5;
		#else
			return 0;
		#endif
	}
	else
	{
		return 0;
	}
}

void com_send_buf(COM_PORT_ENUM com_port, uint8_t *buf, uint16_t len)
{
	UART_T *p_uart;

	p_uart = com_to_uart(com_port);
	if (p_uart == 0)
	{
		return;
	}

	uart_send(p_uart, buf, len);
}

void com_send_char(COM_PORT_ENUM com_port, uint8_t write_byte)
{
	com_send_buf(com_port, &write_byte, 1);
}

uint8_t com_get_char(COM_PORT_ENUM com_port, uint8_t *read_byte)
{
	UART_T *pUart;

	pUart = com_to_uart(com_port);
	if (pUart == 0)
	{
		return 0;
	}

	return uart_get_char(pUart, read_byte);
}

void com_clear_txfifo(COM_PORT_ENUM com_port)
{
	UART_T *p_uart;

	p_uart = com_to_uart(com_port);
	if (p_uart == 0)
	{
		return;
	}

	p_uart->tx_write = 0;
	p_uart->tx_read = 0;
	p_uart->tx_count = 0;
}

void com_clear_rxfifo(COM_PORT_ENUM com_port)
{
	UART_T *p_uart;

	p_uart = com_to_uart(com_port);
	if (p_uart == 0)
	{
		return;
	}

	p_uart->rx_write = 0;
	p_uart->rx_read = 0;
	p_uart->rx_count = 0;
}

static void uart_var_init(void)
{
#if USE_UART1 == 1
    g_uart1.p_hal_uart_txbuf = HAL_UART1_RX_BUF;
    g_uart1.p_hal_uart_txbuf_size = HAL_UART1_RX_BUF_SIZE;
	g_uart1.p_txbuf = uart1_tx_buf;
	g_uart1.p_rxbuf = uart1_rx_buf;
	g_uart1.txbuf_size = UART1_TX_BUF_SIZE;
	g_uart1.rxbuf_size = UART1_RX_BUF_SIZE;
	g_uart1.tx_write = 0;
	g_uart1.tx_read = 0;
	g_uart1.rx_write = 0;
	g_uart1.rx_read = 0;
	g_uart1.rx_count = 0;
	g_uart1.tx_count = 0;
#endif

#if USE_UART2 == 1
#endif

#if USE_UART3 == 1
#endif

#if USE_UART4 == 1
#endif

#if USE_UART5 == 1
#endif
}

static void uart_hard_init(void)
{
#if USE_UART1 == 1		
    g_uart1.uart_handle.Instance = USART1;
    g_uart1.uart_handle.Init.BaudRate = 115200;
    g_uart1.uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uart1.uart_handle.Init.StopBits = UART_STOPBITS_1;
    g_uart1.uart_handle.Init.Parity = UART_PARITY_NONE;
    g_uart1.uart_handle.Init.Mode = UART_MODE_TX_RX;
    g_uart1.uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uart1.uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&g_uart1.uart_handle);
    
    /* enable receive interrupt */
    HAL_UART_Receive_IT(&g_uart1.uart_handle, (uint8_t *)g_uart1.p_hal_uart_txbuf, g_uart1.p_hal_uart_txbuf_size);
#endif

#if USE_UART2 == 1
#endif

#if USE_UART3 == 1
#endif

#if USE_UART4 == 1
#endif

#if USE_UART5 == 1
#endif
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(huart->Instance==USART1) /* USART1 TX = PA9   RX = PA10 */
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_SetPriority(USART1_IRQn,1,0);
	}

}

static void uart_send(UART_T *p_uart, uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(&p_uart->uart_handle, buf, len, HAL_MAX_DELAY);
}

static uint8_t uart_get_char(UART_T *p_uart, uint8_t *read_byte)
{
	uint16_t usCount;

	DISABLE_INT();
	usCount = p_uart->rx_count;
	ENABLE_INT();

	if (usCount == 0)
	{
		return 0;
	}
	else
	{
		*read_byte = p_uart->p_rxbuf[p_uart->rx_read];

		DISABLE_INT();
		if (++p_uart->rx_read >= p_uart->rxbuf_size)
		{
			p_uart->rx_read = 0;
		}
		p_uart->rx_count--;
		ENABLE_INT();
		return 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_T *p_uart = NULL;
    uint8_t i = 0;
    
    if (huart->Instance == USART1)
    {
        p_uart = &g_uart1;
    }
    
    if (p_uart == NULL)
    {
        return;
    }
    
    for (i = 0; i < p_uart->p_hal_uart_txbuf_size; i++)
    {
        p_uart->p_rxbuf[p_uart->rx_write] = p_uart->p_hal_uart_txbuf[i];
        if (++p_uart->rx_write >= p_uart->rxbuf_size)
        {
            p_uart->rx_write = 0;
        }
        if (p_uart->rx_count < p_uart->rxbuf_size)
        {
            p_uart->rx_count++;
        }
    }
}

static void uart_irq_handler(UART_T *p_uart)
{
    HAL_UART_IRQHandler(&p_uart->uart_handle);
    
    while (HAL_UART_GetState(&p_uart->uart_handle) != HAL_UART_STATE_READY &&
        HAL_UART_GetState(&p_uart->uart_handle) != HAL_UART_STATE_BUSY_TX);
    /* enable receive interrupt */
    HAL_UART_Receive_IT(&p_uart->uart_handle, (uint8_t *)p_uart->p_hal_uart_txbuf, p_uart->p_hal_uart_txbuf_size);
}

#if USE_UART1 == 1
void USART1_IRQHandler(void)
{
	uart_irq_handler(&g_uart1);
}
#endif

#if USE_UART2 == 1
void USART2_IRQHandler(void)
{
	uart_irq_handler(&g_uart2);
}
#endif

#if USE_UART3 == 1
void USART3_IRQHandler(void)
{
	uart_irq_handler(&g_uart3);
}
#endif

#if USE_UART4 == 1
void UART4_IRQHandler(void)
{
	uart_irq_handler(&g_uart4);
}
#endif

#if USE_UART5 == 1
void UART5_IRQHandler(void)
{
	uart_irq_handler(&g_uart5);
}
#endif

#pragma import(__use_no_semihosting)             
struct __FILE
{ 
	int handle; 

};

FILE __stdout;  
void _sys_exit(int x) 
{ 
	x = x; 
}

void _ttywrch(int ch)
{
	ch = ch;
}

int fputc(int ch, FILE *f)
{
	com_send_char(COM1, ch);

	return ch;
}

int fgetc(FILE *f)
{
	uint8_t ucData;

	while(com_get_char(COM1, &ucData) == 0);

	return ucData;
}

