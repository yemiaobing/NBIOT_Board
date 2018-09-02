#include "usart.h"

/* ����ÿ�����ڽṹ����� */
#if USE_UART1 == 1
static UART_T g_uart1;
static uint8_t uart1_tx_buf[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if USE_UART2 == 1
static UART_T g_uart2;
static uint8_t uart2_tx_buf[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
static uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if USE_UART3 == 1
static UART_T g_uart3;
static uint8_t uart3_tx_buf[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
static uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if USE_UART4 == 1
static UART_T g_uart4;
static uint8_t uart4_tx_buf[UART4_TX_BUF_SIZE];		/* ���ͻ����� */
static uint8_t uart4_rx_buf[UART4_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if USE_UART5 == 1
static UART_T g_uart5;
static uint8_t uart5_tx_buf[UART5_TX_BUF_SIZE];		/* ���ͻ����� */
static uint8_t uart5_rx_buf[UART5_RX_BUF_SIZE];		/* ���ջ����� */
#endif

static void uart_var_init(void);
static void uart_hard_init(void);
static void uart_nvic_config(void);
static uint8_t uart_get_char(UART_T *p_uart, uint8_t *read_byte);
static void uart_send(UART_T *p_uart, uint8_t *buf, uint16_t len);

void uart_init(void)
{
	uart_var_init();

	uart_hard_init();

	uart_nvic_config();
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
	g_uart1.uart = USART1;
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
	g_uart2.uart = USART2;
	g_uart2.p_txbuf = uart2_tx_buf;
	g_uart2.p_rxbuf = uart2_rx_buf;
	g_uart2.txbuf_size = UART2_TX_BUF_SIZE;
	g_uart2.rxbuf_size = UART2_RX_BUF_SIZE;
	g_uart2.tx_write = 0;
	g_uart2.tx_read = 0;
	g_uart2.rx_write = 0;
	g_uart2.rx_read = 0;
	g_uart2.rx_count = 0;
	g_uart2.tx_count = 0;
#endif

#if USE_UART3 == 1
	g_uart3.uart = USART3;
	g_uart3.p_txbuf = uart3_tx_buf;
	g_uart3.p_rxbuf = uart3_rx_buf;
	g_uart3.txbuf_size = UART3_TX_BUF_SIZE;
	g_uart3.rxbuf_size = UART3_RX_BUF_SIZE;
	g_uart3.tx_write = 0;
	g_uart3.tx_read = 0;
	g_uart3.rx_write = 0;
	g_uart3.rx_read = 0;
	g_uart3.rx_count = 0;
	g_uart3.tx_count = 0;
#endif

#if USE_UART4 == 1
	g_uart4.uart = UART4;
	g_uart4.p_txbuf = uart4_tx_buf;
	g_uart4.p_rxbuf = uart4_rx_buf;
	g_uart4.txbuf_size = UART4_TX_BUF_SIZE;
	g_uart4.rxbuf_size = UART4_RX_BUF_SIZE;
	g_uart4.tx_write = 0;
	g_uart4.tx_read = 0;
	g_uart4.rx_write = 0;
	g_uart4.rx_read = 0;
	g_uart4.rx_count = 0;
	g_uart4.tx_count = 0;
#endif

#if USE_UART5 == 1
	g_uart5.uart = UART5;
	g_uart5.p_txbuf = uart5_tx_buf;
	g_uart5.p_rxbuf = uart5_rx_buf;
	g_uart5.txbuf_size = UART5_TX_BUF_SIZE;
	g_uart5.rxbuf_size = UART5_RX_BUF_SIZE;
	g_uart5.tx_write = 0;
	g_uart5.tx_read = 0;
	g_uart5.rx_write = 0;
	g_uart5.rx_read = 0;
	g_uart5.rx_count = 0;
	g_uart5.tx_count = 0;
#endif
}

static void uart_hard_init(void)
{
	USART_HandleTypeDef USART_InitStructure;

#if USE_UART1 == 1		
    
	USART_InitStructure.USART_BaudRate = UART1_BAUD_RATE;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(USART1, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if USE_UART2 == 1		/* ����2 TX = PA2�� RX = PA3  */
	/* ��1������GPIO��USART������ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��3������USART Rx��GPIO����Ϊ��������ģʽ
		����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
		���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*  ��3���Ѿ����ˣ�����ⲽ���Բ���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��4���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART2_BAUD_RATE;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		/* ��ѡ�����ģʽ */
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(USART2, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART2, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if USE_UART3 == 1			/* ����3 TX = PB10   RX = PB11 */
	/* ��1���� ����GPIO��UARTʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ��3������USART Rx��GPIO����Ϊ��������ģʽ
		����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
		���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*  ��3���Ѿ����ˣ�����ⲽ���Բ���
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ��4���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART3_BAUD_RATE;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(USART3, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if USE_UART4 == 1			/* ����4 TX = PC10   RX = PC11 */
	/* ��1���� ����GPIO��UARTʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ��3������USART Rx��GPIO����Ϊ��������ģʽ
		����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
		���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ��4���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART4_BAUD_RATE;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(UART4, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(UART4, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if USE_UART5 == 1			/* ����5 TX = PC12   RX = PD2 */
	/* ��1���� ����GPIO��UARTʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	/* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ��3������USART Rx��GPIO����Ϊ��������ģʽ
		����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
		���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò���
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* ��4���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART5_BAUD_RATE;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(UART5, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(UART5, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
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
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_SetPriority(USART1_IRQn,1,0);
	}

}

static void uart_send(UART_T *p_uart, uint8_t *buf, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++)
	{
		/* ������ͻ������Ѿ����ˣ���ȴ��������� */	
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = p_uart->tx_count;
			ENABLE_INT();

			if (usCount < p_uart->txbuf_size)
			{
				break;
			}
		}

		/* �����������뷢�ͻ����� */
		p_uart->p_txbuf[p_uart->tx_write] = buf[i];

		DISABLE_INT();
		if (++p_uart->tx_write >= p_uart->txbuf_size)
		{
			p_uart->tx_write = 0;
		}
		p_uart->tx_count++;
		ENABLE_INT();
	}

	USART_ITConfig(p_uart->uart, USART_IT_TXE, ENABLE);
}

static uint8_t uart_get_char(UART_T *p_uart, uint8_t *read_byte)
{
	uint16_t usCount;

	/* rx_write �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� */
	DISABLE_INT();
	usCount = p_uart->rx_count;
	ENABLE_INT();

	/* �������д������ͬ���򷵻�0 */
	//if (p_uart->rx_read == rx_write)
	if (usCount == 0)	/* �Ѿ�û������ */
	{
		return 0;
	}
	else
	{
		*read_byte = p_uart->p_rxbuf[p_uart->rx_read];		/* �Ӵ��ڽ���FIFOȡ1������ */

		/* ��дFIFO������ */
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

static void uart_irq_handler(UART_T *p_uart)
{
	/* ��������ж�  */
	if (USART_GetITStatus(p_uart->uart, USART_IT_RXNE) != RESET)
	{
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
		uint8_t ch;

		ch = USART_ReceiveData(p_uart->uart);
		p_uart->p_rxbuf[p_uart->rx_write] = ch;
		if (++p_uart->rx_write >= p_uart->rxbuf_size)
		{
			p_uart->rx_write = 0;
		}
		if (p_uart->rx_count < p_uart->rxbuf_size)
		{
			p_uart->rx_count++;
		}
	}

	/* �����ͻ��������ж� */
	if (USART_GetITStatus(p_uart->uart, USART_IT_TXE) != RESET)
	{
		if (p_uart->tx_count == 0)
		{
			/* ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�*/
			USART_ITConfig(p_uart->uart, USART_IT_TXE, DISABLE);

			/* ʹ�����ݷ�������ж� */
			USART_ITConfig(p_uart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			/* �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� */
			USART_SendData(p_uart->uart, p_uart->p_txbuf[p_uart->tx_read]);
			if (++p_uart->tx_read >= p_uart->txbuf_size)
			{
				p_uart->tx_read = 0;
			}
			p_uart->tx_count--;
		}

	}
	/* ����bitλȫ��������ϵ��ж� */
	else if (USART_GetITStatus(p_uart->uart, USART_IT_TC) != RESET)
	{
		//if (p_uart->tx_read == p_uart->tx_write)
		if (p_uart->tx_count == 0)
		{
			/* �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� */
			USART_ITConfig(p_uart->uart, USART_IT_TC, DISABLE);
		}
		else
		{
			/* ��������£��������˷�֧ */

			/* �������FIFO�����ݻ�δ��ϣ���ӷ���FIFOȡ1������д�뷢�����ݼĴ��� */
			USART_SendData(p_uart->uart, p_uart->p_txbuf[p_uart->tx_read]);
			if (++p_uart->tx_read >= p_uart->txbuf_size)
			{
				p_uart->tx_read = 0;
			}
			p_uart->tx_count--;
		}
	}
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

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE
{ 
	int handle; 

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
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
#if 1	/* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
	com_send_char(COM1, ch);

	return ch;
#else	/* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
	/* дһ���ֽڵ�USART1 */
	USART_SendData(USART1, (uint8_t) ch);

	/* �ȴ����ͽ��� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
#endif
}

int fgetc(FILE *f)
{

#if 1	/* �Ӵ��ڽ���FIFO��ȡ1������, ֻ��ȡ�����ݲŷ��� */
	uint8_t ucData;

	while(com_get_char(COM1, &ucData) == 0);

	return ucData;
#else
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
#endif
}

