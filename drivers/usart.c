#include "usart.h"

/* 定义每个串口结构体变量 */
#if USE_UART1 == 1
static UART_T g_uart1;
static uint8_t uart1_tx_buf[UART1_TX_BUF_SIZE];		/* 发送缓冲区 */
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if USE_UART2 == 1
static UART_T g_uart2;
static uint8_t uart2_tx_buf[UART2_TX_BUF_SIZE];		/* 发送缓冲区 */
static uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if USE_UART3 == 1
static UART_T g_uart3;
static uint8_t uart3_tx_buf[UART3_TX_BUF_SIZE];		/* 发送缓冲区 */
static uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if USE_UART4 == 1
static UART_T g_uart4;
static uint8_t uart4_tx_buf[UART4_TX_BUF_SIZE];		/* 发送缓冲区 */
static uint8_t uart4_rx_buf[UART4_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if USE_UART5 == 1
static UART_T g_uart5;
static uint8_t uart5_tx_buf[UART5_TX_BUF_SIZE];		/* 发送缓冲区 */
static uint8_t uart5_rx_buf[UART5_RX_BUF_SIZE];		/* 接收缓冲区 */
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
    
	USART_InitStructure.USART_BaudRate = UART1_BAUD_RATE;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART1, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if USE_UART2 == 1		/* 串口2 TX = PA2， RX = PA3  */
	/* 第1步：打开GPIO和USART部件的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第3步：将USART Rx的GPIO配置为浮空输入模式
		由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*  第3步已经做了，因此这步可以不做
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第4步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART2_BAUD_RATE;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;		/* 仅选择接收模式 */
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART2, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if USE_UART3 == 1			/* 串口3 TX = PB10   RX = PB11 */
	/* 第1步： 开启GPIO和UART时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 第3步：将USART Rx的GPIO配置为浮空输入模式
		由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*  第3步已经做了，因此这步可以不做
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	*/
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 第4步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART3_BAUD_RATE;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART3, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if USE_UART4 == 1			/* 串口4 TX = PC10   RX = PC11 */
	/* 第1步： 开启GPIO和UART时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 第3步：将USART Rx的GPIO配置为浮空输入模式
		由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 第4步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART4_BAUD_RATE;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(UART4, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(UART4, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if USE_UART5 == 1			/* 串口5 TX = PC12   RX = PD2 */
	/* 第1步： 开启GPIO和UART时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

	/* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 第3步：将USART Rx的GPIO配置为浮空输入模式
		由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
		但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	/* 第4步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART5_BAUD_RATE;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(UART5, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(UART5, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
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
		/* 如果发送缓冲区已经满了，则等待缓冲区空 */	
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

		/* 将新数据填入发送缓冲区 */
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

	/* rx_write 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
	DISABLE_INT();
	usCount = p_uart->rx_count;
	ENABLE_INT();

	/* 如果读和写索引相同，则返回0 */
	//if (p_uart->rx_read == rx_write)
	if (usCount == 0)	/* 已经没有数据 */
	{
		return 0;
	}
	else
	{
		*read_byte = p_uart->p_rxbuf[p_uart->rx_read];		/* 从串口接收FIFO取1个数据 */

		/* 改写FIFO读索引 */
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
	/* 处理接收中断  */
	if (USART_GetITStatus(p_uart->uart, USART_IT_RXNE) != RESET)
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
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

	/* 处理发送缓冲区空中断 */
	if (USART_GetITStatus(p_uart->uart, USART_IT_TXE) != RESET)
	{
		if (p_uart->tx_count == 0)
		{
			/* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
			USART_ITConfig(p_uart->uart, USART_IT_TXE, DISABLE);

			/* 使能数据发送完毕中断 */
			USART_ITConfig(p_uart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			USART_SendData(p_uart->uart, p_uart->p_txbuf[p_uart->tx_read]);
			if (++p_uart->tx_read >= p_uart->txbuf_size)
			{
				p_uart->tx_read = 0;
			}
			p_uart->tx_count--;
		}

	}
	/* 数据bit位全部发送完毕的中断 */
	else if (USART_GetITStatus(p_uart->uart, USART_IT_TC) != RESET)
	{
		//if (p_uart->tx_read == p_uart->tx_write)
		if (p_uart->tx_count == 0)
		{
			/* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
			USART_ITConfig(p_uart->uart, USART_IT_TC, DISABLE);
		}
		else
		{
			/* 正常情况下，不会进入此分支 */

			/* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
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

//定义_sys_exit()以避免使用半主机模式
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE
{ 
	int handle; 

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式    
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
#if 1	/* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	com_send_char(COM1, ch);

	return ch;
#else	/* 采用阻塞方式发送每个字符,等待数据发送完毕 */
	/* 写一个字节到USART1 */
	USART_SendData(USART1, (uint8_t) ch);

	/* 等待发送结束 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
#endif
}

int fgetc(FILE *f)
{

#if 1	/* 从串口接收FIFO中取1个数据, 只有取到数据才返回 */
	uint8_t ucData;

	while(com_get_char(COM1, &ucData) == 0);

	return ucData;
#else
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
#endif
}

