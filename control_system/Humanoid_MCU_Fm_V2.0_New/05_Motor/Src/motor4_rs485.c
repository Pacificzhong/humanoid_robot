
#include "motor_rs485.h"

void MOTOR4_A1_RS485_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(MOTOR4_485_CONTROL_PORT_RCC, ENABLE);
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOB10复用为USART3
	
	//485 control pin
    GPIO_InitStructure.GPIO_Pin = MOTOR4_485_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR4_485_CONTROL_PORT, &GPIO_InitStructure);
	GPIO_SetBits(MOTOR4_485_CONTROL_PORT, MOTOR4_485_CONTROL_PIN);
	
	/* 如果是通信频率不够，需要加入这句  */
    USART_OverSampling8Cmd(UART5, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 4800000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5, ENABLE);
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
	
	USART_ClearFlag(UART5, USART_FLAG_TC);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn; //DMA发送中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	{ // RX
        DMA_InitTypeDef dma;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        DMA_DeInit(DMA1_Stream0);
        while (DMA_GetCmdStatus(DMA1_Stream0) != DISABLE)
        {
        };
        dma.DMA_Channel = DMA_Channel_4; //USART3_RX Channel
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (UART5->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)A1Motorbuffer[1];
        dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
        dma.DMA_BufferSize = A1B1MOTOR_RECVBUF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Circular;
        dma.DMA_Priority = DMA_Priority_Medium;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream0, &dma);
        DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
        DMA_Cmd(DMA1_Stream0, ENABLE);
    }
	
	{ //  TX
        DMA_InitTypeDef dma;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
        DMA_DeInit(DMA1_Stream7);
        while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE)
        {
        };
        dma.DMA_Channel = DMA_Channel_4;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (UART5->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)SendToA1Motor_Buff[1];
        dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dma.DMA_BufferSize = SEND_TO_A1B1MOTOR_BUFF_SIZE;
        dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dma.DMA_Mode = DMA_Mode_Normal;
        dma.DMA_Priority = DMA_Priority_VeryHigh;
        dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
        dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream7, &dma);
        DMA_Cmd(DMA1_Stream7, DISABLE);
        DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);
    }
	MOTOR4_485_START_READ();
}

void MOTOR4_RS485_SEND(void)
{
	MOTOR4_485_START_WRITE();
	//delay_ms(300);
	DMA_Cmd(DMA1_Stream7, ENABLE);
}

/**
 * @brief RS485的接收中断
 * @param[in] void
 */
void MOTOR4_RS485_RECV_USARTx_IRQHandler(void)
{
    //uint8_t temp;
	if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        //        uint8_t DATA_LENGTH;
		
        (void)UART5->SR;
        (void)UART5->DR;
        DMA_Cmd(DMA1_Stream0, DISABLE);
        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);

        A1B1RS485Receive(MOTOR4_A1_485_ID);

        DMA_SetCurrDataCounter(DMA1_Stream0, A1B1MOTOR_RECVBUF_SIZE);
        DMA_Cmd(DMA1_Stream0, ENABLE);
    }
}

/**
 * @brief RS485的发送中断
 * @param[in] void
 */
void MOTOR4_RS485_SEND_DMA_IRQHandler(void)
{
    if (DMA1->HISR & 0x08000000) //使用DMA_GetStatus()函数无法进入if体内，需修改系统文件
    {
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_FLAG_TCIF7);
        while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
        USART_ClearFlag(UART5, USART_FLAG_TC);
        MOTOR4_485_START_READ();
    }
}
