
#include "motor_rs485.h"

//static int rec_nums = 0;

void MOTOR1_B1_RS485_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(MOTOR1_485_CONTROL_PORT_RCC, ENABLE);
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOB10复用为USART3
	
	//485 control pin
    GPIO_InitStructure.GPIO_Pin = MOTOR1_485_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR1_485_CONTROL_PORT, &GPIO_InitStructure);
	GPIO_SetBits(MOTOR1_485_CONTROL_PORT, MOTOR1_485_CONTROL_PIN);
	
	/* 如果是通信频率不够，需要加入这句  */
    USART_OverSampling8Cmd(USART1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 6000000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn; //DMA发送中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	{ // RX
        DMA_InitTypeDef dma;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
        DMA_DeInit(DMA2_Stream2);
        while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
        {
        };
        dma.DMA_Channel = DMA_Channel_4; //USART3_RX Channel
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)B1Motorbuffer[0];
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
        DMA_Init(DMA2_Stream2, &dma);
        DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
	
	{ //  TX
        DMA_InitTypeDef dma;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
        DMA_DeInit(DMA2_Stream7);
        while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
        {
        };
        dma.DMA_Channel = DMA_Channel_4;
        dma.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
        dma.DMA_Memory0BaseAddr = (uint32_t)SendToB1Motor_Buff[0];
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
        DMA_Init(DMA2_Stream7, &dma);
        DMA_Cmd(DMA2_Stream7, DISABLE);
        DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
    }
	MOTOR1_485_START_READ();
}

void MOTOR1_RS485_SEND(void)
{
	MOTOR1_485_START_WRITE();
	//delay_ms(300);
	DMA_Cmd(DMA2_Stream7, ENABLE);
}

/**
 * @brief RS485的接收中断
 * @param[in] void
 */
void MOTOR1_RS485_RECV_USARTx_IRQHandler(void)
{
    uint8_t temp;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        //        uint8_t DATA_LENGTH;
		
        temp = USART1->SR;
        temp = USART1->DR;
        DMA_Cmd(DMA2_Stream2, DISABLE);
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

        A1B1RS485Receive(MOTOR1_B1_485_ID);

        DMA_SetCurrDataCounter(DMA2_Stream2, A1B1MOTOR_RECVBUF_SIZE);
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
}

/**
 * @brief RS485的发送中断
 * @param[in] void
 */
void MOTOR1_RS485_SEND_DMA_IRQHandler(void)
{
    if (DMA2->HISR & 0x08000000) //使用DMA_GetStatus()函数无法进入if体内，需修改系统文件
    {
        //DMA_Cmd(DMA1_Stream3, DISABLE);
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_FLAG_TCIF7);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_ClearFlag(USART1, USART_FLAG_TC);
		//DMA_Cmd(DMA1_Stream3, DISABLE);
        MOTOR1_485_START_READ();
    }
}
