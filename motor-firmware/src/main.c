#include "stm32f0xx.h"

#define PERIOD 1500

#define SERIAL_BUFFER_SIZE 64

volatile uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

static void init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStructure;

    // GPIOs
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF | RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |
        GPIO_Pin_9 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // AF: Timer
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);

    // AF: USART
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1); // USART TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1); // USART RX

    // Timers
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM14, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = PERIOD - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
    TIM_OC2Init(TIM3, &TIM_OCInitStruct);
    TIM_OC3Init(TIM3, &TIM_OCInitStruct);
    TIM_OC4Init(TIM3, &TIM_OCInitStruct);
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC2Init(TIM1, &TIM_OCInitStruct);

    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);

    // Clever synchronization trick
    TIM3->CNT = TIM1->CNT;
    TIM1->CNT = TIM1->CNT;

    // USART
    USART_InitStruct.USART_BaudRate = 3000000;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);

    // USART DMA
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_BufferSize = SERIAL_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)serial_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->RDR);
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    DMA_SetCurrDataCounter(DMA1_Channel3, SERIAL_BUFFER_SIZE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

static void set_a(int16_t c1, int16_t c2)
{
    if(c1 >= 0)
    {
        TIM3->CCR1 = c1;
        GPIOA->BRR = GPIO_Pin_10;
    }

    else
    {
        TIM3->CCR1 = -c1;
        GPIOA->BSRR = GPIO_Pin_10;
    }

    if(c2 >= 0)
    {
        TIM3->CCR2 = c2;
        GPIOA->BRR = GPIO_Pin_11;
    }
    else
    {
        TIM3->CCR2 = -c2;
        GPIOA->BSRR = GPIO_Pin_11;
    }
}


static void set_b(int16_t c3, int16_t c4)
{
    if(c3 >= 0)
    {
        TIM3->CCR3 = c3;
        GPIOA->BRR = GPIO_Pin_15;
    }
    else
    {
        TIM3->CCR3 = -c3;
        GPIOA->BSRR = GPIO_Pin_15;
    }

    if(c4 >= 0)
    {
        TIM3->CCR4 = c4;
        GPIOB->BRR = GPIO_Pin_3;
    }
    else
    {
        TIM3->CCR4 = -c4;
        GPIOB->BSRR = GPIO_Pin_3;
    }
}

static void set_c(int16_t c5, int16_t c6)
{
    if(c5 >= 0)
    {
        TIM1->CCR1 = c5;
        GPIOB->BRR = GPIO_Pin_4;
    }
    else
    {
        TIM1->CCR1 = -c5;
        GPIOB->BSRR = GPIO_Pin_4;
    }

    if(c6 >= 0)
    {
        TIM1->CCR2 = c6;
        GPIOB->BRR = GPIO_Pin_5;
    }
    else
    {
        TIM1->CCR2 = -c6;
        GPIOB->BSRR = GPIO_Pin_5;
    }
}

uint8_t state;

int16_t c1,c2,c3,c4,c5,c6;

uint8_t c;
uint8_t l_c;

uint16_t ptr;

int main()
{
    state = 0;
    ptr = 0;

    init();

    for(;;)
    {
        while(ptr == (SERIAL_BUFFER_SIZE - DMA1_Channel3->CNDTR - 1));

        l_c = c;
        c = serial_buffer[ptr++];
        if(ptr == SERIAL_BUFFER_SIZE) ptr = 0;

        if(l_c == 0x80 && c == 0x80)
        {
            state = 0;
            continue;
        }

        switch(state)
        {
            case 0:
                ((uint8_t*)&c1)[0] = c;
                break;
            case 1:
                ((uint8_t*)&c1)[1] = c;
                break;
            case 2:
                ((uint8_t*)&c2)[0] = c;
                break;
            case 3:
                ((uint8_t*)&c2)[1] = c;
                set_a(c1, c2);
                break;
            case 4:
                ((uint8_t*)&c3)[0] = c;
                break;
            case 5:
                ((uint8_t*)&c3)[1] = c;
                break;
            case 6:
                ((uint8_t*)&c4)[0] = c;
                break;
            case 7:
                ((uint8_t*)&c4)[1] = c;
                set_b(c3, c4);
                break;
            case 8:
                ((uint8_t*)&c5)[0] = c;
                break;
            case 9:
                ((uint8_t*)&c5)[1] = c;
                break;
            case 10:
                ((uint8_t*)&c6)[0] = c;
                break;
            case 11:
                ((uint8_t*)&c6)[1] = c;
                set_c(c5, c6);
                state=0;
                continue;
        }
        state++;
    }
}


