/**
 * @file    Int_SI24R1.c
 * @brief   SI24R1 2.4GHz无线射频模块驱动
 * @details 实现SI24R1无线模块的SPI通信、寄存器读写、收发模式切换及数据包收发功能
 */

#include "Int_SI24R1.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

/* 发送地址 - 5字节地址，用于与接收端匹配通信 */
static u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01};

/* SPI通信超时时间，单位毫秒 */
#define SI24R1_SPI_TIMEOUT_MS 100U

/* 发送完成等待超时时间，避免IRQ异常时阻塞任务 */
#define SI24R1_TX_WAIT_TIMEOUT_MS 10U

/**
 * @brief   SPI单字节读写
 * @param   byte: 要发送的字节
 * @return  接收到的字节
 * @note    SI24R1采用全双工SPI通信，发送和接收同时进行
 */
static u8 SPI_RW(u8 byte)
{
    u8 rx_data = NOP;

    if (HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, SI24R1_SPI_TIMEOUT_MS) != HAL_OK)
    {
        Error_Handler();
    }

    return rx_data;
}

/**
 * @brief   SI24R1 GPIO初始化
 * @details 配置CSN(片选)、CE(使能)为输出模式，IRQ为输入模式(上拉)
 */
static void SI24R1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 初始化时设置CSN为高(未选中)，CE为低(待机) */
    HAL_GPIO_WritePin(SI24R1_GPIO_CSN_Port, SI24R1_GPIO_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SI24R1_GPIO_CE_Port, SI24R1_GPIO_CE_Pin, GPIO_PIN_RESET);

    /* 配置 SI24R1 片选和使能引脚，SPI1 的 SCK/MISO/MOSI 由 MX_SPI1_Init 初始化 */
    GPIO_InitStruct.Pin = SI24R1_GPIO_CSN_Pin | SI24R1_GPIO_CE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* IRQ 为模块输出的低有效中断脚 */
    GPIO_InitStruct.Pin = SI24R1_GPIO_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SI24R1_GPIO_IRQ_Port, &GPIO_InitStruct);
}

/**
 * @brief   SI24R1模块初始化
 * @note    初始化GPIO，设置CSN和CE初始电平
 */
void SI24R1_Init(void)
{
    SI24R1_GPIO_Init();
    SI24R1_CSN_HIGH();
    SI24R1_CE_LOW();
}

/**
 * @brief   写单个寄存器
 * @param   reg:   寄存器地址
 * @param   value: 要写入的值
 * @return  状态寄存器的值
 */
u8 SI24R1_Write_Reg(u8 reg, u8 value)
{
    u8 status;

    SI24R1_CSN_LOW();
    status = SPI_RW(reg);
    SPI_RW(value);
    SI24R1_CSN_HIGH();

    return status;
}

/**
 * @brief   写数据缓冲区到寄存器
 * @param   reg:   寄存器地址
 * @param   pBuf:  数据缓冲区指针
 * @param   bytes: 要写入的字节数
 * @return  状态寄存器的值
 */
u8 SI24R1_Write_Buf(u8 reg, const u8 *pBuf, u8 bytes)
{
    u8 status;
    u8 byte_ctr;

    SI24R1_CSN_LOW();
    status = SPI_RW(reg);
    for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
    {
        SPI_RW(*pBuf++);
    }
    SI24R1_CSN_HIGH();

    return status;
}

/**
 * @brief   读取单个寄存器的值
 * @param   reg: 寄存器地址
 * @return  寄存器当前值
 */
u8 SI24R1_Read_Reg(u8 reg)
{
    u8 value;

    SI24R1_CSN_LOW();
    SPI_RW(reg);
    value = SPI_RW(NOP);
    SI24R1_CSN_HIGH();

    return value;
}

/**
 * @brief   从寄存器读取数据缓冲区
 * @param   reg:   寄存器地址
 * @param   pBuf:  数据缓冲区指针(用于存储读取的数据)
 * @param   bytes: 要读取的字节数
 * @return  状态寄存器的值
 */
u8 SI24R1_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
    u8 status;
    u8 byte_ctr;

    SI24R1_CSN_LOW();
    status = SPI_RW(reg);
    for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
    {
        pBuf[byte_ctr] = SPI_RW(NOP);
    }
    SI24R1_CSN_HIGH();

    return status;
}

/**
 * @brief   设置SI24R1为接收模式
 * @details 配置接收通道0的地址、数据长度、射频通道(40)、自动应答、空中速率(1Mbps)
 * @note    配置完成后拉高CE使能接收
 */
void SI24R1_RX_Mode(void)
{
    SI24R1_CE_LOW();
    SI24R1_Write_Buf(SI24R1_WRITE_REG_CMD + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  /* 设置接收通道0地址 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + EN_AA, 0x01);                            /* 使能通道0自动应答 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + EN_RXADDR, 0x01);                       /* 使能接收通道0 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + RF_CH, 40);                              /* 设置射频通道为40 (2.440GHz) */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + RX_PW_P0, TX_PLOAD_WIDTH);               /* 设置接收数据长度 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + RF_SETUP, 0x0f);                         /* 设置空中速率1Mbps，发射功率0dBm */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + CONFIG, 0x0f);                           /* PWR_UP=1, PRIM_RX=1, 开启CRC，使能中断 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + STATUS, 0xff);                           /* 清除所有中断标志 */
    SI24R1_CE_HIGH();                                                      /* 拉高CE，进入接收模式 */
}

/**
 * @brief   设置SI24R1为发送模式
 * @details 配置发送地址、自动重传、射频通道等参数
 * @note    发送模式需要在拉高CE后尽快发送数据
 */
void SI24R1_TX_Mode(void)
{
    SI24R1_CE_LOW();
    SI24R1_Write_Buf(SI24R1_WRITE_REG_CMD + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);      /* 设置发送地址 */
    SI24R1_Write_Buf(SI24R1_WRITE_REG_CMD + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);   /* 设置接收确认地址(自动应答用) */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + EN_AA, 0x01);                             /* 使能通道0自动应答 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + EN_RXADDR, 0x01);                         /* 使能接收通道0 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + SETUP_RETR, 0x0a);                        /* 设置自动重传间隔250us，重传次数10次 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + RF_CH, 40);                               /* 设置射频通道为40 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + RF_SETUP, 0x0f);                          /* 设置空中速率1Mbps，发射功率0dBm */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + CONFIG, 0x0e);                            /* PWR_UP=1, PRIM_RX=0, 开启CRC，使能中断 */
}

/**
 * @brief   接收数据包
 * @param   rxbuf: 数据接收缓冲区指针
 * @return  0: 接收成功
 *          1: 无数据接收
 * @note    检查STATUS寄存器的RX_DR标志位，收到数据后清空RX FIFO
 */
u8 SI24R1_RxPacket(u8 *rxbuf)
{
    u8 state;

    /* 先读取并回写 STATUS，清除本次处理中断标志 */
    state = SI24R1_Read_Reg(STATUS);
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + STATUS, state);

    if (state & RX_DR)  /* RX_DR: 接收数据就绪标志 */
    {
        SI24R1_Read_Buf(RD_RX_PLOAD, rxbuf, TX_PLOAD_WIDTH);
        SI24R1_Write_Reg(FLUSH_RX, 0xff);  /* 清空RX FIFO */
        return 0;
    }

    return 1;
}

/**
 * @brief   发送数据包
 * @param   txbuf: 要发送的数据缓冲区指针
 * @return  发送状态:
 *          MAX_RT: 达到最大重发次数仍未成功
 *          TX_DS:  发送成功
 *          0xFF:   其他错误
 * @note    发送前拉低CE写入数据，写入完成后拉高CE触发发送
 */
u8 SI24R1_TxPacket(u8 *txbuf)
{
    u8 state;
    uint32_t start_tick;

    SI24R1_CE_LOW();
    SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);
    SI24R1_CE_HIGH();  /* 拉高CE触发发送 */

    /* 等待IRQ引脚变低(表示发送完成或达到最大重发次数) */
    start_tick = HAL_GetTick();
    while (SI24R1_IRQ_READ() == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start_tick) >= SI24R1_TX_WAIT_TIMEOUT_MS)
        {
            SI24R1_CE_LOW();
            SI24R1_Write_Reg(FLUSH_TX, 0xff);
            return 0xFF;
        }
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    state = SI24R1_Read_Reg(STATUS);
    /* 回写 STATUS 清除发送完成或重发失败标志 */
    SI24R1_Write_Reg(SI24R1_WRITE_REG_CMD + STATUS, state);

    if (state & MAX_RT)  /* MAX_RT: 最大重发次数标志 */
    {
        SI24R1_Write_Reg(FLUSH_TX, 0xff);  /* 清空TX FIFO */
        return MAX_RT;
    }
    if (state & TX_DS)   /* TX_DS: 发送成功标志 */
    {
        return TX_DS;
    }

    return 0xFF;
}
