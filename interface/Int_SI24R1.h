#ifndef INT_SI24R1_H
#define INT_SI24R1_H

#include "main.h"
#include <stdint.h>

typedef uint8_t u8;

/* SI24R1 地址宽度与单包载荷长度定义 */
#define TX_ADR_WIDTH   5
#define TX_PLOAD_WIDTH 32

/* SI24R1 模块引脚定义 */
#define SI24R1_GPIO_MOSI_Port GPIOA
#define SI24R1_GPIO_MOSI_Pin   GPIO_PIN_7
#define SI24R1_GPIO_MISO_Port GPIOA
#define SI24R1_GPIO_MISO_Pin   GPIO_PIN_6
#define SI24R1_GPIO_SCK_Port  GPIOA
#define SI24R1_GPIO_SCK_Pin   GPIO_PIN_5
#define SI24R1_GPIO_CSN_Port  GPIOA
#define SI24R1_GPIO_CSN_Pin   GPIO_PIN_4
#define SI24R1_GPIO_CE_Port   GPIOA
#define SI24R1_GPIO_CE_Pin    GPIO_PIN_8
#define SI24R1_GPIO_IRQ_Port  GPIOA
#define SI24R1_GPIO_IRQ_Pin   GPIO_PIN_9

/* SPI 指令码定义 */
#define SI24R1_READ_REG_CMD  0x00
#define SI24R1_WRITE_REG_CMD 0x20
#define RD_RX_PLOAD     0x61
#define WR_TX_PLOAD     0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

/* SI24R1 常用寄存器地址 */
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define RSSI            0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17

/* STATUS 寄存器中的关键状态位 */
#define RX_DR           0x40
#define TX_DS           0x20
#define MAX_RT          0x10

/* 引脚控制宏，简化模块时序操作 */
#define SI24R1_MOSI_HIGH() HAL_GPIO_WritePin(SI24R1_GPIO_MOSI_Port, SI24R1_GPIO_MOSI_Pin, GPIO_PIN_SET)
#define SI24R1_MOSI_LOW()  HAL_GPIO_WritePin(SI24R1_GPIO_MOSI_Port, SI24R1_GPIO_MOSI_Pin, GPIO_PIN_RESET)
#define SI24R1_MISO_READ() HAL_GPIO_ReadPin(SI24R1_GPIO_MISO_Port, SI24R1_GPIO_MISO_Pin)
#define SI24R1_SCK_HIGH()  HAL_GPIO_WritePin(SI24R1_GPIO_SCK_Port, SI24R1_GPIO_SCK_Pin, GPIO_PIN_SET)
#define SI24R1_SCK_LOW()   HAL_GPIO_WritePin(SI24R1_GPIO_SCK_Port, SI24R1_GPIO_SCK_Pin, GPIO_PIN_RESET)
#define SI24R1_CSN_HIGH()  HAL_GPIO_WritePin(SI24R1_GPIO_CSN_Port, SI24R1_GPIO_CSN_Pin, GPIO_PIN_SET)
#define SI24R1_CSN_LOW()   HAL_GPIO_WritePin(SI24R1_GPIO_CSN_Port, SI24R1_GPIO_CSN_Pin, GPIO_PIN_RESET)
#define SI24R1_CE_HIGH()   HAL_GPIO_WritePin(SI24R1_GPIO_CE_Port, SI24R1_GPIO_CE_Pin, GPIO_PIN_SET)
#define SI24R1_CE_LOW()    HAL_GPIO_WritePin(SI24R1_GPIO_CE_Port, SI24R1_GPIO_CE_Pin, GPIO_PIN_RESET)
#define SI24R1_IRQ_READ()  HAL_GPIO_ReadPin(SI24R1_GPIO_IRQ_Port, SI24R1_GPIO_IRQ_Pin)

/** 初始化 SI24R1 模块 */
void SI24R1_Init(void);

/** 写单个寄存器 */
u8 SI24R1_Write_Reg(u8 reg, u8 value);

/** 写寄存器数据缓冲区 */
u8 SI24R1_Write_Buf(u8 reg, const u8 *pBuf, u8 bytes);

/** 读单个寄存器 */
u8 SI24R1_Read_Reg(u8 reg);

/** 读寄存器数据缓冲区 */
u8 SI24R1_Read_Buf(u8 reg, u8 *pBuf, u8 bytes);

/** 切换到接收模式 */
void SI24R1_RX_Mode(void);

/** 切换到发送模式 */
void SI24R1_TX_Mode(void);

/** 接收一个 32 字节数据包 */
u8 SI24R1_RxPacket(u8 *rxbuf);

/** 发送一个 32 字节数据包 */
u8 SI24R1_TxPacket(u8 *txbuf);

#endif
