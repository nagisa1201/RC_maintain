/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:26:13
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2024-10-29 21:56:05
 * @FilePath: \MDK-ARMg:\project\stm32\f427iih6\RC\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "maincpp.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
uint8_t common_buffer[8] = {0};//数据缓冲区清零
// Motor::MotorInterface_t motor;
Motor::Motor_t motor;
void can_filter_init(CAN_HandleTypeDef *_hcan);
void Configure_Filter(void);
void Serial_Printf(char *format, ...);
int main_cpp()
{
    Configure_Filter();
    motor.bind_pin(8, &hcan1, common_buffer, true);
    while (1)
    {
        // motor.ControlOutput(400);
				
        Serial_Printf("%d\n",motor._rev_raw);
        motor.set_target(motor.debug);
        motor.ControlUpdate();
        HAL_Delay(10);
    }
    return 0;
}

void can_filter_init(CAN_HandleTypeDef *_hcan)
{
    if (_hcan == &hcan2)
    {
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else
    {
        CAN_FilterTypeDef CAN_FilterConfigStructure;

        CAN_FilterConfigStructure.FilterBank = 0;
        CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
        CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
        CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
        CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
        CAN_FilterConfigStructure.FilterActivation = ENABLE;

        if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
        {
            Error_Handler();
        }
        CAN_FilterConfigStructure.FilterBank = 14;
        if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}
/**
 * @brief 
 * hal库中对can滤波器的配置
 */
void Configure_Filter(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterIdHigh = 0x0000;//过滤器的标识符高位
    sFilterConfig.FilterIdLow = 0x0000;//过滤器的标识符低位
    sFilterConfig.FilterMaskIdHigh = 0x0000;//过滤器的掩码高位
    sFilterConfig.FilterMaskIdLow = 0x0000;//过滤器的掩码低位
    //所有消息都接收且均存储
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 把接收到的报文放入到FIFO0
    //FIFO0 和 FIFO1 是两个用于缓存接收到 CAN 消息的硬件缓冲区。FIFO0 用于存储标准帧，FIFO1 用于存储扩展帧。
    sFilterConfig.FilterBank = 0;
    //FilterBank = 0 表示使用第 0 号滤波器组，即第一个滤波器组来配置和过滤 CAN 消息
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//CAN_FILTERSCALE_32BIT：表示使用 32 位的标识符进行过滤
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//CAN_FILTERMODE_IDMASK：表示使用标识符和掩码进行过滤
    sFilterConfig.FilterActivation = ENABLE;//使能过滤器
    // sFilterConfig.SlaveStartFilterBank = 14;//为从CAN实例选择启动
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) // creat CanFilter
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) // initialize can
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
    /*
        通常需要先配置滤波器，再启动 CAN 控制器。
        这是因为 CAN 控制器在启动后会立即开始接收和处理消息，
        滤波器配置必须在启动前完成，
        否则可能会导致 CAN 控制器接收到不符合期望的消息
    */
    // 当FIFO0中有消息的时候进入中
   
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // The FIFO0 receive interrupt function was enabled
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
 //这段代码的作用是启用 CAN 控制器的接收中断功能，具体来说是启用 FIFO0 接收消息的中断。如果 CAN 控制器的 FIFO0 中有新消息到达，将触发中断，通知 MCU 去处理接收到的消息
}
void Serial_Printf(char *format, ...)
{
    char String[100];              // 定义字符数组
    va_list arg;                   // 定义可变参数列表数据类型的变量arg
    va_start(arg, format);         // 从format开始，接收参数列表到arg变量
    vsprintf(String, format, arg); // 使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);                   // 结束变量arg
    // Serial_SendString(String);     // 串口发送字符数组（字符串）
    HAL_UART_Transmit(&huart7, (uint8_t *)String, strlen(String), 0xffff);
}
/**
 * @brief 
 * 此处的FIFO0指定为接收中断，当 CAN 控制器的 FIFO0 中有新消息到达，
 * 并且你已经启用了 FIFO0 的接收中断（如前面提到的 HAL_CAN_ActivateNotification 调用），
 * MCU 会产生一个中断信号，触发这个回调函数的执行
 * @param hcan 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN接收中断
{
    CAN_RxHeaderTypeDef rx_header;
    //已经定义好了CAN_TxHeaderTypeDef的结构体，用于存储要发送的 CAN 消息的头部信息，RX直接接受可直接对应赋值
    //不需要初始化 rx_header，因为它会在接收消息时被自动填充
    uint8_t rx_data[8];//定义一个8x8的数据存放区
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case 0x208:
    //大疆电调协议，反馈报文ID为0x200+id
        motor._angle_raw = (rx_data[0] << 8) | rx_data[1];//转子机械角度高八位左移为高位，与低八位或运算生成高前低后16位数据
        motor._rev_raw = (rx_data[2] << 8) | rx_data[3];//同理对转子转速进行处理
        break;
    default:
    {
        break;
    }
    }
}