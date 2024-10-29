/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2024-10-29 23:55:06
 * @FilePath: \MDK-ARM\Hardware\motor.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "motor.h"
using namespace Motor;
/**
 * @brief 
 * 在电机接口类中声明，在此处定义了电机的控制输出函数，将控制量输出到电机
 * 对于大疆电调而言，数据域分16byte，data[0]为控制电流高八位，data[1]为控制电流高八位，且[0]、[1]均属于电调id1，以此类推，直到data[7]，id4
 * 注意，这个函数中只进行了一个电调id的控制。且一个电调只需要16byte的数据域
 * @param control 
 */
void MotorInterface_t::ControlOutput(int16_t control)
{
    // int16_t remap_control = control / 2; // 映射

    // 第0个是电调1的高8位，第1个是电调1的低8位，依次类推
    _common_buffer[(_id-1)%4*2] = (control >> 8) & 0xFF;
    _common_buffer[(_id-1 )%4*2+1] = (control) & 0xFF;
    if (HaveTxPermission)
    {
        uint32_t TxMailbox;//发送邮箱一般为uint32_t类型
        CAN_TxHeaderTypeDef Can_Tx;//Can_Tx 是一个结构体，用于存储要发送的 CAN 消息的头部信息，下列是根据大疆电调协议填写的
        Can_Tx.DLC = 0x08;// 数据长度设为 8 字节
        Can_Tx.ExtId = 0x0000;// 扩展 ID 设置为 0（可选）

        // 大疆的电机协议，电调id为1-4用0x200,id为5-8用0x1FF，这部分决定了不同id电调can消息的发送数据目标地址
        if (_id / 4 == 0)
        {
            Can_Tx.StdId = 0x200;
        }
        else
        {
            Can_Tx.StdId = 0x1FF;
        }

        Can_Tx.IDE = CAN_ID_STD; // 使用标准 ID
        Can_Tx.RTR = CAN_RTR_DATA;// 设定为数据帧
        Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
        while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
        {
            /* code */
        }
        HAL_CAN_AddTxMessage(_hcan, &Can_Tx, _common_buffer, &TxMailbox);
    }
}

void MotorInterface_t::update()
{
}
/**
 * @brief 
 * 电机类的构造函数，初始化电机的pid控制器
 * @param pid_config 
 */
void Motor_t::set_target(float target)
{
    _target = target * rev_fator * forward;//将目标值转化为原始数据
    pid.target_update(_target);
}

/**
 * @brief: 电机底层的更新函数，
 * @return {*}
 * @note:
 */
void Motor_t::ControlUpdate()
{
    update();
    // int16_t error = (_target + _rev_raw);
    int16_t control = pid.update(_rev_raw);//输入控制目标量转速
    ControlOutput(control);
}