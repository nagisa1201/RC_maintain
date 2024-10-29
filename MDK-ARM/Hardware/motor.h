/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-11 15:12:25
 * @FilePath: \MDK-ARM\Hardware\motor.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
//定义了电机的接口类和电机类
#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"
#include "pid_template.h"
/**
 * @brief 
 * namespace（命名空间）用于为标识符（如变量、函数、类等）创建作用域，
 * 以避免命名冲突。将相关的代码组织在同一个命名空间中，而不必担心它与其他部分的代码发生冲突。
 * 例如在Motor空间中定义的MotorType_t枚举类型，只能在Motor空间中使用，不会与其他空间的MotorType_t发生冲突
 * 其他空间中想要引用需要Motor::MotorType_t
 */
namespace Motor
{
    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor
    };
    //区分驱动电机和转向电机
    // 接口类
    class MotorInterface_t
    {
    public:
        void bind_pin(uint8_t id, CAN_HandleTypeDef *hcan, uint8_t *common_buffer, bool HavePermission = false)
        {
            _id = id;
            _hcan = hcan;
            _common_buffer = common_buffer;
            HaveTxPermission = HavePermission;
        }
        // MotorInterface_t(uint8_t id, CAN_HandleTypeDef *hcan, uint8_t *common_buffer, bool HavePermission = false) : _id(id),
        //                                                                                                              _hcan(hcan), _common_buffer(common_buffer), HaveTxPermission(HavePermission) {
        //                                                                                                                  // HAL_CAN_Start(_hcan);
        //                  
                                                                                       
        void ControlOutput(int16_t control);
        void update();
        //声明两个函数，后续定义
        int16_t _rev_raw;   // 转速原始数据
        int16_t _angle_raw; // 角度原始数据
    protected:
        MotorType_t _type = Drive_Motor;//驱动电机
        /*因为一般原始数据要大于真实数据，所以这里取真实数据到原始数据的映射*/
        float rev_fator = 33;       // 转速系数 从原始数据转化到真实数据的倒数
        float angle_fator = 22.475; // 角度系数从真实数据到原始数据
    private:
        uint8_t *_common_buffer;
        uint8_t _id;
        CAN_HandleTypeDef *_hcan;
        bool HaveTxPermission = false;
    };

    class Motor_t : public MotorInterface_t
    {
    public:
    //同样声明函数，后续定义
        void set_target(float target);
        void ControlUpdate();
        float debug;     // 给调试用的
        int forward = 1; // 正反转
    protected:
        float _target;
        //调用的是 pid_base_template_t(PidBaseConfig_T<int, float> config) 委托构造函数：
        //大括号中为结构体的成员值
        /*
            使用大括号 {} 初始化是一种列表初始化（List Initialization）方式，
            这种语法用于直接初始化聚合类型（如结构体、数组、类等）
            传入大括号的参数时参数列表 {5, 2, 0, -5000, 5000, 2000} 视为一个聚合类型（如结构体）中的各个成员的初始化值。
        */
        pid_base_template_t<int, float> pid = pid_base_template_t<int, float>({5, 2, 0, -5000, 5000, 2000});
    };
}
#endif
