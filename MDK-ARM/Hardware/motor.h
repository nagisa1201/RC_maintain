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
#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"
#include "pid_template.h"
namespace Motor
{
    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor
    };
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
        //                                                                                                              };
        void ControlOutput(int16_t control);
        void update();
        int16_t _rev_raw;   // 转速原始数据
        int16_t _angle_raw; // 角度原始数据
    protected:
        MotorType_t _type = Drive_Motor;
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
        void set_target(float target);
        void ControlUpdate();
        float debug;     // 给调试用的
        int forward = 1; // 正反转
    protected:
        float _target;
        pid_base_template_t<int, float> pid = pid_base_template_t<int, float>({5, 2, 0, -5000, 5000, 2000});
    };
}
#endif
