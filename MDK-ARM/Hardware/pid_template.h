/*
    * pid_template.h
    *
    *  Created on: 2024.10.28
    *      Author Elaina
    *     maintainer: Nagisa
*/
#ifndef __PID_TEMPLATE_H
#define __PID_TEMPLATE_H
//定义当前.h文件的宏，防止重复引用
template <typename T, typename T2>
//定义模板类,两个参数分别为T和T2，可以是任意类型传入
struct PidBaseConfig_T
{
    T2 kp = 0;
    T2 ki = 0;
    T2 kd = 0;
    T out_min = 0;
    T out_max = 0;
    T integral_max = 25;
};
/**
 * @brief //C++模板规则，在后续定义中，需要加上template <typename T, typename T2>声明模版函数调用模版
 * 
 * @tparam T 
 * @tparam T2 
 */
template <typename T, typename T2>
class pid_base_template_t
{
public:
    // 构造函数
    pid_base_template_t() {}; // 默认构造函数
    pid_base_template_t(T2 kp, T2 ki, T2 kd);//带3参构造函数
    pid_base_template_t(T2 kp, T2 ki, T2 kd, T out_min, T out_max);//带5参构造函数
    pid_base_template_t(PidBaseConfig_T<T, T2> config) : pid_base_template_t(config.kp, config.ki, config.kd, config.out_min, config.out_max)
    {
        error_sum_max = config.integral_max;//初始化积分上限，这个参数在前面并未传入
    };//委托构造函数，调用上面的构造函数，PidBaseConfig_T<T, T2> config传入结构体参数，并借助上述带5参的构造函数赋值，相当于一个带结构体参数的带参构造函数
    T target_;
    T2 kp_;         // 比例系数
    T2 ki_;         // 积分系数
    T2 kd_;         // 微分系数
    T last_output_; // 上一次输出值

    T update(T contrl);                                     // 更新输出，在有target情况下
    T cal(T target, T contrl, bool clear_integral = false); // 计算
    void reset(void);                                       // 重置pid控制器
    void target_update(T target, bool clear_integral = false);
    void pid_update(T2 kp, T2 ki, T2 kd); // 更新pid
    void out_limit(T out_min, T out_max); // 设置限幅
    void reset_integral(void);            // 重置积分
    T output_limit(T output);             // 输出限幅
protected:
    T error;              // 误差
    T error_sum;          // 累计误差
    T error_sum_max = 25; // 积分上限;

    T error_delta; // 误差微分
    T error_last;  // 上一次的误差

    T error_pre; // 前次的误差

    T out_min_; // 输出下限
    T out_max_; // 输出上限
};
//上述为变量，参量定义，方便下述函数实现=============================================================================================  




// 前馈pid
template <typename T, typename T2>
class pid_foward_template_t : public pid_base_template_t<T, T2>
{
public:
    pid_foward_template_t(T2 kp, T2 ki, T2 kd, T forward_k = 0) : pid_base_template_t<T, T2>(kp, ki, kd), forwardfeed_k_(forward_k) {};
    //列表初始化方法，调用父类的构造函数，并用括号初始化的方法初始化forwardfeed_k_
    pid_foward_template_t(PidBaseConfig_T<T, T2> config, T forward_k = 0) : pid_base_template_t<T, T2>(config), forwardfeed_k_(forward_k) {};
    //同上，只不过调用的是传入结构体参数的构造函数

    T update(T contrl, bool clear_integral = false); // 在有target情况下计算，更新输出的函数

    T forwardfeed();
    T cal(T target, T contrl, bool clear_integral = false); // 计算
private:
    T2 forwardfeed_k_ = 0;
};

//==================================================================================================




// 增量式pid
template <typename T, typename T2>
class pid_Increment_template_t : public pid_base_template_t<T, T2>
{
public:
    pid_Increment_template_t(PidBaseConfig_T<T, T2> config) : pid_base_template_t<T, T2>(config) {};
    T update(T contrl);
    T cal(T target, T contrl, bool clear_integral = false);

private:
    T error_last_last;
};



//上述为混合使用模版函数构成的pid核心功能函数，下列为底层简单模版函数================================================================================================







//模板函数的实现
/**
 * @brief Construct a new pid base template t<T, T2>::pid base template t object
 * 
 * @tparam T 
 * @tparam T2 
 * @param kp 
 * @param ki 
 * @param kd 
 */
template <typename T, typename T2>
pid_base_template_t<T, T2>::pid_base_template_t(T2 kp, T2 ki, T2 kd)
{
    reset();                // 初始化控制器
    pid_update(kp, ki, kd); // 更新PID参数
}

/**
 * @brief Construct a new pid base template t<T, T2>::pid base template t object
 * 
 * @tparam T 
 * @tparam T2 
 * @param kp 
 * @param ki 
 * @param kd 
 * @param out_min 
 * @param out_max 
 */

template <typename T, typename T2>
pid_base_template_t<T, T2>::pid_base_template_t(T2 kp, T2 ki, T2 kd, T out_min, T out_max)
{
    pid_update(kp, ki, kd); // 更新PID参数
    out_limit(out_min, out_max);
}


/**
 * @brief 
 * 调用reset函数，初始化pid控制器
 * 传入kp,ki,kd参数，更新pid参数
 * 该函数会先将控制器的状态重置，然后更新比例、积分和微分系数
 * @tparam T 
 * @tparam T2 
 * @param kp 
 * @param ki 
 * @param kd 
 */
template <typename T, typename T2>
void pid_base_template_t<T, T2>::pid_update(T2 kp, T2 ki, T2 kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}


/**
 * @brief 
 * 输出限幅的设置
 * @tparam T 
 * @tparam T2 
 * @param out_min 
 * @param out_max 
 */
template <typename T, typename T2>
void pid_base_template_t<T, T2>::out_limit(T out_min, T out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}



/**
 * @brief 
 * 清零函数，初始化pid控制器
 *  error = 0.0;
    error_delta = 0.0;
    error_last = 0.0;
    error_sum = 0.0;
    error_pre = 0.0;
 *  均为类内的保护成员变量，只有类内的（模板）成员函数可以访问
 * @tparam T 
 * @tparam T2 
 * 
 */
template <typename T, typename T2>
void pid_base_template_t<T, T2>::reset(void)
{
    last_output_ = 0.0f; // 上一次的控制输出值
    target_ = 0.0f;      // 控制目标值
    out_min_ = 0.0f;     // 控制输出最小值
    out_max_ = 0.0f;     // 控制输出最大值

    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;

    error = 0.0;
    error_delta = 0.0;
    error_last = 0.0;
    error_sum = 0.0;
    error_pre = 0.0;
}

/**
 * @brief 
 * 目标值更新函数,更新目标值并尝试清零误差累计项
 * @tparam T 
 * @tparam T2 
 * @param target 
 * @param clear_integral 
 */
template <typename T, typename T2>
void pid_base_template_t<T, T2>::target_update(T target, bool clear_integral)
{
    target_ = target;
    if (clear_integral)
    {
        error_sum = 0;
    }
}



/**
 * @brief 
 * 更新函数，pid的核心控制函数，位置式pid
 * 首先，误差error = target_ - contrl
 * 然后，误差累计项error_sum += error
 * 误差微分项error_delta = error_last - error
 * K p * error + K i * error_sum(求和) + K d * error_delta（前次误差-后次误差）
 * @tparam T 
 * @tparam T2 
 * @param contrl 
 * @return T 
 */
template <typename T, typename T2>
T pid_base_template_t<T, T2>::update(T contrl)
{
    error = target_ - contrl;
    error_sum += error;
    error_delta = error_last - error;
    error_last = error;
    //实现上述公式

    if (error_sum > error_sum_max)
    {
        error_sum = error_sum_max;
    }
    //积分项正限幅
    if (error_sum < -error_sum_max)
    {
        error_sum = -error_sum_max;
    }
    //积分项负限幅
    T output = kp_ * error + ki_ * error_sum + kd_ * error_delta;
    //输出值最大最小值限幅
    if (output > out_max_)
    {
        output = out_max_;
    }

    else if (output < out_min_)
    {
        output = out_min_;
    }

    last_output_ = output;

    return output;
}


/**
 * @brief 
 * 重置积分项
 * @tparam T 
 * @tparam T2 
 */
template <typename T, typename T2>
void pid_base_template_t<T, T2>::reset_integral(void)
{
    error_sum = 0;
}


/**
 * @brief 
 * 计算函数
 * @tparam T 
 * @tparam T2 
 * @param target 
 * @param contrl 
 * @param clear_integral 
 * @return T 
 */
template <typename T, typename T2>
T pid_base_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    target_update(target, clear_integral);
    return update(contrl);
    //返回update中的output
}


/**
 * @brief 
 * 输出限幅函数
 * @tparam T 
 * @tparam T2 
 * @param output 
 * @return T 
 */ 
template <typename T, typename T2>
T pid_base_template_t<T, T2>::output_limit(T output)
{
    if (output > out_max_)
    {
        output = out_max_;
    }
    else if (output < out_min_)
    {
        output = out_min_;
    }
    return output;
}

//下列为前馈pid和增量pid的具体实现模板成员函数=================================================================================================
/**
 * @brief 
 * 前馈pid模板成员函数update，pid_base_template_t<T, T2>::update(contrl)调用了父类的成员函数
 * 父类中定义的public(亲测protected类型也可)成员函数output_limit()，用于输出限幅（可以不用在派生类中定义重载）
 * this隐形指针指向当前对象的实例，调用forwardfeed()函数，返回forwardfeed_k_ * this->target_
 * @tparam T 
 * @tparam T2 
 * @param contrl 
 * @param clear_integral 
 * @return T 
 */
template <typename T, typename T2>
T pid_foward_template_t<T, T2>::update(T contrl, bool clear_integral)
{
    return this->output_limit(pid_base_template_t<T, T2>::update(contrl) + forwardfeed());
}



/**
 * @brief 
 * 前馈pid模板成员函数forwardfeed()，返回forwardfeed_k_ * this->target_
 * @tparam T 
 * @tparam T2 
 * @return T 
 */
template <typename T, typename T2>
T pid_foward_template_t<T, T2>::forwardfeed()
{
    return forwardfeed_k_ * this->target_;
}


/**
 * @brief 
 * 前馈pid模板成员函数cal，调用了父类的update函数
 * 更新目标值并试图清除误差积分项，返回update(contrl)
 * @tparam T 
 * @tparam T2 
 * @param target 
 * @param contrl 
 * @param clear_integral 
 * @return T 
 */
template <typename T, typename T2>
T pid_foward_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}
/**
 * @brief 
 * 增量式pid模板成员函数update
 * 误差error = target_ - contrl
 * 输出值output = kp_ * error + ki_ * error_last + kd_ * (error - 2 * error_last + error_last_last)
 * error_last_last = error_last
 * error_last = error
 * 输出值output限幅
 * @tparam T 
 * @tparam T2 
 * @param contrl 
 * @return T 
 */
template <typename T, typename T2>
T pid_Increment_template_t<T, T2>::update(T contrl)
{
    this->error = this->target_ - contrl;
    T output = this->kp_ * this->error + this->ki_ * this->error_last + this->kd_ * (this->error - 2 * this->error_last + this->error_last_last);

    this->error_last_last = this->error_last;
    this->error_last = this->error;

    if (output > this->out_max_)
    {
        output = this->out_max_;
    }
    else if (output < this->out_min_)
    {
        output = this->out_min_;
    }
    return output;
}


/**
 * @brief 
 * 增量式pid模板成员函数cal
 * 更新目标值并试图清除误差积分项，返回update(contrl)
 * @tparam T 
 * @tparam T2 
 * @param target 
 * @param contrl 
 * @param clear_integral 
 * @return T 
 */
template <typename T, typename T2>
T pid_Increment_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}
#endif
