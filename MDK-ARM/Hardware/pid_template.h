#ifndef __PID_TEMPLATE_H
#define __PID_TEMPLATE_H
template <typename T, typename T2>
struct PidBaseConfig_T
{
    T2 kp = 0;
    T2 ki = 0;
    T2 kd = 0;
    T out_min = 0;
    T out_max = 0;
    T integral_max = 25;
};

// 位置式pid
template <typename T, typename T2>
class pid_base_template_t
{
public:
    // 构造函数
    pid_base_template_t() {}; // 默认构造函数
    pid_base_template_t(T2 kp, T2 ki, T2 kd);
    pid_base_template_t(T2 kp, T2 ki, T2 kd, T out_min, T out_max);
    pid_base_template_t(PidBaseConfig_T<T, T2> config) : pid_base_template_t(config.kp, config.ki, config.kd, config.out_min, config.out_max)
    {
        error_sum_max = config.integral_max;
    };

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

// 前馈pid
template <typename T, typename T2>
class pid_foward_template_t : public pid_base_template_t<T, T2>
{
public:
    pid_foward_template_t(T2 kp, T2 ki, T2 kd, T forward_k = 0) : pid_base_template_t<T, T2>(kp, ki, kd), forwardfeed_k_(forward_k) {};
    pid_foward_template_t(PidBaseConfig_T<T, T2> config, T forward_k = 0) : pid_base_template_t<T, T2>(config), forwardfeed_k_(forward_k) {};
    T update(T contrl, bool clear_integral = false); // 在有target情况下计算
    T forwardfeed();
    T cal(T target, T contrl, bool clear_integral = false); // 计算
private:
    T2 forwardfeed_k_ = 0;
};

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

template <typename T, typename T2>
pid_base_template_t<T, T2>::pid_base_template_t(T2 kp, T2 ki, T2 kd)
{
    reset();                // 初始化控制器
    pid_update(kp, ki, kd); // 更新PID参数
}

template <typename T, typename T2>
pid_base_template_t<T, T2>::pid_base_template_t(T2 kp, T2 ki, T2 kd, T out_min, T out_max)
{
    pid_update(kp, ki, kd); // 更新PID参数
    out_limit(out_min, out_max);
}

template <typename T, typename T2>
void pid_base_template_t<T, T2>::pid_update(T2 kp, T2 ki, T2 kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

template <typename T, typename T2>
void pid_base_template_t<T, T2>::out_limit(T out_min, T out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}

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

template <typename T, typename T2>
void pid_base_template_t<T, T2>::target_update(T target, bool clear_integral)
{
    target_ = target;
    if (clear_integral)
    {
        error_sum = 0;
    }
}

template <typename T, typename T2>
T pid_base_template_t<T, T2>::update(T contrl)
{
    error = target_ - contrl;
    error_sum += error;
    error_delta = error_last - error;
    error_last = error;

    if (error_sum > error_sum_max)
    {
        error_sum = error_sum_max;
    }
    if (error_sum < -error_sum_max)
    {
        error_sum = -error_sum_max;
    }

    T output = kp_ * error + ki_ * error_sum + kd_ * error_delta;
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

template <typename T, typename T2>
void pid_base_template_t<T, T2>::reset_integral(void)
{
    error_sum = 0;
}

template <typename T, typename T2>
T pid_base_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    target_update(target, clear_integral);
    return update(contrl);
}

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

template <typename T, typename T2>
T pid_foward_template_t<T, T2>::update(T contrl, bool clear_integral)
{
    return this->output_limit(pid_base_template_t<T, T2>::update(contrl) + forwardfeed());
}

template <typename T, typename T2>
T pid_foward_template_t<T, T2>::forwardfeed()
{
    return forwardfeed_k_ * this->target_;
}

template <typename T, typename T2>
T pid_foward_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}

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

template <typename T, typename T2>
T pid_Increment_template_t<T, T2>::cal(T target, T contrl, bool clear_integral)
{
    this->target_update(target, clear_integral);
    return update(contrl);
}
#endif
