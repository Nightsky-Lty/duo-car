/**
 * Copyright (c) 2023 Milk-V
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <wiringx.h>
#include <sys/time.h>

/* 
 * MilkV Duo智能小车轮子控制示例代码
 *
 * 注意：确保设备在3.3伏而不是5伏的电压下使用。
 * Duo GPIO 不能在5伏电平下使用。
 * 
 * 假设小车使用双电机驱动，每个电机需要2个GPIO引脚控制方向，一个PWM引脚控制速度
 * 
 * 请根据实际硬件连接情况修改引脚定义
 */
// 空指针
#define NULL ((void *)0)

// 左侧电机引脚定义
#define LEFT_MOTOR_IN1 2    // 左电机方向控制引脚1
#define LEFT_MOTOR_IN2 3    // 左电机方向控制引脚2
#define LEFT_MOTOR_PWM 4    // 左电机PWM控制引脚

// 右侧电机引脚定义
#define RIGHT_MOTOR_IN1 6   // 右电机方向控制引脚1
#define RIGHT_MOTOR_IN2 7   // 右电机方向控制引脚2
#define RIGHT_MOTOR_PWM 5   // 右电机PWM控制引脚

// 超声波测距引脚定义
#define TRIG_PIN 3   // 触发引脚
#define ECHO_PIN 4   // 回声引脚

// PWM参数
#define PWM_PERIOD 100000     // PWM周期(ns)
#define PWM_DUTY_HIGH 100000   // 最大占空比(满速)
#define PWM_DUTY_LOW 70000   // 低速占空比

// 测量距离函数
float measure_distance() {
    struct timeval start_time, end_time;
    float distance;
    long pulse_duration;
    int timeout_counter = 0;
    const int MAX_TIMEOUT = 30000; // 30ms超时

    // 设置引脚2为高电平，给超声波模块供电
    pinMode(2, PINMODE_OUTPUT);
    digitalWrite(2, HIGH);
    usleep(10000); // 给模块上电稳定的时间 (10ms)
    
    // 设置Trig为输出模式
    pinMode(TRIG_PIN, PINMODE_OUTPUT);
    // 设置Echo为输入模式
    pinMode(ECHO_PIN, PINMODE_INPUT);
    
    // 确保Trig引脚处于低电平
    digitalWrite(TRIG_PIN, LOW);
    usleep(2);
    
    // 发送10us高电平脉冲到Trig引脚
    digitalWrite(TRIG_PIN, HIGH);
    usleep(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // 等待Echo引脚变为高电平（开始接收回声）
    timeout_counter = 0;
    while(digitalRead(ECHO_PIN) == LOW) {
        usleep(1);
        timeout_counter++;
        if(timeout_counter > MAX_TIMEOUT) {
            printf("等待Echo变高超时\n");
            return -1; // 返回错误值
        }
    }
    
    // Echo变为高电平，开始计时
    gettimeofday(&start_time, NULL);
    
    // 等待Echo引脚变为低电平（回声接收完成）
    timeout_counter = 0;
    while(digitalRead(ECHO_PIN) == HIGH) {
        usleep(1);
        timeout_counter++;
        if(timeout_counter > MAX_TIMEOUT) {
            printf("等待Echo变低超时\n");
            return -1; // 返回错误值
        }
    }
    
    // Echo变回低电平，结束计时
    gettimeofday(&end_time, NULL);
    
    // 计算脉冲持续时间（微秒）
    pulse_duration = (end_time.tv_sec - start_time.tv_sec) * 1000000 + 
                     (end_time.tv_usec - start_time.tv_usec);
    
    // 声速是340m/s或34000cm/s
    // 脉冲往返时间，所以除以2
    distance = (pulse_duration / 2.0) * 0.0343;
    
    printf("脉冲持续时间: %ld 微秒, 距离: %.2f 厘米\n", pulse_duration, distance);
    
    return distance;
}

// 初始化电机控制引脚
int init_motors() {
    int result = 0;
    
    printf("设置引脚模式...\n");
    // 设置GPIO引脚为输出模式
    result = pinMode(LEFT_MOTOR_IN1, PINMODE_OUTPUT);
    if(result != 0) {
        printf("错误: 设置LEFT_MOTOR_IN1为输出模式失败，错误码: %d\n", result);
        return 0;
    }
    
    result = pinMode(LEFT_MOTOR_IN2, PINMODE_OUTPUT);
    if(result != 0) {
        printf("错误: 设置LEFT_MOTOR_IN2为输出模式失败，错误码: %d\n", result);
        return 0;
    }
    
    result = pinMode(RIGHT_MOTOR_IN1, PINMODE_OUTPUT);
    if(result != 0) {
        printf("错误: 设置RIGHT_MOTOR_IN1为输出模式失败，错误码: %d\n", result);
        return 0;
    }
    
    result = pinMode(RIGHT_MOTOR_IN2, PINMODE_OUTPUT);
    if(result != 0) {
        printf("错误: 设置RIGHT_MOTOR_IN2为输出模式失败，错误码: %d\n", result);
        return 0;
    }
    
    printf("方向控制引脚设置完成\n");
    
    printf("初始化PWM...\n");
    // 初始化PWM
    result = wiringXPWMSetPeriod(LEFT_MOTOR_PWM, PWM_PERIOD);
    if(result != 0) {
        printf("错误: 设置LEFT_MOTOR_PWM周期失败，错误码: %d\n", result);
        printf("可能PWM引脚 %d 不支持PWM功能\n", LEFT_MOTOR_PWM);
        return 0;
    }
    
    result = wiringXPWMSetPeriod(RIGHT_MOTOR_PWM, PWM_PERIOD);
    if(result != 0) {
        printf("错误: 设置RIGHT_MOTOR_PWM周期失败，错误码: %d\n", result);
        printf("可能PWM引脚 %d 不支持PWM功能\n", RIGHT_MOTOR_PWM);
        return 0;
    }
    
    printf("PWM周期设置完成\n");
    
    printf("设置PWM极性...\n");
    // 设置PWM极性
    result = wiringXPWMSetPolarity(LEFT_MOTOR_PWM, 0);
    if(result != 0) {
        printf("错误: 设置LEFT_MOTOR_PWM极性失败，错误码: %d\n", result);
        return 0;
    }
    
    result = wiringXPWMSetPolarity(RIGHT_MOTOR_PWM, 0);
    if(result != 0) {
        printf("错误: 设置RIGHT_MOTOR_PWM极性失败，错误码: %d\n", result);
        return 0;
    }

    wiringXPWMSetDuty(LEFT_MOTOR_PWM, PWM_DUTY_HIGH);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, PWM_DUTY_HIGH);
    
    printf("PWM极性设置完成\n");
    
    printf("设置初始状态...\n");
    // 初始状态为停止
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    
    printf("初始状态设置完成\n");
    
    printf("启用PWM...\n");
    result = wiringXPWMEnable(LEFT_MOTOR_PWM, 1);
    if(result != 0) {
        printf("错误: 启用LEFT_MOTOR_PWM失败，错误码: %d\n", result);
        return 0;
    }
    
    result = wiringXPWMEnable(RIGHT_MOTOR_PWM, 1);
    if(result != 0) {
        printf("错误: 启用RIGHT_MOTOR_PWM失败，错误码: %d\n", result);
        return 0;
    }
    
    // printf("PWM启用完成\n");
    printf("电机初始化完成\n");
    return 1;
}

void reset()
{
    // 禁用PWM
    wiringXPWMEnable(LEFT_MOTOR_PWM, 0);
    wiringXPWMEnable(RIGHT_MOTOR_PWM, 0);

    // 复位引脚状态
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);

    // 清理wiringX资源
    wiringXGC();

    printf("系统已复位\n");
}


void stop()
{
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void move_forward(int speed)
{
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void move_backward(int speed) 
{
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

void move_left(int speed)
{
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed * 0.8);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void move_right(int speed)
{
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed * 0.8);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// 原地左转 - 左轮反转，右轮正转
void spin_left(int speed) {
    // 设置PWM速度
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed);
    
    // 设置方向：左轮反转，右轮正转
    digitalWrite(LEFT_MOTOR_IN1, LOW);    // 左电机反转
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);  // 右电机正转
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

// 原地右转 - 左轮正转，右轮反转
void spin_right(int speed) {
    // 设置PWM速度
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, speed);
    
    // 设置方向：左轮正转，右轮反转
    digitalWrite(LEFT_MOTOR_IN1, HIGH);   // 左电机正转
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    
    digitalWrite(RIGHT_MOTOR_IN1, LOW);   // 右电机反转
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

// 主函数中添加
void hcsr04_test() {
    float distance;
    
    printf("HC-SR04超声波测距测试\n");
    printf("按Ctrl+C退出\n");
    
    while(1) {
        distance = measure_distance();
        printf("距离: %.2f 厘米\n", distance);
        sleep(1);  // 每秒测量一次
        fflush(stdout);
    }
}

void test_gpio() {
    printf("测试GPIO引脚...\n");
    
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, PWM_DUTY_HIGH);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, PWM_DUTY_HIGH);


    // 测试方向控制引脚
    printf("测试左电机方向控制引脚...\n");
    printf("正转\n");
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    sleep(3);
    
    printf("反转\n");
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    sleep(3);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    
    printf("测试右电机方向控制引脚...\n");
    printf("正转\n");
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    sleep(3);
    
    printf("反转\n");
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    sleep(3);

    
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    
    printf("前进 慢速\n");
    move_forward(PWM_DUTY_LOW);
    sleep(3);

    printf("前进 快速\n");
    move_forward(PWM_DUTY_HIGH);
    sleep(3);
    stop();

    printf("后退\n");
    move_backward(PWM_DUTY_HIGH);
    sleep(3);
    stop();

    printf("左转\n");
    move_left(PWM_DUTY_HIGH);
    sleep(3);
    stop();

    printf("右转\n");
    move_right(PWM_DUTY_HIGH);
    sleep(3);
    stop();

    printf("差速\n");
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, PWM_DUTY_HIGH);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, PWM_DUTY_LOW);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    sleep(3);
    stop();

    printf("GPIO测试完成\n");
}
void test_pin_high(int pin, int duration) {
    pinMode(pin, PINMODE_OUTPUT);
    printf("测试引脚%d\n", pin);
    digitalWrite(pin, HIGH);
    sleep(duration);
    digitalWrite(pin, LOW);
    printf("测试引脚%d结束\n", pin);
}
void test_pin_pwm(int pin, int duration) 
{
    printf("测试引脚%d\n", pin);
    wiringXPWMSetPeriod(pin, PWM_PERIOD);
    wiringXPWMSetDuty(pin, PWM_DUTY_HIGH);
    wiringXPWMSetPolarity(pin, 0);
    wiringXPWMEnable(pin, 1);
    printf("高占空比\n");
    sleep(duration);
    printf("低占空比\n");
    wiringXPWMSetDuty(pin, PWM_DUTY_LOW);
    sleep(duration);
    printf("停止\n");
    printf("测试引脚%d结束\n", pin);
    wiringXPWMEnable(pin, 0);
    pinMode(pin, PINMODE_OUTPUT);
    digitalWrite(pin, LOW);
    wiringXGC();
}


// 主函数
int main() {
    
    // 初始化wiringX
    if(wiringXSetup("milkv_duo", NULL) == -1) {
        printf("wiringX初始化失败\n");
        wiringXGC();
        return 1;
    }
    
    init_motors();

    // pinMode(9, PINMODE_OUTPUT);
    // digitalWrite(9, LOW);

    // test_pin_high(2, 5);
    // test_pin_pwm(3, 5);
    // hcsr04_test();

    test_gpio();
    reset();
    return 0;
}
