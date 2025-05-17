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
#include <time.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>

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

// 左侧电机引脚定义
#define LEFT_MOTOR_IN1 2    // 左电机方向控制引脚1
#define LEFT_MOTOR_IN2 3    // 左电机方向控制引脚2
#define LEFT_MOTOR_PWM 4    // 左电机PWM控制引脚

// 右侧电机引脚定义
#define RIGHT_MOTOR_IN1 6   // 右电机方向控制引脚1
#define RIGHT_MOTOR_IN2 7   // 右电机方向控制引脚2
#define RIGHT_MOTOR_PWM 5   // 右电机PWM控制引脚

// 超声波测距引脚定义
#define TRIG_PIN 0   // 触发引脚
#define ECHO_PIN 1   // 回声引脚

// PWM参数
#define PWM_PERIOD 1000000    // PWM周期(ns)，对应1kHz
#define PWM_DUTY_HIGH 800000  // 最大占空比(满速)
#define PWM_DUTY_LOW 300000   // 低速占空比，提高最低速度的力矩

// 避障参数
#define OBSTACLE_DISTANCE 15  // 检测到障碍物的距离阈值（厘米）
#define TURN_TIME 1          // 转向时间（秒）
#define FORWARD_TIME 1       // 前进时间（秒）

// 循迹传感器相关定义
#define LINE_SENSOR1 18    // 最左侧传感器
#define LINE_SENSOR2 19    // 左侧传感器
#define LINE_SENSOR3 10    // 左中传感器
#define LINE_SENSOR4 11    // 中左传感器
#define LINE_SENSOR5 12    // 中右传感器
#define LINE_SENSOR6 13    // 右中传感器
#define LINE_SENSOR7 20    // 右侧传感器
#define LINE_SENSOR8 21    // 最右侧传感器

#define LINE_FOLLOWER_SPEED 1000000  // 增加到最大速度
#define MIN_SPEED 500000     // 提高最小速度，确保有足够的动力
#define MAX_SPEED 1000000    // 最大速度
#define SMOOTH_FACTOR 0.4    // 增加平滑因子，使速度变化更快

// PID参数
#define KP 400000    // 增加比例系数，使转向更灵敏
#define KI 30        // 减小积分系数，避免累积误差
#define KD 200000    // 增加微分系数，使转向更快速

// 全局变量用于速度平滑
static int current_left_speed = 0;
static int current_right_speed = 0;

// 测量距离函数
float measure_distance() {
    struct timeval start_time, end_time;
    float distance;
    long pulse_duration;
    int timeout_counter = 0;
    const int MAX_TIMEOUT = 30000; // 30ms超时

    usleep(10000); // 给模块上电稳定的时间 (10ms)
    
    // 设置Trig为输出模式
    pinMode(TRIG_PIN, PINMODE_OUTPUT);

    // pinMode(ECHO_PIN, PINMODE_OUTPUT);
    // digitalWrite(ECHO_PIN, LOW);

    // 设置Echo为输入模式
    pinMode(ECHO_PIN, PINMODE_INPUT);
    printf("ECHO_PIN: %d\n", digitalRead(ECHO_PIN));
    
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
// 循迹传感器初始化
int init_line_follower() {
    printf("Initializing 8-channel GPIO line follower sensors...\n");
    
    // 设置GPIO引脚为输入模式
    if (pinMode(LINE_SENSOR1, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR2, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR3, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR4, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR5, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR6, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR7, PINMODE_INPUT) != 0 ||
        pinMode(LINE_SENSOR8, PINMODE_INPUT) != 0) {
        printf("Failed to initialize line follower GPIO pins\n");
        return -1;
    }
    
    printf("Line follower initialization completed\n");
    return 0;
}

// 读取循迹传感器数据
int read_line_sensor(int fd, int *sensor1, int *sensor2, int *sensor3, int *sensor4, 
                    int *sensor5, int *sensor6, int *sensor7, int *sensor8) {
    // 直接读取GPIO引脚状态
    // 0表示检测到黑线，1表示未检测到
    *sensor1 = digitalRead(LINE_SENSOR1) == 0;  // 最左侧传感器
    *sensor2 = digitalRead(LINE_SENSOR2) == 0;  // 左侧传感器
    *sensor3 = digitalRead(LINE_SENSOR3) == 0;  // 左中传感器
    *sensor4 = digitalRead(LINE_SENSOR4) == 0;  // 中左传感器
    *sensor5 = digitalRead(LINE_SENSOR5) == 0;  // 中右传感器
    *sensor6 = digitalRead(LINE_SENSOR6) == 0;  // 右中传感器
    *sensor7 = digitalRead(LINE_SENSOR7) == 0;  // 右侧传感器
    *sensor8 = digitalRead(LINE_SENSOR8) == 0;  // 最右侧传感器

    return 0;
}

// 平滑速度变化
void smooth_speed_change(int target_left, int target_right) {
    current_left_speed = current_left_speed + (int)((target_left - current_left_speed) * SMOOTH_FACTOR);
    current_right_speed = current_right_speed + (int)((target_right - current_right_speed) * SMOOTH_FACTOR);
    
    wiringXPWMSetDuty(LEFT_MOTOR_PWM, current_left_speed);
    wiringXPWMSetDuty(RIGHT_MOTOR_PWM, current_right_speed);
}

// 循迹功能
void line_following(int runtime_seconds) {
    int fd;
    int sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8;
    time_t start_time = time(NULL);
    int last_error = 0;  // 上一次的误差
    int error = 0;       // 当前误差
    int error_sum = 0;   // 误差累积
    int error_diff = 0;  // 误差变化
    int last_left_speed = 0;  // 上一次左轮速度
    int last_right_speed = 0; // 上一次右轮速度
    
    // 初始化循迹传感器
    fd = init_line_follower();
    if (fd < 0) {
        printf("Line follower initialization failed\n");
        fflush(stdout);
        return;
    }

    printf("Starting line following...\n");
    fflush(stdout);
    
    while (time(NULL) - start_time < runtime_seconds) {
        if (read_line_sensor(0, &sensor1, &sensor2, &sensor3, &sensor4, 
                           &sensor5, &sensor6, &sensor7, &sensor8) == 0) {
            // 计算误差值（-7到7之间）
            error = 0;
            if (sensor1) error -= 7;  // 最左侧
            if (sensor2) error -= 5;  // 左侧
            if (sensor3) error -= 3;  // 左中
            if (sensor4) error -= 1;  // 中左
            if (sensor5) error += 1;  // 中右
            if (sensor6) error += 3;  // 右中
            if (sensor7) error += 5;  // 右侧
            if (sensor8) error += 7;  // 最右侧

            // 检测直角弯
            int is_sharp_turn = 0;
            if ((sensor1 && sensor2 && sensor3) || (sensor6 && sensor7 && sensor8)) {
                is_sharp_turn = 1;
                // 根据传感器状态确定转弯方向
                if (sensor1 && sensor2 && sensor3) {
                    error = -7;  // 强制左转
                } else {
                    error = 7;   // 强制右转
                }
            }
            
            // 计算误差变化
            error_diff = error - last_error;
            error_sum += error;
            
            // 限制误差累积
            if (error_sum > 15) error_sum = 15;  // 减小积分限制
            if (error_sum < -15) error_sum = -15;
            
            // 计算PID输出
            int pid_output = error * KP + error_sum * KI + error_diff * KD;
            
            // 根据误差大小动态调整基础速度
            int base_speed = LINE_FOLLOWER_SPEED;
            if (is_sharp_turn || abs(error) >= 5) {  // 直角弯或大转弯时
                base_speed = LINE_FOLLOWER_SPEED * 0.8;  // 保持较高速度
            } else if (abs(error) >= 3) {  // 中等转弯
                base_speed = LINE_FOLLOWER_SPEED * 0.8;
            }
            
            // 计算左右轮速度
            int left_speed, right_speed;
            
            if (is_sharp_turn || abs(error) >= 5) {  // 直角弯或大转弯处理
                if (error < 0) {  // 左转
                    left_speed = base_speed * 0.8;  // 左轮反转
                    right_speed = base_speed;       // 右轮正转
                    digitalWrite(LEFT_MOTOR_IN1, LOW);
                    digitalWrite(LEFT_MOTOR_IN2, HIGH);
                    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
                    digitalWrite(RIGHT_MOTOR_IN2, LOW);
                } else {  // 右转
                    left_speed = base_speed;        // 左轮正转
                    right_speed = base_speed * 0.8; // 右轮反转
                    digitalWrite(LEFT_MOTOR_IN1, HIGH);
                    digitalWrite(LEFT_MOTOR_IN2, LOW);
                    digitalWrite(RIGHT_MOTOR_IN1, LOW);
                    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
                }
            } else if (error > 0) {  // 需要小右转
                left_speed = base_speed;
                right_speed = base_speed * 0.75;  // 增加速度差
                digitalWrite(LEFT_MOTOR_IN1, HIGH);
                digitalWrite(LEFT_MOTOR_IN2, LOW);
                digitalWrite(RIGHT_MOTOR_IN1, HIGH);
                digitalWrite(RIGHT_MOTOR_IN2, LOW);
            } else if (error < 0) {  // 需要小左转
                left_speed = base_speed * 0.75;  // 增加速度差
                right_speed = base_speed;
                digitalWrite(LEFT_MOTOR_IN1, HIGH);
                digitalWrite(LEFT_MOTOR_IN2, LOW);
                digitalWrite(RIGHT_MOTOR_IN1, HIGH);
                digitalWrite(RIGHT_MOTOR_IN2, LOW);
            } else {  // 直行
                left_speed = base_speed;
                right_speed = base_speed;
                digitalWrite(LEFT_MOTOR_IN1, HIGH);
                digitalWrite(LEFT_MOTOR_IN2, LOW);
                digitalWrite(RIGHT_MOTOR_IN1, HIGH);
                digitalWrite(RIGHT_MOTOR_IN2, LOW);
            }
            
            // 限制速度范围
            if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
            if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
            if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
            if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
            
            // 平滑速度变化
            smooth_speed_change(left_speed, right_speed);
            
            // 只在误差或速度发生显著变化时输出
            if (abs(error - last_error) >= 1 || 
                abs(left_speed - last_left_speed) > 100000 || 
                abs(right_speed - last_right_speed) > 100000) {
                printf("Error: %d, Left: %d, Right: %d\n", error, left_speed, right_speed);
                fflush(stdout);
                last_left_speed = left_speed;
                last_right_speed = right_speed;
            }
            
            last_error = error;
        }
        usleep(10000);  // 10ms控制周期
    }
    
    // 停止小车
    stop();
    printf("Line following completed\n");
    fflush(stdout);
}

// 避障功能
void obstacle_avoidance(int runtime_seconds) {
    float distance;
    time_t start_time = time(NULL);
    
    printf("Starting obstacle avoidance...\n");
    fflush(stdout);
    
    while (time(NULL) - start_time < runtime_seconds) {
        // 测量距离
        distance = measure_distance();
        
        // 检测到障碍物
        if (distance > 0 && distance < OBSTACLE_DISTANCE) {
            printf("检测到障碍物，距离: %.2f cm，原地右转\n", distance);
            spin_right(PWM_DUTY_HIGH);
        } else {
            // 没有障碍物，继续前进
            printf("前方无障碍，继续前进\n");
            digitalWrite(LEFT_MOTOR_IN1, HIGH);
            digitalWrite(LEFT_MOTOR_IN2, LOW);
            digitalWrite(RIGHT_MOTOR_IN1, HIGH);
            digitalWrite(RIGHT_MOTOR_IN2, LOW);
            wiringXPWMSetDuty(LEFT_MOTOR_PWM, PWM_DUTY_HIGH);
            wiringXPWMSetDuty(RIGHT_MOTOR_PWM, PWM_DUTY_HIGH);
        }
        
        usleep(10000);  // 10ms检测一次
    }
    
    // 停止小车
    stop();
    printf("Obstacle avoidance completed\n");
    fflush(stdout);
}

// 主函数
int main() {    
    // 初始化wiringX
    if(wiringXSetup("milkv_duo", NULL) < 0) {
        printf("wiringX setup failed\n");
        return -1;
    }
    
    init_motors();

    // test_gpio();
    // hcsr04_test();
    
    // obstacle_avoidance(30);
    line_following(30);
    reset();
    return 0;
}
