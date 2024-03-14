#include "Chassis_task.h"
#include "cmsis_os.h"
// #include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
// #include "imu_task.h"
#include "imu_task.h"
#include "JY901.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "arm_math.h"
#include "struct_typedef.h"

#define KEY_START_OFFSET 3
#define KEY_STOP_OFFSET 20
#define RC_MAX 660
#define RC_MIN -660
#define motor_max 2000
#define motor_min -2000
#define angle_valve 5
#define angle_weight 55
float pid_yaw_angle_value[3];
float pid_yaw_speed_value[3];
chassis_t chassis;
// extern INS_t INS;
// pid_struct_t pid_yaw_angle;
// pid_struct_t pid_yaw_speed;
#define chassis_speed_max 2000
float Yaw;
extern float yaw12;
// float yaw;
extern float Yaw_top ;//float Yaw_top;
float Yaw_update;
float Yaw_init;
int yaw_correction_flag = 1; // yaw值校正标志

pid_struct_t supercap_pid;
motor_info_t motor_info_chassis[10]; // 电机信息结构体
fp32 superpid[3] = {120, 0.1, 0};

int8_t chassis_mode;
float relative_yaw = 0;
extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体
extern float powerdata[4];
extern uint16_t shift_flag;
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow;
uint8_t rc[18];

static void Chassis_Init();

static void Chassis_loop_Init();

// 模式选择
static void mode_chooce();

// 遥控器控制底盘电机
static void RC_Move(void);

// 小陀螺模式
static void gyroscope(void);

// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);

// 电机电流控制
static void chassis_current_give();

// 运动解算
static void chassis_motol_speed_calculate();

static void chassis_follow();

static void yaw_correct();

void Chassis_task(void const *pvParameters)
{
  Chassis_Init();
  // imu_task_init();
  // imu_task();
 for (;;) // 底盘运动任务
  {
    Chassis_loop_Init();

    yaw_correct(); // 校准

    // 选择底盘运动模式
    mode_chooce();

    // 电机速度解算
    chassis_motol_speed_calculate();

    // 电机电流控制
// Motor_Speed_limiting(chassis.speed_target,motor_max);

    chassis_current_give();
    // imu_task();
    osDelay(1);
  }
}

static void Chassis_Init()
{
  chassis.pid_parameter[0] = 20, chassis.pid_parameter[1] = 0.0, chassis.pid_parameter[2] = 5;

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis.pid[i], chassis.pid_parameter, 6000, 6000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }
  pid_init(&supercap_pid, superpid, 3000, 3000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

  chassis.Vx = 0, chassis.Vy = 0, chassis.Wz = 0;
}

static void Chassis_loop_Init()
{
  chassis.Vx = 0;
  chassis.Vy = 0;
  chassis.Wz = 0;
  pid_yaw_angle_value[0] = 1;
  pid_yaw_angle_value[1] = 0;
  pid_yaw_angle_value[2] = 0;

  pid_yaw_speed_value[0] = 10;
  pid_yaw_speed_value[1] = 0;
  pid_yaw_speed_value[2] = 0;
}

static void mode_chooce()
{
  // 遥控器控制
  // chanel 0 left max==-660,right max==660
  // chanel 1 up max==660,down max==-660
  // chanel 2 left max==-660,right max==660
  // chanel 3 up max==660,down max==-660
  // chanel 4 The remote control does not have this channel

  if (rc_ctrl.rc.s[0] == 1)
  {
    // LEDB_ON(); // BLUE LED
    // LEDR_OFF();
    // LEDG_OFF();
    gyroscope();
  }
  else if (rc_ctrl.rc.s[0] == 2)
  {
    // LEDG_ON(); // GREEN LED
    // LEDR_OFF();
    // LEDB_OFF();
  chassis_follow();

  }
  else if (rc_ctrl.rc.s[0] == 3)
  {
    // LEDR_ON(); // RED LED
    // LEDB_OFF();
    // LEDG_OFF();
    RC_Move();
  }
  // else
  // {
  //   // LEDR_OFF();
  //   // LEDB_OFF();
  //   // LEDG_OFF();
  // }
}

// // 运动解算
static void chassis_motol_speed_calculate()
{

  // 根据分解的速度调整电机速度目标
  // chassis.speed_target[CHAS_LF] = 3*(-chassis.Wz)*0.4 + chassis.Vx + chassis.Vy;
  // chassis.speed_target[CHAS_RF] = 3*(-chassis.Wz)*0.4 + chassis.Vx - chassis.Vy;
  // chassis.speed_target[CHAS_RB] = 3*(-chassis.Wz)*0.4 - chassis.Vx - chassis.Vy;
  // chassis.speed_target[CHAS_LB] = 3*(-chassis.Wz)*0.4 - chassis.Vx + chassis.Vy;

    chassis.speed_target[CHAS_LF] = 3*(-chassis.Wz)*0.4 + chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RF] = 3*(-chassis.Wz)*0.4 - chassis.Vx - chassis.Vy;
  chassis.speed_target[CHAS_RB] = 3*(-chassis.Wz)*0.4 + chassis.Vx + chassis.Vy;
  chassis.speed_target[CHAS_LB] = 3*(-chassis.Wz)*0.4 - chassis.Vx + chassis.Vy;
}
// 运动解算
// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}

// 电机电流控制
static void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    chassis.motor_info[i].set_current = pid_calc(&chassis.pid[i], chassis.motor_info[i].rotor_speed, chassis.speed_target[i]);
  }
  set_motor_current_chassis(0, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
  // set_curruent(MOTOR_3508_0, hcan1, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
}

// 线性映射函数
static int16_t map_range(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值映射到[0, 1]的范围
  double normalized_value = (value * 1.0 - from_min * 1.0) / (from_max * 1.0 - from_min * 1.0);

  // 然后将[0, 1]的范围映射到[to_min, to_max]的范围
  int16_t mapped_value = (int16_t)(normalized_value * (to_max - to_min) + to_min);

  return mapped_value;
}

static void RC_Move(void)
{
  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  chassis.Wz = rc_ctrl.rc.ch[4]; // 旋转输入

  /*************记得加上线性映射***************/
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Wz = map_range(chassis.Wz, RC_MIN, RC_MAX, motor_min, motor_max);
}

// 小陀螺模式
static void gyroscope(void)
{

  chassis.Wz = 1000;
  // chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  // chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  /*************记得加上线性映射***************/
  chassis.Vx = map_range(rc_ctrl.rc.ch[3], RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Vy = map_range(rc_ctrl.rc.ch[2], RC_MIN, RC_MAX, motor_min, motor_max);
  // int16_t Temp_Vx = chassis.Vx;
  // int test111=cos(1);
  // int16_t Temp_Vy = chassis.Vy;
  //  relative_yaw = Yaw - INS_top.Yaw;//改变量 这里是下面的减去上面的 但是按照五号的来 需要下面的陀螺仪减去上面的c板
   relative_yaw = yaw12 - Yaw_top; //暂时没有加陀螺仪代码 需要改
  relative_yaw = -relative_yaw / 57.3f; // 此处加负是因为旋转角度后，旋转方向相反
  // chassis.Vx = cosf(relative_yaw) * Temp_Vx - sinf(relative_yaw) * Temp_Vy;
  // chassis.Vy = sinf(relative_yaw) * Temp_Vx + cosf(relative_yaw) * Temp_Vy;


  int16_t temp_Vx = 0;
  int16_t temp_Vy = 0;
  temp_Vx = chassis.Vx * cosf(relative_yaw) - chassis.Vy * sinf(relative_yaw);
  temp_Vy = chassis.Vx * sinf(relative_yaw) + chassis.Vy * cosf(relative_yaw);
  chassis.Vx = temp_Vx;
  chassis.Vy = temp_Vy;



}
// 底盘跟随云台
static void chassis_follow(void)
{

  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  chassis.Wz = rc_ctrl.rc.ch[4]; // 旋转输入
  /*************记得加上线性映射***************/
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max);

  // int16_t relative_yaw = Yaw - INS.Yaw_update; // 最新的减去上面的
  int16_t relative_yaw = Yaw_update-Yaw_top;
  int16_t yaw_speed = pid_calc(&pid_yaw_angle, 0, relative_yaw);
  int16_t rotate_w = (chassis.motor_info[0].rotor_speed + chassis.motor_info[1].rotor_speed + chassis.motor_info[2].rotor_speed + chassis.motor_info[3].rotor_speed) / (4 * 19);
  // 消除静态旋转
  if (relative_yaw > -2 && relative_yaw < 2)
  {
    chassis.Wz = 0;
  }
  else
  {
    chassis.Wz = pid_calc(&pid_yaw_speed, yaw_speed, rotate_w);
  }
  int16_t Temp_Vx = chassis.Vx;
  int16_t Temp_Vy = chassis.Vy;

  chassis.Vx = cos(-relative_yaw / 57.3f) * Temp_Vx - sin(-relative_yaw / 57.3f) * Temp_Vy;
  chassis.Vy = sin(-relative_yaw / 57.3f) * Temp_Vx + cos(-relative_yaw / 57.3f) * Temp_Vy;
}
// /*************************yaw值校正*******************************/
static void yaw_correct(void)
{
  // 只执行一次
  if (yaw_correction_flag)
  {
    yaw_correction_flag = 0;
    Yaw_init = yaw12;
  }
  Yaw_update = yaw12 - Yaw_init;
}
/*************************** 键盘控制函数 ************************/
static void key_control(void)
{
  if (d_flag)
    key_y_fast += KEY_START_OFFSET;
  else
    key_y_fast -= KEY_STOP_OFFSET;
  if (a_flag)
    key_y_slow += KEY_START_OFFSET;
  else
    key_y_slow -= KEY_STOP_OFFSET;
  if (w_flag)
    key_x_fast += KEY_START_OFFSET;
  else
    key_x_fast -= KEY_STOP_OFFSET;
  if (s_flag)
    key_x_slow += KEY_START_OFFSET;
  else
    key_x_slow -= KEY_STOP_OFFSET;

  if (key_x_fast > chassis_speed_max)
    key_x_fast = chassis_speed_max;
  if (key_x_fast < 0)
    key_x_fast = 0;
  if (key_x_slow > chassis_speed_max)
    key_x_slow = chassis_speed_max;
  if (key_x_slow < 0)
    key_x_slow = 0;
  if (key_y_fast > chassis_speed_max)
    key_y_fast = chassis_speed_max;
  if (key_y_fast < 0)
    key_y_fast = 0;
  if (key_y_slow > chassis_speed_max)
    key_y_slow = chassis_speed_max;
  if (key_y_slow < 0)
    key_y_slow = 0;
}