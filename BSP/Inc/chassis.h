#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

/* ────────── 10 状态枚举 ────────── */
typedef enum {
    STATE_DEBUG = 0,        // VOFA+ 手动调参
    STATE_IDLE,             // 空闲等待
    STATE_LINE_FOLLOW,      // 红外巡线行驶
    STATE_OBSTACLE,         // 超声波避障绕行
    STATE_GREEN_FILTER,     // 绿草莓抗干扰（保留，暂未用）
    STATE_BREAKAWAY,        // 脱离巡线 → 视觉伺服转向植株
    STATE_VISUAL_APPROACH,  // 双轴伺服前进靠近植株
    STATE_SCANNING,         // 静止扫描统计（固定窗口）
    STATE_DOCKING,          // 蜂鸣上报，采摘确认
    STATE_BLIND_RETURN,     // 倒退回赛道 + 回正
    STATE_FINISH,           // 任务完成
    STATE_COUNT             // 哨兵值 = 11
} ChassisState_t;

/* ────────── 传感器数据结构体（对齐 1 字节） ────────── */
#pragma pack(1)
typedef struct {
    uint8_t ir[8];          // PE0(最左) ~ PE7(最右), 0=黑线
    float   distance_cm;    // HC-SR04 超声波测距
} SensorData_t;

/* ── K210 视觉帧（v5：定长 24B 多目标批量帧） ── */
typedef struct {
    uint8_t cls;            // 0=绿草莓(未熟), 1=红草莓(成熟)
    uint8_t x;              // 水平坐标 0~255（K210 320→255 压缩）
    uint8_t y;              // 垂直坐标 0~240
    uint8_t area;           // 面积百分比 0~100
} VisionTarget_t;

typedef struct {
    uint8_t         count;      // 有效草莓数 0~5
    VisionTarget_t  t[5];       // 最多 5 个 Target，无效槽全 0
} VisionFrame_t;
#pragma pack()

/* ────────── 状态描述符表项 ────────── */
typedef struct {
    const char    *name;
    void         (*entry)(void);
    ChassisState_t (*run)(const SensorData_t *sensor,
                          uint8_t has_vision, const VisionFrame_t *vis);
    uint8_t        allow_voafa_t;   // 1=DEBUG之外也允许T命令
} StateDescriptor_t;

/* ────────── 全局变量声明 ────────── */
extern volatile uint8_t   g_emergency_stop;
extern ChassisState_t     g_chassis_state;
extern uint32_t           g_state_ticks;
extern uint16_t           g_red_count;       // 红草莓累计计数
extern uint16_t           g_green_count;     // 绿草莓累计计数
extern int32_t            g_enc_total;       // 四轮编码器累积均值（有符号，前+后-）
extern float              g_avg_speed_mps;   // 四轮平均速度 (m/s)，Task_OLED 读取
extern int32_t            g_turn_enc;        // 四轮编码器绝对值累加（用于精算转角）
extern uint32_t           g_sys_tick;        // 全局滴答（每 10ms +1），freertos.c 递增

/* ────────── 避障子阶段枚举（不变） ────────── */
typedef enum {
    OB_STOP = 0,
    OB_BACK,
    OB_TURN_RIGHT,
    OB_CROSS,
    OB_TURN_LEFT1,
    OB_FORWARD,
    OB_TURN_LEFT2,
    OB_RETURN,
    OB_TURN_45,
    OB_FORWARD_TO_LINE,
    OB_DONE
} ObsSubStage_t;

/* ────────── Phase 2 可调参数（巡线 + 避障，不变） ────────── */
#define LINE_KP             0.04f
#define LINE_BASE_SPEED     0.3f
#define OB_BACK_SPEED       0.2f
#define OB_FWD_SPEED        0.3f
#define OB_TURN_SPEED       0.15f
#define OB_TURN_MS          800
#define OB_BACK_CM          10.0f
#define OB_CROSS_CM         25.0f
#define OB_FORWARD_CM       30.0f
#define ENC_PER_CM          2707
#define TURN90_TOTAL_ENC    118200
#define TURN45_TOTAL_ENC    59100
#define OB_FWD_TO_LINE_CM   50.0f
#define OB_BRAKE_TIMEOUT    100
#define ENABLE_ULTRASONIC   1
#define FORCE_STRAIGHT      0

/* ────────── Phase 3 可调参数（草莓检测链 v5） ────────── */
// 触发
#define PLANT_X_LEFT        80      // X<此值=植株在左侧，触发
#define PLANT_X_RIGHT       200     // X>此值=植株在右侧，触发
#define TRIGGER_CONFIRM     3       // 连续确认帧数

// 视觉伺服
#define X_CENTER_PX         160     // 画面水平中心
#define Y_CENTER_PX         120     // 画面垂直中心
#define X_DEADBAND_PX       15      // X 对中死区（像素）
#define Y_DEADBAND_PX       10      // Y 对中死区（像素，舵机不动）
#define Y_FILT_NEW          0.3f    // Y 一阶低通新值权重
#define TURN_SERVO_SPEED    0.15f   // 原地视觉伺服转向速度 (m/s)

// 前进靠近
#define APPROACH_BASE_SPEED 0.20f   // 靠近初始速度 (m/s)
#define APPROACH_MIN_SPEED  0.08f   // 靠近最低速度 (m/s)
#define APPROACH_AREA_STOP  20      // Area≥此值可停车（%）
#define APPROACH_CENTER_OK  10      // X 居中连续稳定帧数

// 静止扫描
#define SCAN_FRAMES         50      // 采样窗口（50帧 = 0.5s @ 100Hz）

// 倒车回线
#define RETURN_SPEED        0.3f    // 倒车速度 (m/s)
#define RETURN_SPIN_SPEED   0.15f   // 找线自旋速度 (m/s)

// 防重触发
#define POST_DOCK_SUPPRESS_MS  2000 // 摘完一株后 N ms 内不触发（滴答 = 10ms）

/* ────────── Chassis_Move API（只设 Target，不碰 PWM） ────────── */
void Chassis_MoveForward(float speed_mps);
void Chassis_MoveBackward(float speed_mps);
void Chassis_Stop(void);
void Chassis_DiffTurn(float base_speed, float turn_diff);
void Chassis_TurnInPlace(float speed_mps, int8_t dir);
void Chassis_SetTargets(float fl, float fr, float bl, float br);

/* ────────── 状态机调度 ────────── */
void Chassis_Init(void);
void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionFrame_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br);
void Chassis_VOFA_Parse(const uint8_t *cmd_buf);

/* ────────── 舵机 Y 轴伺服（供 VISUAL_APPROACH 调用） ────────── */
void Servo_Y_Servo(float y_deviation_px);

#endif /* __CHASSIS_H */
