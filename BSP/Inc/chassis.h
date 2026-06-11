#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

/* ────────── 10 状态枚举 ────────── */
typedef enum {
    STATE_DEBUG = 0,        // VOFA+ 手动调参
    STATE_IDLE,             // 空闲等待
    STATE_LINE_FOLLOW,      // 红外巡线行驶
    STATE_OBSTACLE,         // 超声波避障绕行
    STATE_GREEN_FILTER,     // 绿草莓抗干扰
    STATE_BREAKAWAY,        // 脱离巡线，原地对准红草莓
    STATE_VISUAL_APPROACH,  // 视觉伺服接近
    STATE_DOCKING,          // 对接完成，采摘确认
    STATE_BLIND_RETURN,     // 盲倒回赛道 + 回正
    STATE_FINISH,           // 任务完成
    STATE_COUNT             // 哨兵值 = 10
} ChassisState_t;

/* ────────── 传感器数据结构体（对齐 1 字节） ────────── */
#pragma pack(1)
typedef struct {
    uint8_t ir[8];          // PE0(最左) ~ PE7(最右), 0=黑线
    float   distance_cm;    // HC-SR04 超声波测距
} SensorData_t;

typedef struct {
    uint8_t header;         // 0xAA
    uint8_t class_id;       // 0=绿草莓, 1=红草莓
    uint8_t x;              // X 坐标 (0~320)
    uint8_t y;              // Y 坐标 (0~240)
    uint8_t area;           // 面积百分比 (0~100)
    uint8_t reserved;
    uint8_t checksum;       // XOR(Class, X, Y, Area, Reserved)
    uint8_t tail;           // 0x55
} VisionData_t;
#pragma pack()

/* ────────── 状态描述符表项 ────────── */
typedef struct {
    const char    *name;
    void         (*entry)(void);
    ChassisState_t (*run)(const SensorData_t *sensor,
                          uint8_t has_vision, const VisionData_t *vis);
    uint8_t        allow_voafa_t;   // 1=DEBUG之外也允许T命令
} StateDescriptor_t;

/* ────────── 全局变量声明 ────────── */
extern volatile uint8_t   g_emergency_stop;
extern ChassisState_t     g_chassis_state;
extern uint32_t           g_state_ticks;
extern int32_t            g_last_picked_enc;
extern int32_t            g_skip_until_enc;
extern uint8_t            g_skip_strawberry;
extern uint16_t           g_red_count;       // 红草莓采摘计数
extern uint16_t           g_green_count;     // 绿草莓识别计数
extern int32_t            g_enc_total;       // 四轮编码器累积均值（有符号，前+后-）
extern float              g_avg_speed_mps;   // 四轮平均速度 (m/s)，Task_OLED 读取
extern int32_t            g_turn_enc;        // 四轮编码器绝对值累加（用于精算转角）

/* ────────── 避障子阶段枚举 ────────── */
typedef enum {
    OB_STOP = 0,        // 急停等四轮归零
    OB_BACK,            // 后退 10cm
    OB_TURN_RIGHT,      // 右转 90°
    OB_CROSS,           // 横行 25cm（记录编码器距离）
    OB_TURN_LEFT1,      // 左转 90°（恢复朝前）
    OB_FORWARD,         // 直行 30cm
    OB_TURN_LEFT2,      // 左转 90°（车头朝左，正对黑线）
    OB_RETURN,          // 回线 N cm（= 记忆的横行距离）
    OB_TURN_45,         // 右转 45° 斜向朝线
    OB_FORWARD_TO_LINE, // 直行直到红外发现黑线
    OB_DONE             // 哨兵：避障完成
} ObsSubStage_t;

/* ────────── Phase 2 可调参数 ────────── */
#define LINE_KP             0.04f   // 巡线 P 控制器比例系数
#define LINE_BASE_SPEED     0.3f    // 巡线基准速度 (m/s)
#define OB_BACK_SPEED       0.2f    // 避障后退速度 (m/s)
#define OB_FWD_SPEED        0.3f    // 避障前进速度 (m/s)
#define OB_TURN_SPEED       0.15f   // 避障原地旋转速度 (m/s) — 低速封印
#define OB_TURN_MS          800     // 避障转 90° 计时上限 (ms)，编码器优先
#define OB_BACK_CM          10.0f   // 后退距离 (cm)
#define OB_CROSS_CM         25.0f   // 横行绕过距离 (cm)
#define OB_FORWARD_CM       30.0f   // 直行超越距离 (cm)
#define SUPPRESS_CM         20.0f   // 抑制区距离 (cm)
#define ENC_PER_CM          2707    // 编码器计数/cm ≈ 270709/m ÷ 100
#define TURN90_TOTAL_ENC    118200  // 四轮绝对值累加 = 90°（轮距 13.9cm）
#define TURN45_TOTAL_ENC    59100   // 四轮绝对值累加 = 45°
#define OB_FWD_TO_LINE_CM   50.0f   // 斜向朝线直行最长距离 (cm)，红外兜底
#define OB_BRAKE_TIMEOUT    100     // 阶段间刹车最长等待 (10ms*100=1s)，四轮归零提前退出
#define ENABLE_ULTRASONIC   1       // 超声波使能: 1=接硬件, 0=空转测试禁用
#define FORCE_STRAIGHT      0       // 强制直行: 1=忽略红外差速, 0=正常巡线

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
                              uint8_t has_vision, const VisionData_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br);
void Chassis_VOFA_Parse(const uint8_t *cmd_buf);

#endif /* __CHASSIS_H */
